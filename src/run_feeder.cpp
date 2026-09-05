// external front-end feeder for msceqf: tracks features and injects
// TriangulatedFeatures directly, bypassing the internal detector/KLT.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "msceqf/msceqf.hpp"
#include "vision/tracker.hpp"

namespace
{
constexpr char kImageTopic[] = "/camera/image_raw";
constexpr char kImuTopic[] = "/imu/data";

double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1.0e-9 * static_cast<double>(t.nanosec);
}

}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr <<
      "usage: run_feeder <bag_dir> <config.yaml> <out.csv> [numeric options]\n"
      "\n"
      "The keyframe-gated front-end for MSCEqF. This is the ONE architecture the\n"
      "project settled on (2026-09-04): MSCEqF's own Tracker runs on every frame,\n"
      "each feature accumulates rotation-compensated parallax against its own\n"
      "reference, a frame is injected when enough features are ready, only the\n"
      "ready features go in, and only those get a fresh reference. MSCEqF itself\n"
      "is unmodified; it receives TriangulatedFeatures and does the rest.\n"
      "\n"
      "With no options this runs the deployment recipe. The options are numbers\n"
      "only -- there is no other mode to select.\n"
      "  --delta-px PX      per-feature parallax threshold (4)\n"
      "  --fire-frac F      ready-fraction that fires a frame (0.1)\n"
      "  --min-inject N     minimum ready features to fire (4)\n"
      "  --min-ref N        a frame with fewer than N referenced features injects\n"
      "                     NOTHING (0 = always drop such frames; N>0 force-injects)\n"
      "  --max-dt S         force an injection after S seconds without one (0.5).\n"
      "                     Not a tuning knob: MSCEqF's IMU buffer overflows without it.\n"
      "  --disp-log PATH    write per-frame gate statistics CSV\n"
      "  --track-log PATH   write (t,id) for every injected feature\n"
      "  --start S --duration D\n"
      "\n"
      "Removed 2026-09-04 (defeated alternatives, no longer selectable): --mode,\n"
      "--tracker, --trigger, --decim, --gate, --smooth, --gate-angle, --feat-delta-px.\n"
      "The every-frame path lives in run_offline.\n";
    return 1;
  }
  const std::string bag_dir = argv[1], config_path = argv[2], out_path = argv[3];

  // Defaults are the fixed values of the deployment recipe, so a bare invocation IS the recipe.
  double delta_px = 4.0, fire_frac = 0.1, max_dt = 0.5, start_s = 0.0, duration_s = -1.0;
  int min_inject = 4, min_ref = 0;
  std::string disp_log_path, track_log_path;
  for (int i = 4; i + 1 < argc; i += 2)
  {
    const std::string k = argv[i], v = argv[i + 1];
    if      (k == "--delta-px" || k == "--tau") delta_px = std::stod(v);   // --tau: alias for recorded commands
    else if (k == "--fire-frac")  fire_frac = std::stod(v);
    else if (k == "--min-inject") min_inject = std::stoi(v);
    else if (k == "--min-ref")    min_ref = std::stoi(v);
    else if (k == "--max-dt")     max_dt = std::stod(v);
    else if (k == "--disp-log")   disp_log_path = v;
    else if (k == "--track-log")  track_log_path = v;
    else if (k == "--start")      start_s = std::stod(v);
    else if (k == "--duration")   duration_s = std::stod(v);
    else if (k == "--mode" || k == "--tracker" || k == "--trigger" || k == "--decim" ||
             k == "--gate" || k == "--smooth" || k == "--gate-angle" ||
             k == "--feat-delta-px" || k == "--feat-tau")
    {
      std::cerr << "[feeder] " << k << " was removed on 2026-09-04: run_feeder has exactly one "
                   "architecture (feature-keyframe gate on MSCEqF's builtin tracker). Drop the flag; "
                   "for the every-frame path use run_offline.\n";
      return 1;
    }
    else { std::cerr << "unknown flag " << k << "\n"; return 1; }
  }

  // R_cam_imu: rotates gyro increments into the camera frame (parallax trigger).
  cv::Matx33d R_ci = cv::Matx33d::eye();
  {
    std::ifstream f(config_path);
    std::string line;
    bool in_block = false;
    int row = 0;
    while (std::getline(f, line) && row < 3)
    {
      if (line.rfind("T_cam_imu", 0) == 0) { in_block = true; continue; }
      if (!in_block) continue;
      const auto lb = line.find('[');
      if (lb == std::string::npos) break;
      std::string s = line.substr(lb);
      for (char& c : s) if (c == '[' || c == ']' || c == ',') c = ' ';
      std::istringstream iss(s);
      double a, b, c, d;
      if (iss >> a >> b >> c >> d) { R_ci(row, 0) = a; R_ci(row, 1) = b; R_ci(row, 2) = c; ++row; }
      else break;
    }
    if (row != 3) std::cerr << "[feeder] WARNING: T_cam_imu not parsed; rotation compensation disabled\n";
  }

  msceqf::MSCEqF sys(config_path);

  const auto& mopts = sys.options();
  const auto& topts = mopts.track_manager_options_.tracker_options_;
  // MSCEqF's own Tracker, linked as a class: the front-end is engine code, not a
  // reimplementation. Only the injection decision below is ours.
  auto builtin = std::make_unique<msceqf::Tracker>(
      topts, mopts.state_options_.initial_camera_intrinsics_.k());
  const double bfx = mopts.state_options_.initial_camera_intrinsics_.k()(0);
  const double bfy = mopts.state_options_.initial_camera_intrinsics_.k()(1);

  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rosbag2_cpp::Reader reader;
  reader.open(bag_dir);

  std::ofstream out(out_path);
  out << std::setprecision(17);
  out << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz,"
         "P00,P11,P22,P33,P44,P55,P66,P77,P88,"
         "sx,sy,sz,sqx,sqy,sqz,sqw,"
         "Pe0,Pe1,Pe2,Pe3,Pe4,Pe5\n";

  std::unordered_map<unsigned, cv::Point2f> snap;
  cv::Matx33d dR_frame = cv::Matx33d::eye();
  cv::Matx33d R_abs = cv::Matx33d::eye();
  std::unordered_map<unsigned, cv::Matx33d> snap_R;
  double last_inject_t = -1, prev_imu_t = -1, first_img_t = -1, last_img_t = -1;
  double prev_frame_t = -1;
  std::ofstream track_log;
  if (!track_log_path.empty())
  {
    track_log.open(track_log_path);
    track_log << "t,id\n";
  }
  std::ofstream disp_log;
  if (!disp_log_path.empty())
  {
    disp_log.open(disp_log_path);
    disp_log << "t,n,q10,q25,q50,q75,q90,rot_rate,frac_over_delta,fired,n_inject\n";
  }
  int frame_idx = 0;

  std::vector<double> disp_at_inject;
  std::vector<double> gap_at_inject;
  std::vector<size_t> nfeat_at_inject;
  std::size_t n_img = 0, n_imu = 0, n_pose = 0, n_inject = 0;

  int64_t t0_ns = -1;
  while (reader.has_next())
  {
    auto msg = reader.read_next();
    if (t0_ns < 0) t0_ns = msg->time_stamp;
    const double rel = 1.0e-9 * static_cast<double>(msg->time_stamp - t0_ns);
    if (rel < start_s) continue;
    if (duration_s > 0.0 && rel > start_s + duration_s) break;

    rclcpp::SerializedMessage ser(*msg->serialized_data);

    if (msg->topic_name == kImuTopic)
    {
      sensor_msgs::msg::Imu m;
      imu_ser.deserialize_message(&ser, &m);
      msceqf::Imu imu;
      imu.timestamp_ = stampToSec(m.header.stamp);
      imu.ang_ << m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z;
      imu.acc_ << m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z;
      sys.processMeasurement(imu);
      ++n_imu;

      if (prev_imu_t > 0)
      {
        const double dt = imu.timestamp_ - prev_imu_t;
        if (dt > 0 && dt < 0.1)
        {
          const cv::Vec3d w(imu.ang_(0) * dt, imu.ang_(1) * dt, imu.ang_(2) * dt);
          cv::Matx33d dRb;
          cv::Rodrigues(w, dRb);
          dR_frame = dR_frame * dRb;
          R_abs = R_abs * dRb;
        }
      }
      prev_imu_t = imu.timestamp_;
      continue;
    }

    if (msg->topic_name != kImageTopic) continue;

    sensor_msgs::msg::Image m;
    img_ser.deserialize_message(&ser, &m);
    cv::Mat colour(m.height, m.width, CV_8UC3, m.data.data(), m.step);
    cv::Mat gray;
    cv::cvtColor(colour, gray, cv::COLOR_BGR2GRAY);
    const double t_cam = stampToSec(m.header.stamp);
    ++n_img;
    if (first_img_t < 0) first_img_t = t_cam;
    last_img_t = t_cam;

    bool produced = false;
    double emit_t = t_cam;

    {
      ++frame_idx;

      std::vector<cv::Point2f> xn, raw_uv, undist_uv;
      std::vector<unsigned> ids;
      const double fx_use = bfx, fy_use = bfy;

      {
        msceqf::Camera bcam;
        bcam.timestamp_ = t_cam;
        bcam.image_ = gray;
        if (topts.cam_options_.mask_type_ == msceqf::MaskType::STATIC)
        {
          // required: without the mask the builtin tracker detects nothing
          bcam.mask_ = topts.cam_options_.static_mask_;
        }
        builtin->processCamera(bcam);
        const auto& cf = builtin->currentFeatures().second;
        xn.assign(cf.normalized_uvs_.begin(), cf.normalized_uvs_.end());
        raw_uv.assign(cf.distorted_uvs_.begin(), cf.distorted_uvs_.end());
        undist_uv.assign(cf.uvs_.begin(), cf.uvs_.end());
        ids.assign(cf.ids_.begin(), cf.ids_.end());
      }

      bool fire = false;
      double med_disp = 0.0;
      std::vector<double> dfeat;
      std::vector<char> take;
      bool include_all = true;

      // ---- feature-keyframe gate (the adopted architecture) ----
      {
        dfeat.assign(ids.size(), -1.0);
        int n_ref = 0, n_over = 0;
        for (size_t i = 0; i < ids.size(); ++i)
        {
          auto it = snap.find(ids[i]);
          auto itR = snap_R.find(ids[i]);
          if (it == snap.end() || itR == snap_R.end()) continue;
          const cv::Matx33d dR_i = itR->second.t() * R_abs;
          const cv::Matx33d Rcam = R_ci * dR_i.t() * R_ci.t();
          const cv::Vec3d v0(it->second.x, it->second.y, 1.0);
          const cv::Vec3d vp = Rcam * v0;
          if (vp(2) < 1e-6) continue;
          const double du = fx_use * (xn[i].x - vp(0) / vp(2));
          const double dv = fy_use * (xn[i].y - vp(1) / vp(2));
          dfeat[i] = std::sqrt(du * du + dv * dv);
          ++n_ref;
          if (dfeat[i] >= delta_px) ++n_over;
        }
        if (n_ref)
        {
          std::vector<double> ds;
          ds.reserve(n_ref);
          for (double v : dfeat) if (v >= 0) ds.push_back(v);
          std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
          med_disp = ds[ds.size() / 2];
        }

        const bool have_ref = last_inject_t > 0;
        const double frac = n_ref ? double(n_over) / n_ref : 0.0;
        include_all = !have_ref || (min_ref > 0 && n_ref < min_ref) ||
                      (t_cam - last_inject_t) >= max_dt;
        fire = include_all || (frac >= fire_frac && n_over >= min_inject);

        take.assign(ids.size(), 0);
        for (size_t i = 0; i < ids.size(); ++i)
          take[i] = include_all ? 1 : (dfeat[i] >= delta_px ? 1 : 0);

        for (size_t i = 0; i < ids.size(); ++i)
          if (snap.find(ids[i]) == snap.end()) { snap[ids[i]] = xn[i]; snap_R[ids[i]] = R_abs; }
        {
          std::unordered_set<unsigned> live(ids.begin(), ids.end());
          for (auto it = snap.begin(); it != snap.end();)
            if (live.count(it->first)) ++it; else { snap_R.erase(it->first); it = snap.erase(it); }
        }

        if (disp_log.is_open() && n_ref >= 8)
        {
          cv::Vec3d rv;
          cv::Rodrigues(dR_frame, rv);
          const double dtf = prev_frame_t > 0 ? (t_cam - prev_frame_t) : 0.1;
          int n_take = 0;
          for (char c : take) n_take += c;
          disp_log << std::setprecision(15) << t_cam << ',' << n_ref << ",0,0,"
                   << med_disp << ",0,0," << cv::norm(rv) / dtf << ',' << frac << ','
                   << (fire ? 1 : 0) << ',' << (fire ? n_take : 0) << '\n';
        }
      }
      dR_frame = cv::Matx33d::eye();
      prev_frame_t = t_cam;

      if (fire && !ids.empty())
      {
        emit_t = t_cam;

        msceqf::TriangulatedFeatures tf;
        tf.timestamp_ = emit_t;
        for (size_t i = 0; i < ids.size(); ++i)
        {
          if (!take.empty() && !take[i]) continue;
          cv::Point2f n = xn[i];
          const cv::Point2f uv = undist_uv[i];
          tf.features_.distorted_uvs_.push_back(raw_uv[i]);
          tf.features_.uvs_.push_back(uv);
          tf.features_.normalized_uvs_.push_back(n);
          tf.features_.ids_.push_back(ids[i]);
          tf.points_.emplace_back(msceqf::Vector3::Zero());  // unread; sized to match features_
          if (track_log.is_open())
            track_log << std::setprecision(17) << emit_t << ',' << ids[i] << '\n';
        }
        sys.processMeasurement(tf);
        // processMeasurement shifts the stamp by timeshift_cam_imu internally
        emit_t = tf.timestamp_;
        produced = true;
        ++n_inject;

        if (last_inject_t > 0)
        {
          gap_at_inject.push_back(t_cam - last_inject_t);
          disp_at_inject.push_back(med_disp);
        }
        nfeat_at_inject.push_back(ids.size());
        last_inject_t = t_cam;
        // keyframe rule: only injected features get a fresh reference; the rest keep
        // accumulating parallax against their old one.
        for (size_t i = 0; i < ids.size(); ++i)
          if (take[i]) { snap[ids[i]] = xn[i]; snap_R[ids[i]] = R_abs; }
      }
    }

    if (produced && sys.isInit())
    {
      const auto est = sys.stateEstimate();
      const auto cov = sys.covariance();
      const auto& q = est.T().q();
      const auto& p = est.T().p();
      const auto& v = est.T().v();
      const auto& b = est.b();
      out << emit_t << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
          << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
          << v.x() << ',' << v.y() << ',' << v.z();
      for (int i = 0; i < 6; ++i) out << ',' << b(i);
      for (int i = 0; i < 9; ++i) out << ',' << cov(i, i);
      const auto& S = est.S();
      const auto& sp = S.x();
      const auto& sq = S.q();
      out << ',' << sp.x() << ',' << sp.y() << ',' << sp.z()
          << ',' << sq.x() << ',' << sq.y() << ',' << sq.z() << ',' << sq.w();
      for (int i = 15; i < 21 && i < cov.rows(); ++i) out << ',' << cov(i, i);
      out << '\n';
      ++n_pose;
    }
  }
  out.close();

  auto med = [](std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
    return v[v.size() / 2];
  };
  const double span = (last_img_t > first_img_t) ? (last_img_t - first_img_t) : 0.0;
  const double mean_rate = span > 0 ? static_cast<double>(n_inject) / span : 0.0;
  double mean_feat = 0;
  for (auto n : nfeat_at_inject) mean_feat += static_cast<double>(n);
  if (!nfeat_at_inject.empty()) mean_feat /= static_cast<double>(nfeat_at_inject.size());

  std::cerr << "[feeder] delta_px=" << delta_px << " fire_frac=" << fire_frac
            << " min_inject=" << min_inject << " min_ref=" << min_ref << " max_dt=" << max_dt
            << " | images=" << n_img << " imu=" << n_imu
            << " injections=" << n_inject << " poses=" << n_pose
            << " | mean rate=" << mean_rate << " Hz"
            << " (median gap " << med(gap_at_inject) << " s)"
            << " median parallax=" << med(disp_at_inject) << " px"
            << " mean features=" << mean_feat
            << " -> " << out_path << std::endl;
  return 0;
}
