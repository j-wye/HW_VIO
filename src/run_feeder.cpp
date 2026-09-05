// Offline runner with the keyframe-gated front-end. Reads a rosbag2 in timestamp order and
// drives VioPipeline in one thread -- the same pipeline vio_node runs live, so replaying a
// bag through the node gives the same estimates as this runner.

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "vio_node/pipeline.hpp"

namespace
{
constexpr char kImageTopic[] = "/camera/image_raw";
constexpr char kImuTopic[] = "/imu/data";

double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1.0e-9 * static_cast<double>(t.nanosec);
}

double median(std::vector<double> v)
{
  if (v.empty()) return 0.0;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cerr <<
      "usage: run_feeder <bag_dir> <config.yaml> <out.csv> [options]\n"
      "\n"
      "Keyframe-gated front-end: the engine's own tracker runs on every frame, each\n"
      "feature accumulates rotation-compensated parallax against its own reference, a\n"
      "frame is injected when enough features are ready, only the ready features go in,\n"
      "and only those get a fresh reference.\n"
      "\n"
      "Gate values come from the `frontend:` block of config.yaml if present, otherwise\n"
      "from the built-in defaults (the deployment recipe). Options override both:\n"
      "  --delta-px PX      per-feature parallax threshold (4)\n"
      "  --fire-frac F      ready-fraction that fires a frame (0.1)\n"
      "  --min-inject N     minimum ready features to fire (4)\n"
      "  --min-ref N        a frame with fewer than N referenced features injects\n"
      "                     NOTHING (0 = always drop such frames; N>0 force-injects)\n"
      "  --max-dt S         force an injection after S seconds without one (0.5).\n"
      "                     Not a tuning knob: the IMU buffer overflows without it.\n"
      "  --disp-log PATH    write per-frame gate statistics CSV\n"
      "  --track-log PATH   write (t,id) for every injected feature\n"
      "  --start S --duration D\n"
      "The every-frame path (engine front-end) lives in run_offline.\n";
    return 1;
  }
  const std::string bag_dir = argv[1], config_path = argv[2], out_path = argv[3];

  vio::GateParams gp;
  std::string err;
  vio::VioPipeline::loadGateParams(config_path, gp, err);
  if (!err.empty())
  {
    std::cerr << "[feeder] " << err << "\n";
    return 1;
  }

  double start_s = 0.0, duration_s = -1.0;
  std::string disp_log_path, track_log_path;
  for (int i = 4; i + 1 < argc; i += 2)
  {
    const std::string k = argv[i], v = argv[i + 1];
    if (k == "--delta-px")         gp.delta_px = std::stod(v);
    else if (k == "--fire-frac")   gp.fire_frac = std::stod(v);
    else if (k == "--min-inject")  gp.min_inject = std::stoi(v);
    else if (k == "--min-ref")     gp.min_ref = std::stoi(v);
    else if (k == "--max-dt")      gp.max_dt = std::stod(v);
    else if (k == "--start")       start_s = std::stod(v);
    else if (k == "--duration")    duration_s = std::stod(v);
    else if (k == "--disp-log")    disp_log_path = v;
    else if (k == "--track-log")   track_log_path = v;
    else
    {
      std::cerr << "[feeder] unknown option " << k << " (see usage; options take a value)\n";
      return 1;
    }
  }

  vio::VioPipeline pipe(config_path, gp);

  std::ofstream track_log, disp_log;
  if (!track_log_path.empty())
  {
    track_log.open(track_log_path);
    track_log << "t,id\n";
    pipe.frontend().setTrackLog(&track_log);
  }
  if (!disp_log_path.empty())
  {
    disp_log.open(disp_log_path);
    disp_log << "t,n,q10,q25,q50,q75,q90,rot_rate,frac_over_delta,fired,n_inject\n";
    pipe.frontend().setDispLog(&disp_log);
  }

  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;
  rosbag2_cpp::Reader reader;
  reader.open(bag_dir);

  std::ofstream out(out_path);
  out << std::setprecision(17);
  vio::VioPipeline::csvHeader(out);

  std::size_t n_imu = 0, n_pose = 0;
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
      pipe.processImu(imu);
      ++n_imu;
      continue;
    }
    if (msg->topic_name != kImageTopic) continue;

    sensor_msgs::msg::Image m;
    img_ser.deserialize_message(&ser, &m);
    const cv::Mat gray = vio::VioPipeline::toGray(m.height, m.width, m.encoding, m.data.data(), m.step);
    if (pipe.processImage(stampToSec(m.header.stamp), gray))
    {
      pipe.csvRow(out);
      ++n_pose;
    }
  }
  out.close();

  const auto& st = pipe.frontend().stats();
  const auto& p = pipe.frontend().params();
  const double span = (st.last_img_t > st.first_img_t) ? (st.last_img_t - st.first_img_t) : 0.0;
  const double mean_rate = span > 0 ? static_cast<double>(st.n_inject) / span : 0.0;
  double mean_feat = 0;
  for (auto n : st.nfeat_at_inject) mean_feat += static_cast<double>(n);
  if (!st.nfeat_at_inject.empty()) mean_feat /= static_cast<double>(st.nfeat_at_inject.size());

  std::cerr << "[feeder] delta_px=" << p.delta_px << " fire_frac=" << p.fire_frac
            << " min_inject=" << p.min_inject << " min_ref=" << p.min_ref << " max_dt=" << p.max_dt
            << " | images=" << st.n_img << " imu=" << n_imu
            << " injections=" << st.n_inject << " poses=" << n_pose
            << " | mean rate=" << mean_rate << " Hz"
            << " (median gap " << median(st.gap_at_inject) << " s)"
            << " median parallax=" << median(st.disp_at_inject) << " px"
            << " mean features=" << mean_feat
            << " -> " << out_path << std::endl;
  return 0;
}
