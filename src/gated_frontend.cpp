#include "vio_node/gated_frontend.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <unordered_set>

#include <opencv2/calib3d.hpp>

namespace vio
{

GatedFrontend::GatedFrontend(const msceqf::MSCEqFOptions& opts, const cv::Matx33d& R_cam_imu,
                             const GateParams& params)
    : params_(params),
      R_ci_(R_cam_imu),
      fx_(opts.state_options_.initial_camera_intrinsics_.k()(0)),
      fy_(opts.state_options_.initial_camera_intrinsics_.k()(1)),
      topts_(opts.track_manager_options_.tracker_options_),
      tracker_(topts_, opts.state_options_.initial_camera_intrinsics_.k())
{
}

void GatedFrontend::feedImu(const msceqf::Imu& imu)
{
  if (prev_imu_t_ > 0)
  {
    const double dt = imu.timestamp_ - prev_imu_t_;
    if (dt > 0 && dt < 0.1)
    {
      const cv::Vec3d w(imu.ang_(0) * dt, imu.ang_(1) * dt, imu.ang_(2) * dt);
      cv::Matx33d dRb;
      cv::Rodrigues(w, dRb);
      dR_frame_ = dR_frame_ * dRb;
      R_abs_ = R_abs_ * dRb;
    }
  }
  prev_imu_t_ = imu.timestamp_;
}

bool GatedFrontend::processImage(double t_cam, const cv::Mat& gray, msceqf::TriangulatedFeatures& tf)
{
  ++stats_.n_img;
  if (stats_.first_img_t < 0) stats_.first_img_t = t_cam;
  stats_.last_img_t = t_cam;

  std::vector<cv::Point2f> xn, raw_uv, undist_uv;
  std::vector<unsigned> ids;
  {
    msceqf::Camera cam;
    cam.timestamp_ = t_cam;
    cam.image_ = gray;
    if (topts_.cam_options_.mask_type_ == msceqf::MaskType::STATIC)
    {
      // without the mask the tracker detects nothing
      cam.mask_ = topts_.cam_options_.static_mask_;
    }
    tracker_.processCamera(cam);
    const auto& cf = tracker_.currentFeatures().second;
    xn.assign(cf.normalized_uvs_.begin(), cf.normalized_uvs_.end());
    raw_uv.assign(cf.distorted_uvs_.begin(), cf.distorted_uvs_.end());
    undist_uv.assign(cf.uvs_.begin(), cf.uvs_.end());
    ids.assign(cf.ids_.begin(), cf.ids_.end());
  }

  // parallax per feature against its own reference, rotation compensated
  std::vector<double> dfeat(ids.size(), -1.0);
  int n_ref = 0, n_over = 0;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    auto it = snap_.find(ids[i]);
    auto itR = snap_R_.find(ids[i]);
    if (it == snap_.end() || itR == snap_R_.end()) continue;
    const cv::Matx33d dR_i = itR->second.t() * R_abs_;
    const cv::Matx33d Rcam = R_ci_ * dR_i.t() * R_ci_.t();
    const cv::Vec3d v0(it->second.x, it->second.y, 1.0);
    const cv::Vec3d vp = Rcam * v0;
    if (vp(2) < 1e-6) continue;
    const double du = fx_ * (xn[i].x - vp(0) / vp(2));
    const double dv = fy_ * (xn[i].y - vp(1) / vp(2));
    dfeat[i] = std::sqrt(du * du + dv * dv);
    ++n_ref;
    if (dfeat[i] >= params_.delta_px) ++n_over;
  }
  double med_disp = 0.0;
  if (n_ref)
  {
    std::vector<double> ds;
    ds.reserve(n_ref);
    for (double v : dfeat) if (v >= 0) ds.push_back(v);
    std::nth_element(ds.begin(), ds.begin() + ds.size() / 2, ds.end());
    med_disp = ds[ds.size() / 2];
  }

  const bool have_ref = last_inject_t_ > 0;
  const double frac = n_ref ? double(n_over) / n_ref : 0.0;
  const bool include_all = !have_ref || (params_.min_ref > 0 && n_ref < params_.min_ref) ||
                           (t_cam - last_inject_t_) >= params_.max_dt;
  const bool fire = include_all || (frac >= params_.fire_frac && n_over >= params_.min_inject);

  std::vector<char> take(ids.size(), 0);
  for (size_t i = 0; i < ids.size(); ++i)
    take[i] = include_all ? 1 : (dfeat[i] >= params_.delta_px ? 1 : 0);

  for (size_t i = 0; i < ids.size(); ++i)
    if (snap_.find(ids[i]) == snap_.end()) { snap_[ids[i]] = xn[i]; snap_R_[ids[i]] = R_abs_; }
  {
    std::unordered_set<unsigned> live(ids.begin(), ids.end());
    for (auto it = snap_.begin(); it != snap_.end();)
      if (live.count(it->first)) ++it; else { snap_R_.erase(it->first); it = snap_.erase(it); }
  }

  if (disp_log_ && n_ref >= 8)
  {
    cv::Vec3d rv;
    cv::Rodrigues(dR_frame_, rv);
    const double dtf = prev_frame_t_ > 0 ? (t_cam - prev_frame_t_) : 0.1;
    int n_take = 0;
    for (char c : take) n_take += c;
    *disp_log_ << std::setprecision(15) << t_cam << ',' << n_ref << ",0,0,"
               << med_disp << ",0,0," << cv::norm(rv) / dtf << ',' << frac << ','
               << (fire ? 1 : 0) << ',' << (fire ? n_take : 0) << '\n';
  }
  dR_frame_ = cv::Matx33d::eye();
  prev_frame_t_ = t_cam;

  if (!(fire && !ids.empty())) return false;

  tf = msceqf::TriangulatedFeatures();
  tf.timestamp_ = t_cam;
  for (size_t i = 0; i < ids.size(); ++i)
  {
    if (!take[i]) continue;
    tf.features_.distorted_uvs_.push_back(raw_uv[i]);
    tf.features_.uvs_.push_back(undist_uv[i]);
    tf.features_.normalized_uvs_.push_back(xn[i]);
    tf.features_.ids_.push_back(ids[i]);
    tf.points_.emplace_back(msceqf::Vector3::Zero());  // unread; sized to match features_
    if (track_log_) *track_log_ << std::setprecision(17) << t_cam << ',' << ids[i] << '\n';
  }

  ++stats_.n_inject;
  if (last_inject_t_ > 0)
  {
    stats_.gap_at_inject.push_back(t_cam - last_inject_t_);
    stats_.disp_at_inject.push_back(med_disp);
  }
  stats_.nfeat_at_inject.push_back(ids.size());
  last_inject_t_ = t_cam;
  // keyframe rule: only the injected features get a fresh reference
  for (size_t i = 0; i < ids.size(); ++i)
    if (take[i]) { snap_[ids[i]] = xn[i]; snap_R_[ids[i]] = R_abs_; }
  return true;
}

}
