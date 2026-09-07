#pragma once

#include <cstddef>
#include <ostream>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include "msceqf/msceqf.hpp"
#include "vision/tracker.hpp"

namespace vio
{

// Defaults are the deployment recipe; a config.yaml `frontend:` block overrides them.
struct GateParams
{
  double delta_px = 4.0;   // per-feature parallax threshold (px)
  double fire_frac = 0.1;  // fraction of referenced features over delta_px that fires a frame
  int min_inject = 4;      // minimum number of ready features to fire
  int min_ref = 0;         // below this many referenced features: drop the frame (N>0 force-injects)
  double max_dt = 0.5;     // force an injection after this long (the IMU buffer overflows without it)
};

struct GateStats
{
  std::size_t n_img = 0;
  std::size_t n_inject = 0;
  double first_img_t = -1.0;
  double last_img_t = -1.0;
  std::vector<double> gap_at_inject;     // seconds since the previous injection
  std::vector<double> disp_at_inject;    // median parallax (px) at injection
  std::vector<std::size_t> nfeat_at_inject;
};

// Keyframe-gated front-end. The engine's Tracker runs on every frame; each feature
// accumulates rotation-compensated parallax against its own reference, and a frame is
// injected once enough features are over delta_px. Only the ready features go in, and only
// those get a fresh reference, so the update rate follows the motion, not the camera rate.
class GatedFrontend
{
 public:
  GatedFrontend(const msceqf::MSCEqFOptions& opts, const cv::Matx33d& R_cam_imu, const GateParams& params);

  void feedImu(const msceqf::Imu& imu);

  // True when the frame fires, with `tf` filled for the filter (tf.timestamp_ = t_cam).
  bool processImage(double t_cam, const cv::Mat& gray, msceqf::TriangulatedFeatures& tf);

  void setDispLog(std::ostream* os) { disp_log_ = os; }
  void setTrackLog(std::ostream* os) { track_log_ = os; }

  const GateParams& params() const { return params_; }
  const GateStats& stats() const { return stats_; }

 private:
  GateParams params_;
  cv::Matx33d R_ci_;
  double fx_, fy_;
  const msceqf::TrackerOptions& topts_;
  msceqf::Tracker tracker_;

  std::unordered_map<unsigned, cv::Point2f> snap_;    // feature id -> reference normalized uv
  std::unordered_map<unsigned, cv::Matx33d> snap_R_;  // feature id -> body rotation at reference
  cv::Matx33d dR_frame_ = cv::Matx33d::eye();
  cv::Matx33d R_abs_ = cv::Matx33d::eye();
  double last_inject_t_ = -1.0;
  double prev_imu_t_ = -1.0;
  double prev_frame_t_ = -1.0;

  std::ostream* disp_log_ = nullptr;
  std::ostream* track_log_ = nullptr;
  GateStats stats_;
};

}
