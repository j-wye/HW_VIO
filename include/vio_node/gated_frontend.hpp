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

// Gate parameters. The defaults are the deployment recipe; a config.yaml may override
// them in a `frontend:` block (see VioPipeline::loadGateParams).
struct GateParams
{
  double delta_px = 4.0;   // per-feature parallax threshold (px)
  double fire_frac = 0.1;  // fraction of referenced features over delta_px that fires a frame
  int min_inject = 4;      // minimum number of ready features to fire
  int min_ref = 0;         // a frame with fewer than this many referenced features injects nothing
                           // (0 = always drop such frames; N > 0 force-injects them)
  double max_dt = 0.5;     // force an injection after this many seconds without one
                           // (not a tuning knob: the filter's IMU buffer overflows without it)
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

// Keyframe-gated front-end.
//
// The engine's own Tracker runs on every frame. Each tracked feature accumulates
// rotation-compensated parallax against its own reference observation; a frame is
// injected into the filter when enough features are "ready" (over delta_px), only the
// ready features go in, and only those get a fresh reference. Between injections the
// filter only propagates, so the update rate follows the motion, not the camera rate.
class GatedFrontend
{
 public:
  GatedFrontend(const msceqf::MSCEqFOptions& opts, const cv::Matx33d& R_cam_imu, const GateParams& params);

  // Integrates the gyro increment (rotation compensation for the parallax test).
  void feedImu(const msceqf::Imu& imu);

  // Tracks the frame and decides whether to inject. Returns true when the frame fires,
  // with `tf` filled with the features to hand to the filter (tf.timestamp_ = t_cam).
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

}  // namespace vio
