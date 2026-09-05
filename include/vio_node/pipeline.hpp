#pragma once

#include <ostream>
#include <string>

#include <opencv2/core.hpp>

#include "msceqf/msceqf.hpp"
#include "vio_node/gated_frontend.hpp"

namespace vio
{

// Filter + gated front-end, driven in timestamp order. Both the ROS node and the offline
// runner use this class, so a bag replayed through the node produces the same estimates
// as run_feeder does on the same bag.
class VioPipeline
{
 public:
  VioPipeline(const std::string& config_path, const GateParams& gate);

  // Reads the optional `frontend:` block of a config.yaml into `p` (fields not listed keep
  // their current value). Returns true if the block exists. Unknown keys inside the block
  // are an error (`err` is set) -- a misspelt key must not silently fall back to a default.
  static bool loadGateParams(const std::string& config_path, GateParams& p, std::string& err);

  void processImu(const msceqf::Imu& imu);

  // Tracks the frame, injects it if the gate fires. Returns true when the filter was updated
  // and is initialised, i.e. a new estimate is available (see lastEmitTime / csvRow).
  bool processImage(double t_cam, const cv::Mat& gray);

  // Grayscale conversion used by every input path (bag reader and ROS subscriber alike).
  static cv::Mat toGray(int height, int width, const std::string& encoding, const unsigned char* data,
                        std::size_t step);

  const msceqf::MSCEqF& sys() const { return sys_; }
  msceqf::MSCEqF& sys() { return sys_; }
  GatedFrontend& frontend() { return fe_; }
  const GatedFrontend& frontend() const { return fe_; }

  // Timestamp of the last update as seen by the filter (injection time + timeshift_cam_imu).
  double lastEmitTime() const { return last_emit_t_; }

  static void csvHeader(std::ostream& os);
  void csvRow(std::ostream& os) const;

 private:
  static cv::Matx33d loadRotationCamImu(const std::string& config_path);

  msceqf::MSCEqF sys_;
  GatedFrontend fe_;
  double last_emit_t_ = -1.0;
};

}  // namespace vio
