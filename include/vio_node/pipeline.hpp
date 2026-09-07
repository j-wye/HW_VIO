#pragma once

#include <ostream>
#include <string>

#include <opencv2/core.hpp>

#include "msceqf/msceqf.hpp"
#include "vio_node/gated_frontend.hpp"

namespace vio
{

// Filter + gated front-end, driven in timestamp order. Shared by the node and the runner.
class VioPipeline
{
 public:
  VioPipeline(const std::string& config_path, const GateParams& gate);

  // Reads the optional `frontend:` block into `p`. Unknown keys inside it are an error:
  // a misspelt key must not silently fall back to a default.
  static bool loadGateParams(const std::string& config_path, GateParams& p, std::string& err);

  void processImu(const msceqf::Imu& imu);

  // True when the gate fired and the filter produced a new estimate.
  bool processImage(double t_cam, const cv::Mat& gray);

  static cv::Mat toGray(int height, int width, const std::string& encoding, const unsigned char* data,
                        std::size_t step);

  const msceqf::MSCEqF& sys() const { return sys_; }
  msceqf::MSCEqF& sys() { return sys_; }
  GatedFrontend& frontend() { return fe_; }
  const GatedFrontend& frontend() const { return fe_; }

  // injection time + timeshift_cam_imu
  double lastEmitTime() const { return last_emit_t_; }

  // False when the engine discarded the frame. Only valid right after processImage() == true.
  bool lastAccepted() const { return last_accepted_; }

  static void csvHeader(std::ostream& os);
  void csvRow(std::ostream& os) const;

 private:
  static cv::Matx33d loadRotationCamImu(const std::string& config_path, std::string& err);

  msceqf::MSCEqF sys_;
  GatedFrontend fe_;
  double last_emit_t_ = -1.0;
  bool last_accepted_ = false;
  std::string extrinsic_err_;
};

}
