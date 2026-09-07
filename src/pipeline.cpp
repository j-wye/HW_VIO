#include "vio_node/pipeline.hpp"

#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace vio
{

cv::Matx33d VioPipeline::loadRotationCamImu(const std::string& config_path, std::string& err)
{
  // Same precedence as the engine's parser: T_imu_cam wins and is used as given, T_cam_imu is
  // inverted. The gate needs cam <- imu, so T_imu_cam's rotation block is transposed here.
  cv::Matx33d R = cv::Matx33d::eye();
  auto read3x3 = [&R](const YAML::Node& T, bool transpose) {
    if (!T || !T.IsSequence() || T.size() < 3) return false;
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R(transpose ? c : r, transpose ? r : c) = T[r][c].as<double>();
    return true;
  };
  try
  {
    const YAML::Node root = YAML::LoadFile(config_path);
    if (read3x3(root["T_imu_cam"], true)) return R;
    if (read3x3(root["T_cam_imu"], false)) return R;
  }
  catch (const std::exception& e)
  {
    err = std::string("cannot read the camera-IMU extrinsic from ") + config_path + ": " + e.what();
    return R;
  }
  err = "neither T_imu_cam nor T_cam_imu found in " + config_path;
  return R;
}

bool VioPipeline::loadGateParams(const std::string& config_path, GateParams& p, std::string& err)
{
  err.clear();
  YAML::Node root;
  try
  {
    root = YAML::LoadFile(config_path);
  }
  catch (const std::exception& e)
  {
    err = std::string("cannot read ") + config_path + ": " + e.what();
    return false;
  }
  const YAML::Node fe = root["frontend"];
  if (!fe) return false;
  if (!fe.IsMap())
  {
    err = "frontend: must be a map";
    return false;
  }
  static const std::set<std::string> known = {"delta_px", "fire_frac", "min_inject", "min_ref", "max_dt"};
  for (const auto& kv : fe)
  {
    const std::string k = kv.first.as<std::string>();
    if (!known.count(k))
    {
      err = "frontend: unknown key '" + k + "' (known: delta_px fire_frac min_inject min_ref max_dt)";
      return false;
    }
  }
  try
  {
    if (fe["delta_px"]) p.delta_px = fe["delta_px"].as<double>();
    if (fe["fire_frac"]) p.fire_frac = fe["fire_frac"].as<double>();
    if (fe["min_inject"]) p.min_inject = fe["min_inject"].as<int>();
    if (fe["min_ref"]) p.min_ref = fe["min_ref"].as<int>();
    if (fe["max_dt"]) p.max_dt = fe["max_dt"].as<double>();
  }
  catch (const std::exception& e)
  {
    err = std::string("frontend: bad value: ") + e.what();
    return false;
  }
  return true;
}

VioPipeline::VioPipeline(const std::string& config_path, const GateParams& gate)
    : sys_(config_path), fe_(sys_.options(), loadRotationCamImu(config_path, extrinsic_err_), gate)
{
  if (!extrinsic_err_.empty())
    throw std::runtime_error(extrinsic_err_ + " -- the gate cannot compensate rotation without it");
  // Both drivers of this class feed every IMU sample stamped up to the frame before the frame
  // itself. The engine shifts the frame stamp by timeshift_cam_imu and propagates to it, so a
  // positive shift would target a time past every sample it was given.
  const double ts = sys_.options().track_manager_options_.tracker_options_.cam_options_.timeshift_cam_imu_;
  if (ts > 0.0)
    throw std::runtime_error("timeshift_cam_imu must be <= 0 for this pipeline (got " + std::to_string(ts) + ")");
}

void VioPipeline::processImu(const msceqf::Imu& imu)
{
  sys_.processMeasurement(imu);
  fe_.feedImu(imu);
}

bool VioPipeline::processImage(double t_cam, const cv::Mat& gray)
{
  msceqf::TriangulatedFeatures tf;
  if (!fe_.processImage(t_cam, gray, tf)) return false;
  const double filter_t_before = sys_.timestamp();
  sys_.processMeasurement(tf);
  // processMeasurement shifts the stamp by timeshift_cam_imu internally
  last_emit_t_ = tf.timestamp_;
  // The engine returns without touching the state when it discards a measurement (stamp older
  // than the state, propagation failure). Its clock only moves on an accepted one.
  last_accepted_ = sys_.isInit() && sys_.timestamp() > filter_t_before;
  return sys_.isInit();
}

cv::Mat VioPipeline::toGray(int height, int width, const std::string& encoding, const unsigned char* data,
                            std::size_t step)
{
  cv::Mat gray;
  auto wrap = [&](int type) { return cv::Mat(height, width, type, const_cast<unsigned char*>(data), step); };
  if (encoding == "mono8")
  {
    wrap(CV_8UC1).copyTo(gray);
  }
  else if (encoding == "bgr8")
  {
    cv::cvtColor(wrap(CV_8UC3), gray, cv::COLOR_BGR2GRAY);
  }
  else if (encoding == "rgb8")
  {
    cv::cvtColor(wrap(CV_8UC3), gray, cv::COLOR_RGB2GRAY);
  }
  else if (encoding == "bgra8")
  {
    cv::cvtColor(wrap(CV_8UC4), gray, cv::COLOR_BGRA2GRAY);
  }
  else if (encoding == "rgba8")
  {
    cv::cvtColor(wrap(CV_8UC4), gray, cv::COLOR_RGBA2GRAY);
  }
  else
  {
    // reinterpreting an unknown layout as packed BGR silently corrupts every pixel
    throw std::invalid_argument("unsupported image encoding '" + encoding + "'");
  }
  return gray;
}

void VioPipeline::csvHeader(std::ostream& os)
{
  os << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz,"
        "P00,P11,P22,P33,P44,P55,P66,P77,P88,"
        "sx,sy,sz,sqx,sqy,sqz,sqw,"
        "Pe0,Pe1,Pe2,Pe3,Pe4,Pe5\n";
}

void VioPipeline::csvRow(std::ostream& os) const
{
  const auto est = sys_.stateEstimate();
  const auto cov = sys_.covariance();
  const auto& q = est.T().q();
  const auto& p = est.T().p();
  const auto& v = est.T().v();
  const auto& b = est.b();
  os << last_emit_t_ << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
     << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
     << v.x() << ',' << v.y() << ',' << v.z();
  for (int i = 0; i < 6; ++i) os << ',' << b(i);
  for (int i = 0; i < 9; ++i) os << ',' << cov(i, i);
  const auto& S = est.S();
  const auto& sp = S.x();
  const auto& sq = S.q();
  os << ',' << sp.x() << ',' << sp.y() << ',' << sp.z()
     << ',' << sq.x() << ',' << sq.y() << ',' << sq.z() << ',' << sq.w();
  for (int i = 15; i < 21 && i < cov.rows(); ++i) os << ',' << cov(i, i);
  os << '\n';
}

}  // namespace vio
