#include "vio_node/pipeline.hpp"

#include <iomanip>
#include <iostream>
#include <set>
#include <stdexcept>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

namespace vio
{

cv::Matx33d VioPipeline::loadRotationCamImu(const std::string& config_path)
{
  cv::Matx33d R = cv::Matx33d::eye();
  try
  {
    const YAML::Node root = YAML::LoadFile(config_path);
    const YAML::Node T = root["T_cam_imu"];
    if (T && T.IsSequence() && T.size() >= 3)
    {
      for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) R(r, c) = T[r][c].as<double>();
      return R;
    }
  }
  catch (const std::exception&)
  {
  }
  std::cerr << "[frontend] WARNING: T_cam_imu not parsed; rotation compensation disabled\n";
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
    : sys_(config_path), fe_(sys_.options(), loadRotationCamImu(config_path), gate)
{
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
  sys_.processMeasurement(tf);
  // processMeasurement shifts the stamp by timeshift_cam_imu internally
  last_emit_t_ = tf.timestamp_;
  return sys_.isInit();
}

cv::Mat VioPipeline::toGray(int height, int width, const std::string& encoding, const unsigned char* data,
                            std::size_t step)
{
  cv::Mat gray;
  if (encoding == "mono8")
  {
    cv::Mat(height, width, CV_8UC1, const_cast<unsigned char*>(data), step).copyTo(gray);
  }
  else if (encoding == "rgb8")
  {
    cv::Mat colour(height, width, CV_8UC3, const_cast<unsigned char*>(data), step);
    cv::cvtColor(colour, gray, cv::COLOR_RGB2GRAY);
  }
  else  // bgr8 (the dataset encoding) and anything else 3-channel
  {
    cv::Mat colour(height, width, CV_8UC3, const_cast<unsigned char*>(data), step);
    cv::cvtColor(colour, gray, cv::COLOR_BGR2GRAY);
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
