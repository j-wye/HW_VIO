// deterministic offline msceqf runner: reads the bag directly, calls processMeasurement
// single-threaded in timestamp order. the ros2 wrapper's result depends on playback rate.

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "msceqf/msceqf.hpp"
#include "vio_node/pipeline.hpp"

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
    std::cerr << "usage: run_offline <bag_dir> <config.yaml> <out.csv> "
                 "[--start S] [--duration D]\n";
    return 1;
  }
  const std::string bag_dir = argv[1];
  const std::string config_path = argv[2];
  const std::string out_path = argv[3];
  double start_s = 0.0;
  double duration_s = -1.0;
  int i = 4;
  for (; i + 1 < argc; i += 2)
  {
    const std::string k = argv[i], v = argv[i + 1];
    try
    {
      if (k == "--start") start_s = std::stod(v);
      else if (k == "--duration") duration_s = std::stod(v);
      else
      {
        std::cerr << "[offline] unknown option " << k << "\n";
        return 1;
      }
    }
    catch (const std::exception&)
    {
      std::cerr << "[offline] " << k << " needs a number, got '" << v << "'\n";
      return 1;
    }
  }
  if (i < argc)
  {
    std::cerr << "[offline] option " << argv[i] << " needs a value\n";
    return 1;
  }

  msceqf::MSCEqF sys(config_path);

  rclcpp::Serialization<sensor_msgs::msg::Image> img_ser;
  rclcpp::Serialization<sensor_msgs::msg::Imu> imu_ser;

  rosbag2_cpp::Reader reader;
  reader.open(bag_dir);

  std::ofstream out(out_path);
  out << std::setprecision(17);
  out << "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,bwx,bwy,bwz,bax,bay,baz,"
         "P00,P11,P22,P33,P44,P55,P66,P77,P88\n";

  // assumes a timestamp-sorted store; one forward pass then keeps sensor order.
  int64_t t0_ns = -1;
  std::size_t n_img = 0, n_imu = 0, n_pose = 0;
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
    }
    else if (msg->topic_name == kImageTopic)
    {
      sensor_msgs::msg::Image m;
      img_ser.deserialize_message(&ser, &m);
      msceqf::Camera cam;
      cam.timestamp_ = stampToSec(m.header.stamp);
      // same conversion as run_feeder and the node, or the paths would not be comparable
      cam.image_ = vio::VioPipeline::toGray(m.height, m.width, m.encoding, m.data.data(), m.step);
      sys.processMeasurement(cam);
      ++n_img;

      if (sys.isInit())
      {
        const auto est = sys.stateEstimate();
        const auto cov = sys.covariance();
        const auto& q = est.T().q();
        const auto& p = est.T().p();
        const auto& v = est.T().v();
        const auto& b = est.b();
        out << cam.timestamp_ << ',' << p.x() << ',' << p.y() << ',' << p.z() << ','
            << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << ','
            << v.x() << ',' << v.y() << ',' << v.z();
        for (int i = 0; i < 6; ++i) out << ',' << b(i);
        for (int i = 0; i < 9; ++i) out << ',' << cov(i, i);
        out << '\n';
        ++n_pose;
      }
    }
  }
  out.close();
  std::cerr << "[offline] images=" << n_img << " imu=" << n_imu
            << " poses=" << n_pose << " -> " << out_path << std::endl;
  return 0;
}
