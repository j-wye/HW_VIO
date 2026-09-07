// vio_node: ROS2 node around VioPipeline (MSCEqF filter + keyframe-gated front-end).
//
// Subscriptions only enqueue; one worker thread does all the work.
//
// IMU is drained as it arrives and used immediately for the fixed-rate output, but it is
// only handed to the filter and the gate once an image bounds it: when a frame stamped
// t_img is releasable (an IMU sample at or after t_img has been seen, which on an ordered
// topic proves no earlier IMU is still in flight), every buffered sample up to t_img goes
// in first, then the frame. So the filter always sees measurements in timestamp order
// regardless of arrival latency, and a replayed bag gives the same estimates as run_feeder.
// The filter updates when the gate fires (~7 Hz on the reference data); the output rate is
// independent of that because it comes from integrating the IMU past the last update.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>

#include "vio_node/pipeline.hpp"

namespace
{
double stampToSec(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1.0e-9 * static_cast<double>(t.nanosec);
}

builtin_interfaces::msg::Time toStamp(double t)
{
  builtin_interfaces::msg::Time s;
  double sec = std::floor(t);
  double nsec = std::round((t - sec) * 1.0e9);
  if (nsec >= 1.0e9) { sec += 1.0; nsec -= 1.0e9; }
  s.sec = static_cast<int32_t>(sec);
  s.nanosec = static_cast<uint32_t>(nsec);
  return s;
}
}  // namespace

class VioNode : public rclcpp::Node
{
 public:
  VioNode() : rclcpp::Node("vio_node")
  {
    const std::string config = declare_parameter<std::string>("config_filepath", "");
    const std::string imu_topic = declare_parameter<std::string>("imu_topic", "");
    const std::string cam_topic = declare_parameter<std::string>("cam_topic", "");
    const std::string odom_topic = declare_parameter<std::string>("odom_topic", "/vio/odom");
    const std::string pose_topic = declare_parameter<std::string>("pose_topic", "/vio/pose");
    const std::string path_topic = declare_parameter<std::string>("path_topic", "/vio/path");
    const std::string div_topic = declare_parameter<std::string>("divergence_topic", "/vio/divergence");
    // REP-105: a world-fixed frame that drifts is "odom". The estimate is the pose of the IMU
    // frame, so body_frame_id names the IMU, not base_link -- a consumer that wants base_link
    // applies the static base_link->imu transform from its own URDF.
    frame_id_ = declare_parameter<std::string>("frame_id", "odom");
    body_frame_id_ = declare_parameter<std::string>("body_frame_id", "imu");
    const std::string qos_profile = declare_parameter<std::string>("qos_profile", "reliable");

    const double out_hz = declare_parameter<double>("output_rate_hz", 10.0);
    imu_hold_max_ = declare_parameter<double>("imu_hold_max_s", 1.0);
    img_q_max_ = declare_parameter<int>("image_queue_max", 30);
    div_timeout_ = declare_parameter<double>("divergence_timeout_s", 2.0);
    div_pos_std_ = declare_parameter<double>("divergence_pos_std_m", 100.0);
    path_max_ = declare_parameter<int>("path_max_poses", 5000);   // 0 = 무제한
    const std::string out_csv = declare_parameter<std::string>("out_csv", "");

    // launch validates qos_profile through `choices`, but `ros2 run` does not, so check here too.
    if (qos_profile != "reliable" && qos_profile != "best_effort")
      throw std::runtime_error("qos_profile must be 'reliable' or 'best_effort', got '" + qos_profile + "'");
    const bool reliable = (qos_profile == "reliable");

    if (config.empty()) throw std::runtime_error("parameter config_filepath is required");
    if (imu_topic.empty()) throw std::runtime_error("parameter imu_topic is required");
    if (cam_topic.empty()) throw std::runtime_error("parameter cam_topic is required");
    if (out_hz <= 0.0) throw std::runtime_error("output_rate_hz must be > 0");
    period_ = 1.0 / out_hz;

    vio::GateParams gp;
    std::string err;
    const bool from_cfg = vio::VioPipeline::loadGateParams(config, gp, err);
    if (!err.empty()) throw std::runtime_error(err);
    pipe_ = std::make_unique<vio::VioPipeline>(config, gp);
    gravity_ = pipe_->sys().stateOptions().gravity_;
    const auto& res = pipe_->sys().options().track_manager_options_.tracker_options_.cam_options_.resolution_;
    cam_w_ = static_cast<int>(res(0));
    cam_h_ = static_cast<int>(res(1));
    RCLCPP_INFO(get_logger(), "config %s | gate %s: delta_px=%g fire_frac=%g min_inject=%d min_ref=%d max_dt=%g",
                config.c_str(), from_cfg ? "from config" : "defaults", gp.delta_px, gp.fire_frac, gp.min_inject,
                gp.min_ref, gp.max_dt);

    if (!out_csv.empty())
    {
      csv_.open(out_csv);
      csv_ << std::setprecision(17);
      vio::VioPipeline::csvHeader(csv_);
    }

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic, 10);
    pub_path_ = create_publisher<nav_msgs::msg::Path>(path_topic, 1);
    pub_div_ = create_publisher<std_msgs::msg::Bool>(div_topic, 10);
    path_.header.frame_id = frame_id_;

    // reliable for bag playback (best-effort drops fragmented image frames locally);
    // use best_effort for live sensor drivers that publish best-effort (a reliable
    // subscription does not match a best-effort publisher at all).
    const rclcpp::QoS cam_qos = reliable ? rclcpp::QoS(rclcpp::KeepLast(100)).reliable()
                                         : rclcpp::QoS(rclcpp::KeepLast(100)).best_effort();
    const rclcpp::QoS imu_qos = reliable ? rclcpp::QoS(rclcpp::KeepLast(2000)).reliable()
                                         : rclcpp::QoS(rclcpp::KeepLast(2000)).best_effort();
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, imu_qos, [this](sensor_msgs::msg::Imu::SharedPtr m) { onImu(*m); });
    sub_cam_ = create_subscription<sensor_msgs::msg::Image>(
        cam_topic, cam_qos, [this](sensor_msgs::msg::Image::SharedPtr m) { onImage(*m); });

    // The odometry path only runs on IMU arrival, so a silent IMU would also silence the
    // divergence flag. This timer reports it independently of the data streams.
    watchdog_ = create_wall_timer(std::chrono::duration<double>(div_timeout_ / 2.0), [this] { watchdog(); });

    worker_ = std::thread([this] { work(); });
  }

  ~VioNode() override
  {
    {
      std::lock_guard<std::mutex> lk(mu_);
      stop_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    if (csv_.is_open()) csv_.close();
    RCLCPP_INFO(get_logger(),
                "images=%zu imu=%zu updates=%zu odom=%zu late_images=%zu imu_flushes=%zu "
                "dropped_images=%zu rejected_updates=%zu",
                n_img_, n_imu_, n_update_, n_odom_, n_late_img_, n_imu_flushed_, n_dropped_img_.load(),
                n_rejected_);
  }

 private:
  struct ImuMsg { double t; msceqf::Imu imu; };
  struct ImgMsg { double t; cv::Mat gray; };

  void onImu(const sensor_msgs::msg::Imu& m)
  {
    ImuMsg q;
    q.t = stampToSec(m.header.stamp);
    q.imu.timestamp_ = q.t;
    q.imu.ang_ << m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z;
    q.imu.acc_ << m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z;
    {
      std::lock_guard<std::mutex> lk(mu_);
      imu_q_.push_back(std::move(q));
      newest_imu_t_ = std::max(newest_imu_t_, q.t);
    }
    cv_.notify_one();
  }

  void onImage(const sensor_msgs::msg::Image& m)
  {
    // The tracker builds its detection grid from the incoming size but masks with the configured
    // one, so a mismatched frame silently tracks the wrong region instead of failing.
    if (static_cast<int>(m.width) != cam_w_ || static_cast<int>(m.height) != cam_h_)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "image is %ux%u, config says %dx%d; dropping",
                            m.width, m.height, cam_w_, cam_h_);
      ++n_dropped_img_;
      return;
    }
    ImgMsg q;
    q.t = stampToSec(m.header.stamp);
    try
    {
      q.gray = vio::VioPipeline::toGray(m.height, m.width, m.encoding, m.data.data(), m.step);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "%s; dropping frame", e.what());
      ++n_dropped_img_;
      return;
    }
    {
      std::lock_guard<std::mutex> lk(mu_);
      img_q_.push_back(std::move(q));
      // Bounded so a worker that falls behind sheds frames instead of growing without limit.
      while (img_q_max_ > 0 && static_cast<int>(img_q_.size()) > img_q_max_)
      {
        img_q_.pop_front();
        ++n_dropped_img_;
      }
    }
    cv_.notify_one();
  }

  // An image is releasable once an IMU sample at or after its stamp has been seen: IMU
  // arrives in order on its own topic, so that proves no earlier IMU is still in flight.
  // (caller holds mu_)
  bool imageReady() const
  {
    if (img_q_.empty()) return false;
    return stop_ || newest_imu_t_ >= img_q_.front().t;
  }

  void work()
  {
    for (;;)
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [this] { return stop_ || !imu_q_.empty() || imageReady(); });
      // IMU first, always: it drives the fixed-rate output and must never wait on a frame.
      try
      {
        if (!imu_q_.empty())
        {
          ImuMsg m = imu_q_.front();
          imu_q_.pop_front();
          lk.unlock();
          handleImu(m);
          continue;
        }
        if (imageReady())
        {
          ImgMsg m = std::move(img_q_.front());
          img_q_.pop_front();
          lk.unlock();
          handleImage(m);
          continue;
        }
      }
      catch (const std::exception& e)
      {
        // without this the worker would die and the node would sit there publishing nothing
        RCLCPP_FATAL(get_logger(), "processing thread failed: %s", e.what());
        rclcpp::shutdown();
        return;
      }
      if (stop_) return;
    }
  }

  void handleImu(const ImuMsg& m)
  {
    ++n_imu_;
    last_imu_t_ = m.t;
    // Held until a frame bounds it (see handleImage). The trigger is camera silence, not the
    // span of held_: with frames already queued, flushing would feed IMU past them.
    held_.push_back(m.imu);
    replay_.push_back(m.imu);
    bool camera_silent;
    {
      std::lock_guard<std::mutex> lk(mu_);
      camera_silent = img_q_.empty();
    }
    if (camera_silent && last_img_t_ >= 0.0 && m.t - last_img_t_ > imu_hold_max_)
    {
      ++n_imu_flushed_;
      for (const auto& u : held_) pipe_->processImu(u);
      if (!held_.empty()) last_fed_imu_t_ = held_.back().timestamp_;
      held_.clear();
    }
    // keep enough history to re-integrate from an update time that lies in the past
    while (replay_.size() > 2 && m.t - replay_.front().timestamp_ > imu_hold_max_ + 1.0) replay_.pop_front();

    std::lock_guard<std::mutex> lk(pub_mu_);
    if (!have_prop_) return;
    integrateLocked(m.imu);

    if (next_tick_ < 0.0) next_tick_ = std::floor(m.t / period_) * period_ + period_;
    if (m.t + 1e-9 >= next_tick_)
    {
      publishOdom(m.t);
      next_tick_ += period_;
      if (m.t >= next_tick_) next_tick_ = std::floor(m.t / period_) * period_ + period_;
    }
  }

  // One Euler step of the output copy. Skipping a sample must not advance prop_t_, otherwise the
  // interval is dropped and a stale pose goes out stamped with the current time.
  // The caller holds pub_mu_ (this must not take it: onUpdate calls this while holding it).
  void integrateLocked(const msceqf::Imu& u)
  {
    const double dt = u.timestamp_ - prop_t_;
    if (dt <= 0.0) return;
    if (dt >= 0.5)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%.3f s gap in the IMU stream", dt);
    }
    const Eigen::Vector3d w = u.ang_ - bw_;
    const Eigen::Vector3d a = u.acc_ - ba_;
    const Eigen::Vector3d acc_w = prop_R_ * a + Eigen::Vector3d(0.0, 0.0, -gravity_);
    prop_p_ += prop_v_ * dt + 0.5 * acc_w * dt * dt;
    prop_v_ += acc_w * dt;
    const double th = w.norm() * dt;
    if (th > 1e-12) prop_R_ = prop_R_ * Eigen::AngleAxisd(th, w / w.norm()).toRotationMatrix();
    last_w_ = w;
    prop_t_ = u.timestamp_;
  }

  void handleImage(const ImgMsg& m)
  {
    ++n_img_;
    last_img_t_ = m.t;
    // every held sample up to this frame's stamp goes in first, in order
    while (!held_.empty() && held_.front().timestamp_ <= m.t)
    {
      pipe_->processImu(held_.front());
      last_fed_imu_t_ = held_.front().timestamp_;
      held_.pop_front();
    }
    // a frame older than IMU already given to the filter: an out-of-order camera topic, or a
    // frame that arrived after a camera-silence flush
    if (m.t < last_fed_imu_t_) ++n_late_img_;
    if (!pipe_->processImage(m.t, m.gray)) return;
    if (!pipe_->lastAccepted())
    {
      ++n_rejected_;  // the filter discarded it; publishing it as an update would hide the failure
      return;
    }
    onUpdate();
  }

  void onUpdate()
  {
    ++n_update_;
    std::lock_guard<std::mutex> lk(pub_mu_);
    const double t = pipe_->lastEmitTime();
    const auto est = pipe_->sys().stateEstimate();
    const auto core = pipe_->sys().coreCovariance();

    // reset the propagation copy to the fresh estimate
    prop_t_ = t;
    prop_R_ = est.T().R();
    prop_v_ = est.T().v();
    prop_p_ = est.T().p();
    bw_ = est.b().head<3>();
    ba_ = est.b().tail<3>();
    have_prop_ = true;
    last_update_t_ = t;

    // The samples between the update time and the newest IMU were already consumed by the old
    // copy; replay them so the next arrival integrates one sample step, not |timeshift| worth.
    for (const auto& u : replay_)
      if (u.timestamp_ > t) integrateLocked(u);

    // The core covariance is expressed in the filter's own error coordinates (blocks: attitude
    // 0-2, velocity 3-5, position 6-8, left error on D relative to the origin frame). ROS wants
    // Cov(p - p_hat) and fixed-axis Cov(dtheta) in frame_id, and the twist covariance in the
    // child frame, so rotate both instead of copying the raw blocks.
    const Eigen::Matrix3d R0 = pipe_->sys().stateOrigin().T().R();
    const Eigen::Vector3d p_hat = est.T().p();
    Eigen::Matrix3d px;  // [p_hat]_x
    px << 0.0, -p_hat.z(), p_hat.y(), p_hat.z(), 0.0, -p_hat.x(), -p_hat.y(), p_hat.x(), 0.0;
    Eigen::Matrix<double, 6, 9> J = Eigen::Matrix<double, 6, 9>::Zero();
    J.block<3, 3>(0, 0) = -px * R0;   // position   <- attitude error
    J.block<3, 3>(0, 6) = R0;         // position   <- position error
    J.block<3, 3>(3, 0) = R0;         // orientation<- attitude error
    cov_pose_ = J * core.block<9, 9>(0, 0) * J.transpose();
    const Eigen::Matrix3d R_D = R0.transpose() * prop_R_;  // origin -> body
    cov_vel_ = R_D.transpose() * core.block<3, 3>(3, 3) * R_D;
    pos_std_ = std::sqrt(cov_pose_.block<3, 3>(0, 0).trace() / 3.0);

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = toStamp(t);
    pose.header.frame_id = frame_id_;
    fillPose(pose.pose.pose, est.T().q(), est.T().p());
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) pose.pose.covariance[6 * r + c] = cov_pose_(r, c);
    pub_pose_->publish(pose);

    geometry_msgs::msg::PoseStamped ps;
    ps.header = pose.header;
    ps.pose = pose.pose.pose;
    path_.header.stamp = pose.header.stamp;
    path_.poses.push_back(ps);
    if (path_max_ > 0 && static_cast<int>(path_.poses.size()) > path_max_)
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - path_max_));
    pub_path_->publish(path_);

    if (csv_.is_open()) pipe_->csvRow(csv_);
  }

  void publishOdom(double t)
  {
    ++n_odom_;
    nav_msgs::msg::Odometry od;
    od.header.stamp = toStamp(t);
    od.header.frame_id = frame_id_;
    od.child_frame_id = body_frame_id_;
    fillPose(od.pose.pose, Eigen::Quaterniond(prop_R_), prop_p_);
    const Eigen::Vector3d v_body = prop_R_.transpose() * prop_v_;
    od.twist.twist.linear.x = v_body.x();
    od.twist.twist.linear.y = v_body.y();
    od.twist.twist.linear.z = v_body.z();
    od.twist.twist.angular.x = last_w_.x();
    od.twist.twist.angular.y = last_w_.y();
    od.twist.twist.angular.z = last_w_.z();
    for (int r = 0; r < 6; ++r)
      for (int c = 0; c < 6; ++c) od.pose.covariance[6 * r + c] = cov_pose_(r, c);
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) od.twist.covariance[6 * r + c] = cov_vel_(r, c);
    pub_odom_->publish(od);

    std_msgs::msg::Bool div;
    div.data = diverged(t);
    pub_div_->publish(div);
  }

  // Runs on the ROS executor thread: reports divergence even when no IMU is arriving.
  void watchdog()
  {
    std_msgs::msg::Bool div;
    {
      std::lock_guard<std::mutex> lk(pub_mu_);
      if (!have_prop_) return;
      div.data = diverged(last_imu_t_);
    }
    pub_div_->publish(div);
  }

  // caller holds pub_mu_ (or is the worker before publishing)
  bool diverged(double t) const
  {
    return (t - last_update_t_) > div_timeout_ || pos_std_ > div_pos_std_ || !std::isfinite(prop_p_.x()) ||
           !std::isfinite(prop_p_.y()) || !std::isfinite(prop_p_.z());
  }

  static void fillPose(geometry_msgs::msg::Pose& out, const Eigen::Quaterniond& q, const Eigen::Vector3d& p)
  {
    out.orientation.x = q.x();
    out.orientation.y = q.y();
    out.orientation.z = q.z();
    out.orientation.w = q.w();
    out.position.x = p.x();
    out.position.y = p.y();
    out.position.z = p.z();
  }

  std::unique_ptr<vio::VioPipeline> pipe_;
  std::string frame_id_, body_frame_id_;
  int img_q_max_ = 30, cam_w_ = 0, cam_h_ = 0;
  double imu_hold_max_ = 1.0, period_ = 0.1, div_timeout_ = 2.0, div_pos_std_ = 100.0, gravity_ = 9.81;
  int path_max_ = 5000;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_div_;
  nav_msgs::msg::Path path_;
  std::ofstream csv_;

  std::mutex mu_;
  std::condition_variable cv_;
  std::deque<ImuMsg> imu_q_;
  std::deque<ImgMsg> img_q_;
  double newest_imu_t_ = -1.0;
  bool stop_ = false;
  std::thread worker_;

  // propagation copy of the last estimate
  bool have_prop_ = false;
  rclcpp::TimerBase::SharedPtr watchdog_;
  std::mutex pub_mu_;              // guards the state the watchdog reads
  std::deque<msceqf::Imu> held_;   // IMU seen but not yet handed to the filter/gate
  std::deque<msceqf::Imu> replay_; // recent IMU, replayed onto the output copy after an update
  double prop_t_ = -1.0, next_tick_ = -1.0, last_update_t_ = -1.0, pos_std_ = 0.0;
  double last_imu_t_ = -1.0, last_fed_imu_t_ = -1.0, last_img_t_ = -1.0;
  Eigen::Matrix3d prop_R_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d prop_v_ = Eigen::Vector3d::Zero(), prop_p_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d bw_ = Eigen::Vector3d::Zero(), ba_ = Eigen::Vector3d::Zero(), last_w_ = Eigen::Vector3d::Zero();
  Eigen::Matrix<double, 6, 6> cov_pose_ = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix3d cov_vel_ = Eigen::Matrix3d::Zero();

  std::size_t n_img_ = 0, n_imu_ = 0, n_update_ = 0, n_odom_ = 0, n_late_img_ = 0, n_imu_flushed_ = 0;
  std::atomic<std::size_t> n_dropped_img_{0};
  std::size_t n_rejected_ = 0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  int rc = 0;
  try
  {
    auto node = std::make_shared<VioNode>();
    rclcpp::spin(node);
  }
  catch (const std::exception& e)
  {
    std::cerr << "[vio_node] " << e.what() << "\n";
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
