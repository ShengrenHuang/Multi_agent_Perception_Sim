#include "ros_gz_example_application/odometry.hpp"
#include <cmath>          // std::sin, std::cos, std::atan2, std::hypot, std::copysign
#include <tf2/utils.h>    // 若日後要操作 quaternion，可保留
#include "rclcpp/rclcpp.hpp"

namespace four_wheel_steering_controller
{
namespace bacc = boost::accumulators;

/* ===============================
 *  建構子與初始化
 * ===============================*/
Odometry::Odometry(size_t win)
: last_update_timestamp_(0),
  x_(0.0), y_(0.0), heading_(0.0),
  linear_(0.0), linear_x_(0.0), linear_y_(0.0), angular_(0.0),
  steering_track_(0.0), wheel_steering_y_offset_(0.0),
  wheel_radius_(0.0), wheel_base_(0.0),
  wheel_old_pos_(0.0),
  velocity_rolling_window_size_(win),
  linear_accel_acc_(RollingWindow::window_size = win),
  linear_jerk_acc_ (RollingWindow::window_size = win),
  front_steer_vel_acc_(RollingWindow::window_size = win),
  rear_steer_vel_acc_ (RollingWindow::window_size = win),
  linear_vel_prev_(0.0), linear_accel_prev_(0.0),
  front_steer_vel_prev_(0.0), rear_steer_vel_prev_(0.0)
{}

void Odometry::init(const rclcpp::Time & now)
{
  resetAccumulators();
  last_update_timestamp_ = now;
  // 重置前一次狀態
  linear_vel_prev_       = 0.0;
  linear_accel_prev_     = 0.0;
  front_steer_vel_prev_  = 0.0;
  rear_steer_vel_prev_   = 0.0;
}

/* ===============================
 *  主要更新函式
 * ===============================*/
bool Odometry::update(const double& fl_speed, const double& fr_speed,
                      const double& rl_speed, const double& rr_speed,
                      const double& fl_steer, const double& fr_steer,
                      const double& rl_steer, const double& rr_steer,
                      const rclcpp::Time& now)
{

  const double front_steer = 0.5 * (fl_steer + fr_steer);
  const double rear_steer  = 0.5 * (rl_steer + rr_steer);

  /* ----- 幾何計算 (四輪轉向) ------------------------- */
  const double front_tmp = std::cos(front_steer) * (std::tan(front_steer) - std::tan(rear_steer)) / wheel_base_;
  const double rear_tmp  = std::cos(rear_steer ) * (std::tan(front_steer) - std::tan(rear_steer)) / wheel_base_;

  // 簡化：輪內外同速假設，取左右輪平均
  const double front_lin = wheel_radius_ * 0.5 * (fl_speed + fr_speed);
  const double rear_lin  = wheel_radius_ * 0.5 * (rl_speed + rr_speed);



  /* ----- 判斷是否為 pivot turn（前後輪近似 ±90°） --- */
  bool pivot_turn = false;

  const double steer_threshold = 0.1;
  const bool opp_steer_front = std::fabs(fl_steer + fr_steer) < steer_threshold;
  const bool opp_steer_rear  = std::fabs(rl_steer + rr_steer) < steer_threshold;

  if (opp_steer_front && opp_steer_rear)
  {
    pivot_turn = true;
    // RCLCPP_INFO(rclcpp::get_logger("odom"), "Pivot turn detected!");
    // 以前左輪速度估算原地角速度 (任選一輪皆可)
    const double R = std::sqrt(std::pow(wheel_base_/2.0, 2) + std::pow(steering_track_/2.0, 2)) + wheel_steering_y_offset_;
    angular_   = - wheel_radius_ * fl_speed / R;  // rad/s
    linear_x_  = 1e-5; // 為了避免 0 讓部份演算法失效，給極小值
    linear_y_  = 1e-5;
    linear_    = 1e-5;
  }
  else
  {
    angular_  = 0.5 * (front_lin * front_tmp + rear_lin * rear_tmp);
    linear_x_ = 0.5 * (front_lin * std::cos(front_steer) + rear_lin * std::cos(rear_steer));
    linear_y_ = 0.5 * (front_lin * std::sin(front_steer) + rear_lin * std::sin(rear_steer));
    linear_   = std::copysign(std::hypot(linear_x_, linear_y_), rear_lin);
  }

  /* ----- 時間差與數值積分 --------------------------- */
  const double dt = (now - last_update_timestamp_).seconds();
  if (dt < 1e-4)  // 間隔太小視為無效
    return false;

  last_update_timestamp_ = now;
  integrateXY(linear_x_ * dt, linear_y_ * dt, angular_ * dt, pivot_turn);

  /* ----- Rolling Mean 估算加速度、jerk 與轉向角速度 - */
  const double linear_accel_curr = (linear_ - linear_vel_prev_) / dt;           // a = Δv/Δt
  linear_accel_acc_(linear_accel_curr);

  const double linear_jerk_curr  = (linear_accel_curr - linear_accel_prev_) / dt; // j = Δa/Δt
  linear_jerk_acc_(linear_jerk_curr);

  const double front_steer_vel_curr = (front_steer - front_steer_vel_prev_) / dt;
  front_steer_vel_acc_(front_steer_vel_curr);

  const double rear_steer_vel_curr  = (rear_steer  - rear_steer_vel_prev_)  / dt;
  rear_steer_vel_acc_(rear_steer_vel_curr);

  // 更新前一次記錄
  linear_vel_prev_      = linear_;
  linear_accel_prev_    = bacc::rolling_mean(linear_accel_acc_);
  front_steer_vel_prev_ = front_steer;
  rear_steer_vel_prev_  = rear_steer;

  return true;
}

/* ===============================
 *  積分函式
 * ===============================*/
void Odometry::integrateXY(double dx_body, double dy_body, double dtheta, bool pivot_turn)
{
  double delta_x = 0.0;
  double delta_y = 0.0;

  if (!pivot_turn) {
    delta_x =  dx_body * std::cos(heading_) - dy_body * std::sin(heading_);
    delta_y =  dx_body * std::sin(heading_) + dy_body * std::cos(heading_);
  }
  // 若 pivot_turn, delta_x / delta_y 維持 0

  x_       += delta_x;
  y_       += delta_y;
  heading_ += dtheta;

  // 將 heading 限制在 [-π, π]
  heading_ = std::atan2(std::sin(heading_), std::cos(heading_));
}

/* ===============================
 *  其他介面
 * ===============================*/
void Odometry::setWheelParams(double track, double y_off, double radius, double base)
{
  steering_track_            = track;
  wheel_steering_y_offset_   = y_off;
  wheel_radius_              = radius;
  wheel_base_                = base;
}

void Odometry::setVelocityRollingWindowSize(size_t win)
{
  velocity_rolling_window_size_ = win;
  resetAccumulators();
}

void Odometry::resetAccumulators()
{
  linear_accel_acc_     = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  linear_jerk_acc_      = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  front_steer_vel_acc_  = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
  rear_steer_vel_acc_   = RollingMeanAcc(RollingWindow::window_size = velocity_rolling_window_size_);
}

} // namespace four_wheel_steering_controller