#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros_gz_example_application/odometry.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>


using std::placeholders::_1;
namespace fws = four_wheel_steering_controller;

class OdometryPublisher : public rclcpp::Node
{
public:
  OdometryPublisher()
  : Node("odometry_publisher_node"), odom_(10)
  {
    // Set wheel parameters
    double wheel_radius = 0.05;
    double wheel_base   = 0.3;
    double steering_track = 0.2;
    double y_offset = 0.0;

    odom_.setWheelParams(steering_track, y_offset, wheel_radius, wheel_base);

    // Init time
    odom_.init(this->now());

    // Publisher
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_pub_   = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    /* -------- Subscribers -------- */
    vel_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_velocity_controller/commands", 10,
      std::bind(&OdometryPublisher::velCb, this, _1));

    steer_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/forward_position_controller/commands", 10,
      std::bind(&OdometryPublisher::steerCb, this, _1));
    
    // Timer to simulate update (50Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&OdometryPublisher::timerCallback, this)
    );
  }

private:
  fws::Odometry odom_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr steer_sub_;

  /* ===== 狀態變數 ===== */
  double fl_speed_ {}, fr_speed_ {}, rl_speed_ {}, rr_speed_ {};
  double fl_steer_ {}, fr_steer_ {}, rl_steer_ {}, rr_steer_ {};
  bool   got_vel_  {false};
  bool   got_steer_{false};

  /* ---------------- 回呼 ---------------- */
  void velCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;
    fl_speed_ = msg->data[0];
    fr_speed_ = msg->data[1];
    rl_speed_ = msg->data[2];
    rr_speed_ = msg->data[3];
    got_vel_  = true;
  }

  void steerCb(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 4) return;
    fl_steer_ = msg->data[0];
    fr_steer_ = msg->data[1];
    rl_steer_ = msg->data[2];
    rr_steer_ = msg->data[3];
    got_steer_ = true;
  }


  void timerCallback()
  {
    // TODO: Replace with real sensor/joint state data
    // double fl_speed = -1.0;
    // double fr_speed = -1.0;
    // double rl_speed = -1.0;
    // double rr_speed = -1.0;
    // double front_steer = 0.0;
    // double rear_steer  = 0.0;
    const double front_steer = 0.5 * (fl_steer_ + fr_steer_);
    const double rear_steer  = 0.5 * (rl_steer_ + rr_steer_);

    rclcpp::Time now = this->now();
    bool updated = odom_.update(fl_speed_, fr_speed_, rl_speed_, rr_speed_, front_steer, rear_steer, now);
    if (!updated) return;

    nav_msgs::msg::Odometry msg;
    msg.header.stamp = now;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "base_link";

    msg.pose.pose.position.x = odom_.getX();
    msg.pose.pose.position.y = odom_.getY();
    msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odom_.getHeading());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    msg.twist.twist.linear.x  = odom_.getLinearX();
    msg.twist.twist.linear.y  = odom_.getLinearY();
    msg.twist.twist.angular.z = odom_.getAngular();

    odom_pub_->publish(msg);

    // TF
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = msg.header;
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = msg.pose.pose.position.x;
    tf_msg.transform.translation.y = msg.pose.pose.position.y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = msg.pose.pose.orientation;
    tf_pub_->sendTransform(tf_msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryPublisher>());
  rclcpp::shutdown();
  return 0;
}
