#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class RobotNavController : public rclcpp::Node {
public:
    RobotNavController() : Node("robot_nav_controller"), initialized_(false) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fws_robot/odometry", 10,
            std::bind(&RobotNavController::odomCallback, this, std::placeholders::_1));

        /* --- 訂閱 IMU --- */
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10,
        std::bind(&RobotNavController::imuCallback, this, std::placeholders::_1));

        wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        waypoints_ = {
            {5.0f, 5.0f},
            {5.0f, 0.0f},
            {-5.0f, 0.0f},
            {0.0f, 0.0f}
        };

        RCLCPP_INFO(this->get_logger(), "Robot Navigation Controller started.");
    }

private:
    float ref_x = 5.0;
    float ref_y = 5.0;
    float tolerance = 0.3;
    float max_steering = M_PI / 4;
    float max_speed = 20.0;

    bool initialized_;
    bool imu_received_;
    float x_prev_{0.0f}, y_prev_{0.0f};
    double imu_yaw_{0.0};  // 最新 IMU yaw (rad)

    std::vector<std::pair<float, float>> waypoints_;
    size_t wp_index_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr    imu_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;

    float saturate(float value, float limit) {
        return std::max(std::min(value, limit), -limit);
    }

    void stopRobot() {
        std_msgs::msg::Float64MultiArray vel;
        std_msgs::msg::Float64MultiArray steer_cmd;
        vel.data = {0.0, 0.0, 0.0, 0.0};
        steer_cmd.data = {0.0, 0.0, 0.0, 0.0};
        steer_pub_->publish(steer_cmd);
        wheel_pub_->publish(vel);
        RCLCPP_INFO(this->get_logger(), "Goal reached. Robot stopped.");
    }

        /* --- IMU Callback --- */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        imu_yaw_ = yaw;
        imu_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (wp_index_ >= waypoints_.size()) {
            stopRobot();
            return;
        }

        float ref_x = waypoints_[wp_index_].first;
        float ref_y = waypoints_[wp_index_].second;

        auto pos = msg->pose.pose.position;
        float x = pos.x;
        float y = pos.y;

        float dx_goal = ref_x - x;
        float dy_goal = ref_y - y;
        float distance = std::hypot(dx_goal, dy_goal);

        RCLCPP_INFO(this->get_logger(),
                "Current (%.2f, %.2f) Dist=%.2f", x, y, distance);

        if (distance < tolerance) {
            RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached.", wp_index_);
            wp_index_++;
            return;
        }

    /* --- 取得機器人當前 heading --- */
        float heading;
        if (imu_received_)            // 有 IMU → 用 IMU yaw
        {
        heading = static_cast<float>(imu_yaw_);
        }
        else if (initialized_)        // 沒 IMU → 用上一點位移估算
        {
        float dx_move = x - x_prev_;
        float dy_move = y - y_prev_;
        if (std::fabs(dx_move) > 1e-3 || std::fabs(dy_move) > 1e-3)
            heading = std::atan2(dy_move, dx_move);
        else
            heading = 0.0f;
        }
        else
        {
        heading = 0.0f;
        }

        // Compute target direction
        float target_yaw = std::atan2(dy_goal, dx_goal);
        float yaw_error = target_yaw - heading;

        // Normalize yaw_error to [-π, π]
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        // Pivot turn threshold
        float pivot_threshold = M_PI / 4.0;  // 45 degrees
        if (std::fabs(yaw_error) > pivot_threshold) {
            float sign = (yaw_error > 0) ? 1.0 : -1.0;

            // Pivot turn: steering angle and wheel velocities
            std_msgs::msg::Float64MultiArray steer_cmd;
            std_msgs::msg::Float64MultiArray vel_cmd;

            if (sign > 0) {
                // Counter-clockwise
                vel_cmd.data = {-1.0, 1.0, -1.0, 1.0};
                steer_cmd.data = {-0.5, 0.5, 0.5, -0.5};
                RCLCPP_INFO(this->get_logger(), "Pivot Turn: Counter-Clockwise");
            } else {
                // Clockwise
                vel_cmd.data = {1.0, -1.0, 1.0, -1.0};
                steer_cmd.data = {-0.5, 0.5, 0.5, -0.5};
                RCLCPP_INFO(this->get_logger(), "Pivot Turn: Clockwise");
            }

            steer_pub_->publish(steer_cmd);
            wheel_pub_->publish(vel_cmd);
            return;
        }

        // Update previous position
        x_prev_ = x;
        y_prev_ = y;
        initialized_ = true;

        float steering = saturate(yaw_error, max_steering);
        std_msgs::msg::Float64MultiArray steer_cmd;
        steer_cmd.data = {steering, steering, steering, steering};
        steer_pub_->publish(steer_cmd);
        RCLCPP_INFO(this->get_logger(), "Steering angle: %.2f", steering);

        // Calculate speed with direction
        float direction = std::cos(yaw_error);  // < 0 means backwards
        float speed = saturate(distance * 5.0, max_speed) * direction;
        std_msgs::msg::Float64MultiArray vel_cmd;
        vel_cmd.data = {speed, speed, speed, speed};
        wheel_pub_->publish(vel_cmd);

        RCLCPP_INFO(this->get_logger(), "Moving %s with speed %.2f",
                    direction < 0 ? "backward" : "forward", speed);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNavController>());
    rclcpp::shutdown();
    return 0;
}
