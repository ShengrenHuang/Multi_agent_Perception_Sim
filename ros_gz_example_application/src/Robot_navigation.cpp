#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RobotNavController : public rclcpp::Node {
public:
    RobotNavController() : Node("robot_nav_controller"), initialized_(false) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/fws_robot/odometry", 10,
            std::bind(&RobotNavController::odomCallback, this, std::placeholders::_1));

        wheel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_velocity_controller/commands", 10);

        steer_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/forward_position_controller/commands", 10);

        RCLCPP_INFO(this->get_logger(), "Robot Navigation Controller started (no yaw)");
    }

private:
    float ref_x = 15.0;
    float ref_y = 10.0;
    float tolerance = 0.3;
    float max_steering = M_PI / 4;
    float max_speed = 20.0;

    bool initialized_;
    float x_prev_, y_prev_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;

    float saturate(float value, float limit) {
        return std::max(std::min(value, limit), -limit);
    }

    void stopRobot() {
        std_msgs::msg::Float64MultiArray vel;
        vel.data = {0.0, 0.0, 0.0, 0.0};
        wheel_pub_->publish(vel);
        RCLCPP_INFO(this->get_logger(), "Goal reached. Robot stopped.");
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto pos = msg->pose.pose.position;
        float x = pos.x;
        float y = pos.y;

        float dx_goal = ref_x - x;
        float dy_goal = ref_y - y;
        float distance = std::sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

        RCLCPP_INFO(this->get_logger(), "Current Pos: (%.2f, %.2f), Distance to Goal: %.2f", x, y, distance);

        if (distance < tolerance) {
            stopRobot();
            return;
        }

        // Estimate heading from previous movement
        float heading = 0.0;
        if (initialized_) {
            float dx_move = x - x_prev_;
            float dy_move = y - y_prev_;
            if (std::abs(dx_move) > 1e-3 || std::abs(dy_move) > 1e-3)
                heading = std::atan2(dy_move, dx_move);
        }

        // Compute target direction
        float target_yaw = std::atan2(dy_goal, dx_goal);
        float yaw_error = target_yaw - heading;

        // Normalize yaw_error to [-π, π]
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

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
