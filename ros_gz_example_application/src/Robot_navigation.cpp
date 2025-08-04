#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <map>
#include <vector>
#include <utility>  // already included but OK to keep
#include <string>   // for to_string
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include <fstream>

class RobotNavController : public rclcpp::Node {
    using State = std::pair<int, int>;
    using Neighbors = std::vector<State>;

    std::map<State, Neighbors> grid_map_;
    int rows_ = 10;
    int cols_ = 10;
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

        wp_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/robot/waypoint_index", 10);


        grid_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/robot/adjacent_cells", 10);

        // Define waypoints
        std::string waypoint_file = "/home/cirl/ros_gz_project_template-main/ros_gz_example_application/src/robot.txt";
        if (!loadWaypointsFromFile(waypoint_file)) {
            rclcpp::shutdown();  // Exit if file loading fails
            return;
        }

        // waypoints_ = {
        //     {1.0f, 0.0f},
        //     {1.0f, 1.0f},
        //     {2.0f, 1.0f},
        //     {2.0f, 2.0f},
        //     {2.0f, 3.0f},
        //     {2.0f, 4.0f},
        //     {2.0f, 5.0f},
        // };


        std::vector<State> directions = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};

        for (int x = 0; x < rows_; ++x) {
            for (int y = 0; y < cols_; ++y) {
                State current = {x, y};
                Neighbors neighbors;

                for (const auto& d : directions) {
                    int nx = x + d.first;
                    int ny = y + d.second;

                    if (nx >= 0 && nx < rows_ && ny >= 0 && ny < cols_) {
                        neighbors.emplace_back(nx, ny);
                    }
                }

                grid_map_[current] = neighbors;
            }
        }


        RCLCPP_INFO(this->get_logger(), "Robot Navigation Controller started.");
    }

private:
    // float ref_x = 5.0;
    // float ref_y = 5.0;
    float tolerance = 0.3;
    float max_steering = M_PI / 4;
    float max_speed = 100.0;  //50

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
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr grid_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wp_index_pub_;

    bool loadWaypointsFromFile(const std::string& filename) {
        std::ifstream infile(filename);
        if (!infile.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", filename.c_str());
            return false;
        }

        float x, y;
        float scale_factor = 1.0;
        waypoints_.clear();
        while (infile >> x >> y) {
            waypoints_.emplace_back(scale_factor*x, scale_factor*y);
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), filename.c_str());
        return true;
    }

    std::string formatNeighbors(const Neighbors& neighbors) {
        std::string result = "[";
        for (size_t i = 0; i < neighbors.size(); ++i) {
            result += "(" + std::to_string(neighbors[i].first) + "," + std::to_string(neighbors[i].second) + ")";
            if (i != neighbors.size() - 1) result += ", ";
        }
        result += "]";
        return result;
    }

    std::pair<int, int> continuousToGridCell(float x, float y, int grid_width, int grid_height) {
        int grid_x = static_cast<int>(std::floor(x));
        int grid_y = static_cast<int>(std::floor(y));

        // Clamp to grid bounds
        grid_x = std::max(0, std::min(grid_x, grid_width - 1));
        grid_y = std::max(0, std::min(grid_y, grid_height - 1));

        return {grid_x, grid_y};
    }


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

        auto grid_pos = continuousToGridCell(x, y, rows_, cols_);
        const auto& neighbors = grid_map_[grid_pos];

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header.stamp = this->get_clock()->now();
        pose_array.header.frame_id = "map";  // Or your world frame

        for (const auto& n : neighbors) {
            geometry_msgs::msg::Pose pose;
            pose.position.x = static_cast<double>(n.first);
            pose.position.y = static_cast<double>(n.second);
            pose.position.z = 0.0;  // flat 2D grid
            pose.orientation.w = 1.0;  // neutral rotation
            pose_array.poses.push_back(pose);
        }

        grid_pub_->publish(pose_array);


        float dx_goal = ref_x - x;
        float dy_goal = ref_y - y;
        float distance = std::hypot(dx_goal, dy_goal);

        // RCLCPP_INFO(this->get_logger(),
        //         "Current (%.2f, %.2f) Dist=%.2f", x, y, distance);
        std_msgs::msg::Int32 index_msg;
        index_msg.data = static_cast<int>(wp_index_);
        wp_index_pub_->publish(index_msg);
        if (distance < tolerance) {
            // RCLCPP_INFO(this->get_logger(), "Waypoint %zu reached.", wp_index_);
            wp_index_++;
            // Publish updated index
            std_msgs::msg::Int32 index_msg;
            index_msg.data = static_cast<int>(wp_index_);
            wp_index_pub_->publish(index_msg);
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
                vel_cmd.data = {-5.0, 5.0, -5.0, 5.0};    // {-1.0, 1.0, -1.0, 1.0};
                steer_cmd.data = {-0.5, 0.5, 0.5, -0.5};
                RCLCPP_INFO(this->get_logger(), "Pivot Turn: Counter-Clockwise");
            } else {
                // Clockwise
                vel_cmd.data = {5.0, -5.0, 5.0, -5.0};    // {1.0, -1.0, 1.0, -1.0};
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
        // RCLCPP_INFO(this->get_logger(), "Steering angle: %.2f", steering);

        // Calculate speed with direction
        float direction = std::cos(yaw_error);  // < 0 means backwards
        float speed = saturate(distance * 50.0, max_speed) * direction;   //20
        std_msgs::msg::Float64MultiArray vel_cmd;
        vel_cmd.data = {speed, speed, speed, speed};
        wheel_pub_->publish(vel_cmd);

        // RCLCPP_INFO(this->get_logger(), "Moving %s with speed %.2f",
        //             direction < 0 ? "backward" : "forward", speed);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotNavController>());
    rclcpp::shutdown();
    return 0;
}
