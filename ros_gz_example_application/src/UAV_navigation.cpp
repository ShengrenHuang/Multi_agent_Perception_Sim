#include <iostream>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <map>
#include <vector>
#include <utility>  // already included but OK to keep
#include <string>   // for to_string
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/int32.hpp"
#include <fstream>


class UAV_Nav_Publisher : public rclcpp::Node {
    using State = std::pair<int, int>;
    using Neighbors = std::vector<State>;

    std::map<State, Neighbors> grid_map_;
    int rows_ = 10;
    int cols_ = 10;
public:
    UAV_Nav_Publisher() : Node("UAV_PID_Control"), wp_index_(0) {
        subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/x3/odometry", 10,
            std::bind(&UAV_Nav_Publisher::PosCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/x3/cmd_vel", 10);
        wp_index_pub_ = this->create_publisher<std_msgs::msg::Int32>("/uav/waypoint_index", 10);
        robot_index_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/robot/waypoint_index", 10,
            std::bind(&UAV_Nav_Publisher::RobotIndexCallback, this, std::placeholders::_1));
        grid_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/uav/adjacent_cells", 10);
        // Define waypoints
        std::string waypoint_file = "/home/cirl/ros_gz_project_template-main/ros_gz_example_application/src/uav.txt";
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

        RCLCPP_INFO(this->get_logger(), "UAV waypoint navigation started");
    }

private:
    std::vector<std::pair<float, float>> waypoints_;
    size_t wp_index_;
    float tolerance_ = 0.5;
    float max_amp_ = 1.0;  //1.0
    float k_p_ = 1.0;   //1.0
    int robot_index_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr grid_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wp_index_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr robot_index_sub_;


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

    void RobotIndexCallback(const std_msgs::msg::Int32::SharedPtr msg) {
        robot_index_ = msg->data;
    }

    std::pair<int, int> continuousToGridCell(float x, float y, int grid_width, int grid_height) {
        int grid_x = static_cast<int>(std::floor(x));
        int grid_y = static_cast<int>(std::floor(y));

        // Clamp to grid bounds
        grid_x = std::max(0, std::min(grid_x, grid_width - 1));
        grid_y = std::max(0, std::min(grid_y, grid_height - 1));

        return {grid_x, grid_y};
    }

    float saturate(float amp) {
        return std::max(std::min(amp, max_amp_), -max_amp_);
    }

    void stopUAV() {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "All waypoints reached. UAV stopped.");
    }

    void UAV_PID_Control(float x, float y) {
        if (wp_index_ >= waypoints_.size()) {
            stopUAV();
            return;
        }

        if (static_cast<int>(wp_index_) > robot_index_) {
            stopUAV();
            RCLCPP_INFO(this->get_logger(), "UAV paused: index (%d) > robot_index (%d) + 1", wp_index_, robot_index_);
            return;
        }

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




        float ref_x = waypoints_[wp_index_].first;
        float ref_y = waypoints_[wp_index_].second;

        float error_x = ref_x - x;
        float error_y = ref_y - y;
        float distance = std::hypot(error_x, error_y);

        // RCLCPP_INFO(this->get_logger(), "Waypoint %zu -> Ref(%.2f, %.2f), Pos(%.2f, %.2f), Dist=%.2f",
        //             wp_index_, ref_x, ref_y, x, y, distance);
        std_msgs::msg::Int32 index_msg;
        index_msg.data = static_cast<int>(wp_index_);
        wp_index_pub_->publish(index_msg);
        if (distance < tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu", wp_index_);
            wp_index_++;
            // Publish updated index
            std_msgs::msg::Int32 index_msg;
            index_msg.data = static_cast<int>(wp_index_);
            wp_index_pub_->publish(index_msg);
            return;
        }


        float vx = saturate(k_p_ * error_x);
        float vy = saturate(k_p_ * error_y);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.linear.z = 0.0;
        msg.angular.z = 0.0;
        publisher_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Command -> vx: %.2f, vy: %.2f", vx, vy);
    }

    void PosCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto pos = msg->pose.pose.position;
        UAV_PID_Control(pos.x, pos.y);
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UAV_Nav_Publisher>());
    rclcpp::shutdown();
    return 0;
}
