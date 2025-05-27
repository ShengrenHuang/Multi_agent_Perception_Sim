#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"


// Reference signal (Position)
float ref_x = 30;
float ref_y = 30;
float max_amp = 1;


class UAV_Nav_Publisher : public rclcpp::Node{
    public:
        UAV_Nav_Publisher(): Node("UAV_PID_Control"){
            subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/x3/odometry",
                10,
                std::bind(&UAV_Nav_Publisher::PosCallback, this, std::placeholders::_1)
            );
            publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/x3/cmd_vel", 10);
            RCLCPP_INFO(this->get_logger(), "UAV_PID_Control node started");
        }

    private:
        float sat_func(float amp){
            float result = 0;
            if(abs(amp)>=max_amp){
                result = (abs(amp)/amp)*max_amp;
            }else{
                result = amp;
            }
            return result;
        }
        void UAV_PID_Control(float x, float y){
            float error_x =  ref_x - x;
            float error_y =  ref_y - y;

            RCLCPP_INFO(this->get_logger(), "Error -> x: %0.3f, y: %0.3f", error_x, error_y);
            float k_p = 1;
            float vx = k_p*error_x;
            float vy = k_p*error_y;
            vx = sat_func(vx);
            vy = sat_func(vy);

            geometry_msgs::msg::Twist msg;
            msg.linear.x = vx;
            msg.linear.y = vy;
            msg.linear.z = 0.0;
            msg.angular.z = 0.0;
            publisher_->publish(msg);
        }

        void PosCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received Pos callback triggered.");
            auto pos = msg->pose.pose.position;
            // RCLCPP_INFO(this->get_logger(), "Position -> x: %0.3f, y: %0.3f", pos.x, pos.y);
            UAV_PID_Control(pos.x, pos.y);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UAV_Nav_Publisher>());
    rclcpp::shutdown();
    return 0;
}