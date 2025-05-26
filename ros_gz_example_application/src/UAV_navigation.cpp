#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
// // Gazebo Transport includes
// #include <gz/msgs/twist.pb.h>
// #include <gz/msgs/vector3d.pb.h>
// #include <gz/transport/Node.hh>

// Reference signal (Position)
float ref_x = 3;
float ref_y = 4;



class UAV_Nav_Publisher : public rclcpp::Node{
    public:
        UAV_Nav_Publisher(): Node("UAV_PID_Control"){
            subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/x3/odometry",
                10,
                std::bind(&UAV_Nav_Publisher::PosCallback, this, std::placeholders::_1)
            );
            // publisher_ = gz_node_.Advertise<gz::msgs::Twist>("/model/x3/cmd_vel");
            RCLCPP_INFO(this->get_logger(), "UAV_PID_Control node started");
        }

    private:
        // void UAV_PID_Control(float x, float y){
        //     float error_x =  ref_x - x;
        //     float error_y =  ref_y - y;

        //     float k_p = 1.0;
        //     float vx = k_p*error_x;
        //     float vy = k_p*error_y;

        //     gz::msgs::Twist cmd_msg;
        //     gz::msgs::Vector3d *linear = cmd_msg.mutable_linear();
        //     linear->set_x(vx);
        //     linear->set_y(vy);
        //     linear->set_z(0.0);

        //     cmd_msg.mutable_angular()->set_x(0.0);
        //     cmd_msg.mutable_angular()->set_y(0.0);
        //     cmd_msg.mutable_angular()->set_z(0.0);

        //     // publisher_.publish(cmd_msg);
        // }


        void PosCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
            RCLCPP_INFO(this->get_logger(), "Received Pos callback triggered.");
            auto pos = msg->pose.pose.position;
            RCLCPP_INFO(this->get_logger(), "Position -> x: %0.3f, y: %0.3f", pos.x, pos.y);
            // UAV_PID_Control(pos.x, pos.y);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;

        // // Gazebo Transport
        // gz::transport::Node gz_node_;
        // gz::transport::Node::Publisher publisher_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UAV_Nav_Publisher>());
    rclcpp::shutdown();
    return 0;
}