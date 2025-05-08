#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace cv;

CascadeClassifier Robot_cascade;





class ImagePublisher : public rclcpp::Node{
    public:
        ImagePublisher(): Node("Detection_publisher"){
            subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                10,
                std::bind(&ImagePublisher::imageCallback, this, std::placeholders::_1)
            );
            publisher_ = image_transport::create_publisher(this, "/detection_result");
            RCLCPP_INFO(this->get_logger(), "Image processor node started");
        }

    private:
        void RobotDetection(const cv_bridge::CvImagePtr& cv_ptr, const std_msgs::msg::Header& header){
            Mat frame = cv_ptr->image;
            Mat frame_gray;
            cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
        
            std::vector<Rect> ground_robot;
            string RobotClassifier = "/home/cirl/ros_gz_project_template-main/ros_gz_example_application/src/cascade.xml";
            if(!Robot_cascade.load(RobotClassifier)){
                RCLCPP_INFO(this->get_logger(), "Could not load the classifier");
            }else{
                RCLCPP_INFO(this->get_logger(), "Load the classifier");
            }
            Robot_cascade.detectMultiScale(frame_gray, ground_robot);
        
            for(size_t i = 0; i < ground_robot.size(); i++){
                rectangle(frame, ground_robot[i], Scalar(0, 255, 0), 2);
            }

            auto out_msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
            publisher_.publish(out_msg);
        }

        void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg){
            RCLCPP_INFO(this->get_logger(), "Received image callback triggered.");
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            }catch(cv_bridge::Exception &e){
                RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
                return;
            }
            RobotDetection(cv_ptr, msg->header);
        }

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
        image_transport::Publisher publisher_;

};


int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}