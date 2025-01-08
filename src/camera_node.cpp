#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher()
        : Node("usb_camera_publisher")
    {
        cap_ = cv::VideoCapture(0);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open camera");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Camera opened successfully");
    }
    ~CameraPublisher()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down");
    }
    void init(image_transport::ImageTransport *it, image_transport::Publisher *pub)
    {
        it_ = it;
        pub_ = pub;
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CameraPublisher::publish_image, this));
        count_ = 0;
    }

private:
    void publish_image()
    {
        cv::Mat frame;
        cap_ >> frame;
        cv::imwrite("captured_image.jpg", frame);
        count_++;
        RCLCPP_INFO(this->get_logger(), "Publishing frame %d", count_);
        if (frame.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty frame");
            return;
        }

        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        pub_->publish(msg);
    }

    image_transport::ImageTransport *it_;
    image_transport::Publisher *pub_;
    cv::VideoCapture cap_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraPublisher>();
    image_transport::ImageTransport it(node);
    image_transport::Publisher pub = it.advertise("camera/image", 1);
    node->init(&it, &pub);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
