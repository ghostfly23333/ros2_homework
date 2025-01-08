#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>  // 使用Float32MultiArray消息类型

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
        : Node("camera_subscriber")
    {
        // 发布物体识别结果
        pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("object_detection", 10);

        // 设置发布频率为 10Hz
        rate_ = std::make_shared<rclcpp::Rate>(10);
    }
    ~CameraSubscriber()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down");
    }

    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
    {
        // 转换 ROS 图像消息为 OpenCV 图像
        cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

        // 物体识别处理（示例：简单的Haar Cascades或深度学习模型）
        std::vector<cv::Rect> objects;

        // 假设你已经加载了物体检测模型（例如 Haar Cascade、YOLO、SSD 等）
        // 在这里使用 OpenCV 的 Haar Cascade 示例（你可以替换为自己的模型）
        cv::CascadeClassifier object_cascade;
        object_cascade.load("/home/lrc/ros2_ws/opencv-files/haarcascade_frontalface_default.xml");  // 替换为你的模型路径
        object_cascade.detectMultiScale(img, objects);

        // 打印识别到的物体数量
        RCLCPP_INFO(this->get_logger(), "Detected %zu objects.", objects.size());

        // 保存图像到文件（每次回调时保存）
        static int count = 0;  // 用于命名保存的文件
        std::string filename = "/home/lrc/ros2_ws/image_gets/test_image.jpg";
        cv::imwrite(filename, img);  // 保存图像为 JPG 文件

        // 构造 Float32MultiArray 消息
        auto result_msg = std_msgs::msg::Float32MultiArray();
        for (const auto& obj : objects)
        {
            // 每个物体包含 x, y, width, height 四个信息
            result_msg.data.push_back(obj.x);      // x坐标
            result_msg.data.push_back(obj.y);      // y坐标
            result_msg.data.push_back(obj.width);  // 宽度
            result_msg.data.push_back(obj.height); // 高度
        }

        // 发布物体识别结果
        pub_->publish(result_msg);

        // 控制发布频率为 10Hz
        rate_->sleep();
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
    std::shared_ptr<rclcpp::Rate> rate_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriber>();
    image_transport::ImageTransport it(node);
    image_transport::Subscriber sub = it.subscribe(
        "camera/image", 1, 
        [node](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            node->CameraSubscriber::image_callback(msg);
        });
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
