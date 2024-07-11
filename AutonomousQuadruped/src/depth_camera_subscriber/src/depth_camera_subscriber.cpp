#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

int counter = 0;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // 这里编写处理接收到的图像消息的代码
    ROS_INFO("Received image with width: %d, height: %d", msg->width, msg->height);

    cv::Mat img(msg->height, msg->width, CV_16UC1, const_cast<uint8_t*>(msg->data.data()));
    // 将 ROS 图像消息转换为 OpenCV 图像格式
    cv::Mat image;
    try {
        image = cv::imdecode(img, cv::IMREAD_GRAYSCALE);
    } catch (cv::Exception& e) {
        ROS_ERROR("Failed to decode image message: %s", e.what());
        return;
    }

    // 检查图像是否成功加载
    if (image.empty()) {
        ROS_ERROR("Empty image received");
        return;
    }

    // 保存图像到文件
    std::string filename = "~/img/image_" + std::to_string(counter) + ".jpg";  // 替换为你想要保存的路径和文件名
    try {
        cv::imwrite(filename, image);
        counter++;
        ROS_INFO("Saved image: %s", filename.c_str());
    } catch (cv::Exception& e) {
        ROS_ERROR("Failed to save image: %s", e.what());
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_camera_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/realsense/depth/image", 10, imageCallback);

    ros::spin();

    return 0;
}
