#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

#include "devices/camera/d435i_video_capture.hpp"

using namespace std::chrono_literals;

class D435IDataPublisherNode : public rclcpp::Node
{
public:
  D435IDataPublisherNode()
  : Node("d435i_data_publisher_node")
  {
    declare_parameter<int>("color_width", 1280);
    declare_parameter<int>("color_height", 720);
    declare_parameter<int>("depth_width", 1280);
    declare_parameter<int>("depth_height", 720);
    declare_parameter<int>("fps", 30);
    declare_parameter<std::string>("rgb_topic", "/camera/rgb/image_raw");
    declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");
    declare_parameter<std::string>("depth_color_topic", "/camera/depth/image_color");
    declare_parameter<std::string>("color_camera_info_topic", "/camera/rgb/camera_info");
    declare_parameter<std::string>("color_frame_id", "camera_color_optical_frame");
    declare_parameter<std::string>("depth_frame_id", "camera_depth_optical_frame");

    color_width_ = get_parameter("color_width").as_int();
    color_height_ = get_parameter("color_height").as_int();
    depth_width_ = get_parameter("depth_width").as_int();
    depth_height_ = get_parameter("depth_height").as_int();
    fps_ = get_parameter("fps").as_int();
    rgb_topic_ = get_parameter("rgb_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    depth_color_topic_ = get_parameter("depth_color_topic").as_string();
    color_camera_info_topic_ = get_parameter("color_camera_info_topic").as_string();
    color_frame_id_ = get_parameter("color_frame_id").as_string();
    depth_frame_id_ = get_parameter("depth_frame_id").as_string();

    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>(rgb_topic_, 10);
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_topic_, 10);
    depth_color_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_color_topic_, 10);
    color_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(color_camera_info_topic_, 10);

    RCLCPP_INFO(get_logger(), "D435i data publisher node created.");
  }

  void startCameraThread()
  {
    running_.store(true);
    camera_thread_ = std::thread(&D435IDataPublisherNode::cameraLoop, this);
  }

  void stopCameraThread()
  {
    running_.store(false);
    if (camera_thread_.joinable())
    {
      camera_thread_.join();
    }
  }

private:
  void cameraLoop()
  {
    RS_D435i camera(color_width_, color_height_, depth_width_, depth_height_, fps_);
    camera.init();

    const rs2_intrinsics &intrin_color = camera.getIntrinColor();
    auto color_info = buildCameraInfo(intrin_color, color_frame_id_);

    rclcpp::Rate rate(fps_);
    while (rclcpp::ok() && running_.load())
    {
      cv::Mat rgb_bgr;
      rs2::depth_frame depth_frame(nullptr);
      cv::Mat depth_color_bgr;
      camera.getImage(rgb_bgr, depth_frame, depth_color_bgr);
      if (rgb_bgr.empty() || !depth_frame)
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for valid D435i frames.");
        rate.sleep();
        continue;
      }

      const auto stamp = now();
      auto rgb_msg = buildImageMessage(rgb_bgr, "rgb8", color_frame_id_, stamp);
      auto depth_color_msg = buildImageMessage(depth_color_bgr, "rgb8", depth_frame_id_, stamp);
      auto depth_msg = buildDepthImageMessage(depth_frame, depth_frame_id_, stamp);

      rgb_pub_->publish(rgb_msg);
      depth_color_pub_->publish(depth_color_msg);
      depth_pub_->publish(depth_msg);

      color_info.header.stamp = stamp;
      color_info_pub_->publish(color_info);

      rate.sleep();
    }
  }

  sensor_msgs::msg::Image buildImageMessage(const cv::Mat &image_bgr,
                                           const std::string &encoding,
                                           const std::string &frame_id,
                                           const rclcpp::Time &stamp)
  {
    cv::Mat image;
    if (image_bgr.channels() == 3)
    {
      cv::cvtColor(image_bgr, image, cv::COLOR_BGR2RGB);
    }
    else
    {
      image = image_bgr;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = image.rows;
    msg.width = image.cols;
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(image.step);
    msg.data.assign(image.data, image.data + image.total() * image.elemSize());
    return msg;
  }

  sensor_msgs::msg::Image buildDepthImageMessage(const rs2::depth_frame &depth_frame,
                                                 const std::string &frame_id,
                                                 const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = depth_frame.get_height();
    msg.width = depth_frame.get_width();
    msg.encoding = "16UC1";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(msg.width * sizeof(uint16_t));
    const uint8_t *data_ptr = reinterpret_cast<const uint8_t *>(depth_frame.get_data());
    const size_t size = static_cast<size_t>(msg.step) * msg.height;
    msg.data.assign(data_ptr, data_ptr + size);
    return msg;
  }

  sensor_msgs::msg::CameraInfo buildCameraInfo(const rs2_intrinsics &intrinsics,
                                               const std::string &frame_id)
  {
    sensor_msgs::msg::CameraInfo info;
    info.header.frame_id = frame_id;
    info.height = intrinsics.height;
    info.width = intrinsics.width;
    info.distortion_model = "plumb_bob";
    info.d = std::vector<double>(intrinsics.coeffs, intrinsics.coeffs + 5);
    info.k = {intrinsics.fx, 0.0, intrinsics.ppx,
              0.0, intrinsics.fy, intrinsics.ppy,
              0.0, 0.0, 1.0};
    info.r = {1.0, 0.0, 0.0,
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0};
    info.p = {intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
              0.0, intrinsics.fy, intrinsics.ppy, 0.0,
              0.0, 0.0, 1.0, 0.0};
    return info;
  }

  int color_width_;
  int color_height_;
  int depth_width_;
  int depth_height_;
  int fps_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string depth_color_topic_;
  std::string color_camera_info_topic_;
  std::string color_frame_id_;
  std::string depth_frame_id_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_color_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr color_info_pub_;

  std::thread camera_thread_;
  std::atomic<bool> running_{false};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<D435IDataPublisherNode>();
  node->startCameraThread();

  rclcpp::spin(node);

  node->stopCameraThread();
  rclcpp::shutdown();
  return 0;
}
