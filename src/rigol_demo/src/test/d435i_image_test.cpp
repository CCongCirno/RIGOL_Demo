#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "librealsense2/rs.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

struct MouseState
{
    cv::Point point{0, 0};
    bool valid = false;
    float depth_m = 0.0f;
};

struct ProgramOptions
{
    std::string rgb_topic = "/camera/rgb/image_raw";
    std::string depth_raw_topic = "/camera/depth/image_raw";
    std::string depth_color_topic = "/camera/depth/image_color";
    std::string camera_info_topic = "/camera/rgb/camera_info";
    std::string rgb_window = "D435i RGB";
    std::string depth_window = "D435i Depth (Pseudo Color)";
    std::string color_frame_id = "camera_color_optical_frame";
    std::string depth_frame_id = "camera_depth_optical_frame";
};

static void mouseCallback(int event, int x, int y, int /*flags*/, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN)
    {
        auto *state = static_cast<MouseState *>(userdata);
        state->point = cv::Point(x, y);
        state->valid = true;
    }
}

class D435IImageTestNode : public rclcpp::Node
{
public:
    D435IImageTestNode()
    : Node("d435i_image_test")
    {
        declare_parameter<std::string>("rgb_topic", options_.rgb_topic);
        declare_parameter<std::string>("depth_raw_topic", options_.depth_raw_topic);
        declare_parameter<std::string>("depth_color_topic", options_.depth_color_topic);
        declare_parameter<std::string>("camera_info_topic", options_.camera_info_topic);
        declare_parameter<std::string>("rgb_window", options_.rgb_window);
        declare_parameter<std::string>("depth_window", options_.depth_window);
        declare_parameter<std::string>("color_frame_id", options_.color_frame_id);
        declare_parameter<std::string>("depth_frame_id", options_.depth_frame_id);

        get_parameter("rgb_topic", options_.rgb_topic);
        get_parameter("depth_raw_topic", options_.depth_raw_topic);
        get_parameter("depth_color_topic", options_.depth_color_topic);
        get_parameter("camera_info_topic", options_.camera_info_topic);
        get_parameter("rgb_window", options_.rgb_window);
        get_parameter("depth_window", options_.depth_window);
        get_parameter("color_frame_id", options_.color_frame_id);
        get_parameter("depth_frame_id", options_.depth_frame_id);

        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            options_.rgb_topic, 10,
            std::bind(&D435IImageTestNode::rgbCallback, this, std::placeholders::_1));
        depth_raw_sub_ = create_subscription<sensor_msgs::msg::Image>(
            options_.depth_raw_topic, 10,
            std::bind(&D435IImageTestNode::depthRawCallback, this, std::placeholders::_1));
        depth_color_sub_ = create_subscription<sensor_msgs::msg::Image>(
            options_.depth_color_topic, 10,
            std::bind(&D435IImageTestNode::depthColorCallback, this, std::placeholders::_1));
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            options_.camera_info_topic, 10,
            std::bind(&D435IImageTestNode::cameraInfoCallback, this, std::placeholders::_1));

        cv::namedWindow(options_.rgb_window, cv::WINDOW_AUTOSIZE);
        cv::namedWindow(options_.depth_window, cv::WINDOW_AUTOSIZE);
        cv::setMouseCallback(options_.rgb_window, mouseCallback, &mouse_state_);

        RCLCPP_INFO(get_logger(), "D435i image test node started and waiting for topic data.");
    }

    bool updateWindows()
    {
        cv::Mat rgb_image;
        cv::Mat depth_color_image;
        cv::Mat depth_raw_image;
        rs2_intrinsics intrinsics;
        bool has_intrinsics;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            if (rgb_image_.empty() || depth_color_image_.empty())
            {
                return true;
            }
            rgb_image = rgb_image_.clone();
            depth_color_image = depth_color_image_.clone();
            depth_raw_image = depth_raw_image_.clone();
            intrinsics = color_intrinsics_;
            has_intrinsics = has_color_intrinsics_;
        }

        if (mouse_state_.valid && has_intrinsics && !depth_raw_image.empty())
        {
            mouse_state_.depth_m = queryDepthAtPoint(depth_raw_image, intrinsics, mouse_state_.point);
            std::ostringstream text;
            text << std::fixed << std::setprecision(3) << mouse_state_.depth_m << " m";
            cv::putText(rgb_image, text.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            cv::circle(rgb_image, mouse_state_.point, 4, cv::Scalar(0, 255, 0), cv::FILLED);
        }
        else
        {
            cv::putText(rgb_image, "Move mouse over image to read depth", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        }

        cv::imshow(options_.rgb_window, rgb_image);
        cv::imshow(options_.depth_window, depth_color_image);

        const int key = cv::waitKey(1);
        if (key == 27 || key == 'q' || key == 'Q')
        {
            return false;
        }
        return true;
    }

private:
    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        rgb_image_ = imageFromMessage(msg);
    }

    void depthRawCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        depth_raw_image_ = imageFromMessage(msg);
    }

    void depthColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        depth_color_image_ = imageFromMessage(msg);
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        color_intrinsics_ = cameraInfoToRs2Intrinsics(*msg);
        has_color_intrinsics_ = true;
    }

    cv::Mat imageFromMessage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding == "rgb8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
            cv::Mat converted;
            cv::cvtColor(image, converted, cv::COLOR_RGB2BGR);
            return converted.clone();
        }
        if (msg->encoding == "bgr8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<uint8_t *>(msg->data.data()));
            return image.clone();
        }
        if (msg->encoding == "16UC1")
        {
            cv::Mat image(msg->height, msg->width, CV_16UC1, const_cast<uint8_t *>(msg->data.data()));
            return image.clone();
        }
        if (msg->encoding == "mono8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC1, const_cast<uint8_t *>(msg->data.data()));
            return image.clone();
        }
        RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
        return cv::Mat();
    }

    rs2_intrinsics cameraInfoToRs2Intrinsics(const sensor_msgs::msg::CameraInfo &info)
    {
        rs2_intrinsics intrin;
        intrin.width = info.width;
        intrin.height = info.height;
        intrin.ppx = static_cast<float>(info.k[2]);
        intrin.ppy = static_cast<float>(info.k[5]);
        intrin.fx = static_cast<float>(info.k[0]);
        intrin.fy = static_cast<float>(info.k[4]);
        intrin.model = RS2_DISTORTION_BROWN_CONRADY;
        for (int i = 0; i < 5; ++i)
        {
            intrin.coeffs[i] = static_cast<float>(info.d[i]);
        }
        return intrin;
    }

    float queryDepthAtPoint(const cv::Mat &depth_raw,
                            const rs2_intrinsics &intrin,
                            const cv::Point &point)
    {
        const int x = std::clamp(point.x, 0, depth_raw.cols - 1);
        const int y = std::clamp(point.y, 0, depth_raw.rows - 1);
        float depth_m = depthValue(depth_raw, x, y);
        if (depth_m <= 0.0f)
        {
            depth_m = getDepthNeighborAverage(depth_raw, x, y, 2);
        }
        if (depth_m > 0.0f)
        {
            float p[3];
            float pix[2] = {static_cast<float>(x), static_cast<float>(y)};
            rs2_deproject_pixel_to_point(p, &intrin, pix, depth_m);
            return p[2];
        }
        return 0.0f;
    }

    float depthValue(const cv::Mat &depth_raw, int x, int y)
    {
        const uint16_t raw = depth_raw.at<uint16_t>(y, x);
        return static_cast<float>(raw) * 0.001f;
    }

    float getDepthNeighborAverage(const cv::Mat &depth_raw, int x, int y, int radius)
    {
        int w = depth_raw.cols;
        int h = depth_raw.rows;
        float sum = 0.0f;
        int count = 0;
        for (int yy = std::max(0, y - radius); yy <= std::min(h - 1, y + radius); ++yy)
        {
            for (int xx = std::max(0, x - radius); xx <= std::min(w - 1, x + radius); ++xx)
            {
                float d = depthValue(depth_raw, xx, yy);
                if (d > 0.0f)
                {
                    sum += d;
                    ++count;
                }
            }
        }
        return count > 0 ? sum / count : 0.0f;
    }

    ProgramOptions options_;
    MouseState mouse_state_;
    std::mutex frame_mutex_;
    cv::Mat rgb_image_;
    cv::Mat depth_color_image_;
    cv::Mat depth_raw_image_;
    rs2_intrinsics color_intrinsics_;
    bool has_color_intrinsics_ = false;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_raw_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<D435IImageTestNode>();

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        if (!node->updateWindows())
        {
            break;
        }
        rate.sleep();
    }

    cv::destroyAllWindows();
    rclcpp::shutdown();
    return 0;
}
