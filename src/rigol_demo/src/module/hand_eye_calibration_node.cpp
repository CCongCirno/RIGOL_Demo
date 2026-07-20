// hand_eye_calibration_node.cpp
// 手眼标定节点 (眼在手上 / Eye-in-Hand)
// 相机安装在机械臂末端,标定 camera_link -> link6 的变换
//
// 工作流程 (预标定 + 正式标定, OpenCV 窗口交互):
//   预标定 (PREPARE):
//     1. 按 'p' 进入预标定模式
//     2. 手动拖动机械臂到不同位姿(确保棋盘格可见)
//     3. 按 SPACE 采集当前末端位姿
//     4. 采集足够位姿后按 'e' 结束预标定
//   正式标定 (CALIBRATION):
//     1. 按 'c' 进入正式标定模式
//     2. 节点自动加载预标定保存的位姿,依次发布到 /target_pose
//     3. 机器人到达每个位姿后自动检测棋盘格并采集图像
//     4. 所有位姿采集完成后按 'r' 执行标定计算
//     5. 标定结果保存到 configs/hand_eye_result.yaml
//
// 键盘操作:
//   p      - 进入预标定模式
//   SPACE  - 采集当前样本(预标定:保存末端位姿)
//   e      - 结束预标定
//   c      - 进入正式标定模式(自动运动到保存的位姿)
//   r      - 执行手眼标定计算
//   d      - 删除最后一个样本(预标定阶段)
//   s      - 保存标定结果(标定完成后)
//   x      - 重置所有样本和数据
//   q/ESC  - 退出

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_prefix.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "Eigen/Dense"
#include "demo_msgs/msg/pose_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "piper_msgs/msg/piper_status_msg.hpp"
#include "piper_msgs/srv/enable.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/header.hpp"

namespace fs = std::filesystem;

// ============================================================
// 标定模式
// ============================================================
enum class CalibrationMode
{
    PENDING,
    PREPARE,
    CALIBRATION
};

// 正式标定阶段状态机
enum class CalibrationStatus
{
    IDLE,
    MOVE,
    WAIT,
    DETECT
};

// ============================================================
// 配置参数结构体
// ============================================================
struct CalibrationConfig
{
    // 话题
    std::string rgb_topic = "/camera/rgb/image_raw";
    std::string camera_info_topic = "/camera/rgb/camera_info";
    std::string joint_state_topic = "/joint_states";
    std::string arm_status_topic = "/arm_status";
    std::string end_pose_topic = "/end_pose_stamped";
    std::string pose_goal_topic = "/target_pose";
    int planner_mode = 0;

    // TF
    std::string base_frame = "base_link";
    std::string ee_frame = "link6";
    std::string camera_frame = "camera_color_optical_frame";

    // 标定板
    int board_width = 11;
    int board_height = 8;
    double square_size = 0.020;

    // 相机内参
    bool use_custom_camera_matrix = true;
    double fx = 891.773183;
    double fy = 893.753426;
    double cx = 628.465114;
    double cy = 353.375401;

    // 采样
    int min_samples = 5;
    int max_samples = 20;

    // 标定方法
    int hand_eye_method = 1;

    // 运动控制
    double pose_wait_time = 5.0;
    std::string target_frame = "base_link";
    std::string enable_topic = "/enable_flag";
    std::string enable_service = "enable_srv";
    bool enable_before_calibration = true;

    // 结果保存
    std::string result_file = "hand_eye_result.yaml";
    std::string robot_pose_file = "robot_pose.yaml";
    std::string calibration_data_file = "calibration_data.yaml";

    // 可视化
    std::string window_name = "Hand-Eye Calibration";
    bool show_corners = true;
};

// ============================================================
// 手眼标定节点
// ============================================================
class HandEyeCalibrationNode : public rclcpp::Node
{
public:
    HandEyeCalibrationNode() : Node("hand_eye_calibration_node")
    {
        loadConfig();
        calibrationParamInit();
        chessBoardPointInit();

        // 相机内参与相机画面获取
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            config_.camera_info_topic, 10,
            std::bind(&HandEyeCalibrationNode::cameraInfoCallback, this, std::placeholders::_1));
        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            config_.rgb_topic, 1,
            std::bind(&HandEyeCalibrationNode::imageCallback, this, std::placeholders::_1));

        // 机器人状态获取 (订阅 /joint_states 读取电机角度)
        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            config_.joint_state_topic, 10,
            std::bind(&HandEyeCalibrationNode::jointStateCallback, this, std::placeholders::_1));
        arm_status_sub_ = create_subscription<piper_msgs::msg::PiperStatusMsg>(
            config_.arm_status_topic, 10,
            std::bind(&HandEyeCalibrationNode::armStatusCallback, this, std::placeholders::_1));

        // 末端位姿获取 (用于自由规划运动)
        end_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            config_.end_pose_topic, 10,
            std::bind(&HandEyeCalibrationNode::endPoseCallback, this, std::placeholders::_1));

        // 发布 PoseGoal 到 /target_pose,由自由规划接口执行
        pose_goal_pub_ = create_publisher<demo_msgs::msg::PoseGoal>(
            config_.pose_goal_topic, 10);

        enable_flag_pub_ = create_publisher<std_msgs::msg::Bool>(
            config_.enable_topic, 10);
        enable_srv_client_ = create_client<piper_msgs::srv::Enable>(
            config_.enable_service);

        // 创建可视化窗口
        cv::namedWindow(config_.window_name, cv::WINDOW_AUTOSIZE);

        RCLCPP_INFO(get_logger(), "=== 手眼标定节点已启动 (眼在手上) ===");
        RCLCPP_INFO(get_logger(), "标定板: %dx%d, 方格大小: %.3f m",
                    config_.board_width, config_.board_height, config_.square_size);
        RCLCPP_INFO(get_logger(), "操作说明:");
        RCLCPP_INFO(get_logger(), "  p     - 进入预标定模式");
        RCLCPP_INFO(get_logger(), "  SPACE - 采集当前样本(预标定)");
        RCLCPP_INFO(get_logger(), "  e     - 结束预标定");
        RCLCPP_INFO(get_logger(), "  c     - 进入正式标定模式");
        RCLCPP_INFO(get_logger(), "  r     - 执行手眼标定计算");
        RCLCPP_INFO(get_logger(), "  d     - 删除最后一个样本");
        RCLCPP_INFO(get_logger(), "  s     - 保存标定结果");
        RCLCPP_INFO(get_logger(), "  x     - 重置所有样本");
        RCLCPP_INFO(get_logger(), "  q/ESC - 退出");
    }

    ~HandEyeCalibrationNode() override
    {
        cv::destroyWindow(config_.window_name);
    }

    // 主循环:处理键盘输入和显示(事件驱动,有新图像才处理)
    bool spinAndUpdate()
    {
        cv::Mat raw;
        {
            // 等待新图像到达,超时 500ms 以便响应退出
            std::unique_lock<std::mutex> lock(frame_mutex_);
            frame_cv_.wait_for(lock, std::chrono::milliseconds(500),
                [this]() { return !rgb_image_.empty() || !running_.load(); });
            if (!rgb_image_.empty())
            {
                raw = rgb_image_;
                rgb_image_.release();
            }
        }

        if (raw.empty())
        {
            return true;
        }

        // raw_frame_ 保存原始图像引用(浅拷贝),供 captureSample 使用
        raw_frame_ = raw;
        cv::Mat display = raw.clone();

        // 实时检测并绘制棋盘格角点
        detectAndDrawChessboard(display);

        // 正式标定模式的状态机处理
        if (calibration_mode_ == CalibrationMode::CALIBRATION)
        {
            handleCalibrationStateMachine();
        }

        // 在图像上绘制状态信息
        drawOverlay(display);

        cv::imshow(config_.window_name, display);

        // waitKey(1) 保证 GUI 有时间刷新
        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 27)  // q or ESC
        {
            RCLCPP_INFO(get_logger(), "退出标定节点。");
            return false;
        }
        else if (key == 'p')  // 进入预标定模式
        {
            enterPrepareMode();
        }
        else if (key == ' ')  // SPACE: 采集样本
        {
            captureSample();
        }
        else if (key == 'e')  // 结束预标定
        {
            endPrepareMode();
        }
        else if (key == 'c')  // 进入正式标定模式
        {
            enterCalibrationMode();
        }
        else if (key == 'r')  // 执行标定计算
        {
            runCalibration();
        }
        else if (key == 'd')  // 删除最后一个样本
        {
            deleteLastSample();
        }
        else if (key == 's')  // 保存结果
        {
            saveHandEyeTransformer();
        }
        else if (key == 'x')  // 重置
        {
            resetAll();
        }

        return true;
    }

    // 请求退出(供 main 线程调用)
    void requestShutdown()
    {
        running_.store(false);
        frame_cv_.notify_all();
    }

private:
    // ---------- 加载配置文件 ----------
    void loadConfig()
    {
        declare_parameter<std::string>("config_file", "");
        declare_parameter<std::string>("rgb_topic", config_.rgb_topic);
        declare_parameter<std::string>("camera_info_topic", config_.camera_info_topic);
        declare_parameter<std::string>("joint_state_topic", config_.joint_state_topic);
        declare_parameter<std::string>("arm_status_topic", config_.arm_status_topic);
        declare_parameter<std::string>("end_pose_topic", config_.end_pose_topic);
        declare_parameter<std::string>("pose_goal_topic", config_.pose_goal_topic);
        declare_parameter<int>("planner_mode", config_.planner_mode);
        declare_parameter<std::string>("enable_topic", config_.enable_topic);
        declare_parameter<std::string>("enable_service", config_.enable_service);
        declare_parameter<bool>("enable_before_calibration", config_.enable_before_calibration);
        declare_parameter<std::string>("base_frame", config_.base_frame);
        declare_parameter<std::string>("ee_frame", config_.ee_frame);
        declare_parameter<std::string>("camera_frame", config_.camera_frame);
        declare_parameter<int>("board_width", config_.board_width);
        declare_parameter<int>("board_height", config_.board_height);
        declare_parameter<double>("square_size", config_.square_size);
        declare_parameter<bool>("use_custom_camera_matrix", config_.use_custom_camera_matrix);
        declare_parameter<double>("fx", config_.fx);
        declare_parameter<double>("fy", config_.fy);
        declare_parameter<double>("cx", config_.cx);
        declare_parameter<double>("cy", config_.cy);
        declare_parameter<int>("min_samples", config_.min_samples);
        declare_parameter<int>("max_samples", config_.max_samples);
        declare_parameter<int>("hand_eye_method", config_.hand_eye_method);
        declare_parameter<double>("pose_wait_time", config_.pose_wait_time);
        declare_parameter<std::string>("target_frame", config_.target_frame);
        declare_parameter<std::string>("result_file", config_.result_file);
        declare_parameter<std::string>("robot_pose_file", config_.robot_pose_file);
        declare_parameter<std::string>("calibration_data_file", config_.calibration_data_file);
        declare_parameter<std::string>("window_name", config_.window_name);
        declare_parameter<bool>("show_corners", config_.show_corners);

        // 尝试从 YAML 配置文件加载
        std::string config_file = get_parameter("config_file").as_string();
        if (!config_file.empty() && fs::exists(config_file))
        {
            RCLCPP_INFO(get_logger(), "加载配置文件: %s", config_file.c_str());
            YAML::Node yaml = YAML::LoadFile(config_file);

            // 保存配置文件所在目录,用于结果保存 (configs 文件夹)
            config_dir_ = fs::path(config_file).parent_path().string();

            if (yaml["rgb_topic"]) config_.rgb_topic = yaml["rgb_topic"].as<std::string>();
            if (yaml["camera_info_topic"]) config_.camera_info_topic = yaml["camera_info_topic"].as<std::string>();
            if (yaml["joint_state_topic"]) config_.joint_state_topic = yaml["joint_state_topic"].as<std::string>();
            if (yaml["arm_status_topic"]) config_.arm_status_topic = yaml["arm_status_topic"].as<std::string>();
            if (yaml["end_pose_topic"]) config_.end_pose_topic = yaml["end_pose_topic"].as<std::string>();
            if (yaml["pose_goal_topic"]) config_.pose_goal_topic = yaml["pose_goal_topic"].as<std::string>();
            if (yaml["planner_mode"]) config_.planner_mode = yaml["planner_mode"].as<int>();
            if (yaml["enable_topic"]) config_.enable_topic = yaml["enable_topic"].as<std::string>();
            if (yaml["enable_service"]) config_.enable_service = yaml["enable_service"].as<std::string>();
            if (yaml["enable_before_calibration"]) config_.enable_before_calibration = yaml["enable_before_calibration"].as<bool>();
            if (yaml["base_frame"]) config_.base_frame = yaml["base_frame"].as<std::string>();
            if (yaml["ee_frame"]) config_.ee_frame = yaml["ee_frame"].as<std::string>();
            if (yaml["camera_frame"]) config_.camera_frame = yaml["camera_frame"].as<std::string>();
            if (yaml["board_width"]) config_.board_width = yaml["board_width"].as<int>();
            if (yaml["board_height"]) config_.board_height = yaml["board_height"].as<int>();
            if (yaml["square_size"]) config_.square_size = yaml["square_size"].as<double>();
            if (yaml["use_custom_camera_matrix"]) config_.use_custom_camera_matrix = yaml["use_custom_camera_matrix"].as<bool>();
            // 相机内参 (嵌套在 camera_matrix 下)
            if (yaml["camera_matrix"])
            {
                YAML::Node cm = yaml["camera_matrix"];
                if (cm["fx"]) config_.fx = cm["fx"].as<double>();
                if (cm["fy"]) config_.fy = cm["fy"].as<double>();
                if (cm["cx"]) config_.cx = cm["cx"].as<double>();
                if (cm["cy"]) config_.cy = cm["cy"].as<double>();
            }
            if (yaml["min_samples"]) config_.min_samples = yaml["min_samples"].as<int>();
            if (yaml["max_samples"]) config_.max_samples = yaml["max_samples"].as<int>();
            if (yaml["hand_eye_method"]) config_.hand_eye_method = yaml["hand_eye_method"].as<int>();
            if (yaml["pose_wait_time"]) config_.pose_wait_time = yaml["pose_wait_time"].as<double>();
            if (yaml["target_frame"]) config_.target_frame = yaml["target_frame"].as<std::string>();
            if (yaml["result_file"]) config_.result_file = yaml["result_file"].as<std::string>();
            if (yaml["robot_pose_file"]) config_.robot_pose_file = yaml["robot_pose_file"].as<std::string>();
            if (yaml["calibration_data_file"]) config_.calibration_data_file = yaml["calibration_data_file"].as<std::string>();
            if (yaml["window_name"]) config_.window_name = yaml["window_name"].as<std::string>();
            if (yaml["show_corners"]) config_.show_corners = yaml["show_corners"].as<bool>();

            // 将 YAML 加载的值同步到 ROS2 参数,使后续 get_parameter 能取到 YAML 值
            // (否则 get_parameter 返回 declare_parameter 的默认值,覆盖 YAML)
            set_parameter(rclcpp::Parameter("rgb_topic", config_.rgb_topic));
            set_parameter(rclcpp::Parameter("camera_info_topic", config_.camera_info_topic));
            set_parameter(rclcpp::Parameter("joint_state_topic", config_.joint_state_topic));
            set_parameter(rclcpp::Parameter("arm_status_topic", config_.arm_status_topic));
            set_parameter(rclcpp::Parameter("end_pose_topic", config_.end_pose_topic));
            set_parameter(rclcpp::Parameter("pose_goal_topic", config_.pose_goal_topic));
            set_parameter(rclcpp::Parameter("planner_mode", config_.planner_mode));
            set_parameter(rclcpp::Parameter("enable_topic", config_.enable_topic));
            set_parameter(rclcpp::Parameter("enable_service", config_.enable_service));
            set_parameter(rclcpp::Parameter("enable_before_calibration", config_.enable_before_calibration));
            set_parameter(rclcpp::Parameter("base_frame", config_.base_frame));
            set_parameter(rclcpp::Parameter("ee_frame", config_.ee_frame));
            set_parameter(rclcpp::Parameter("camera_frame", config_.camera_frame));
            set_parameter(rclcpp::Parameter("board_width", config_.board_width));
            set_parameter(rclcpp::Parameter("board_height", config_.board_height));
            set_parameter(rclcpp::Parameter("square_size", config_.square_size));
            set_parameter(rclcpp::Parameter("use_custom_camera_matrix", config_.use_custom_camera_matrix));
            set_parameter(rclcpp::Parameter("fx", config_.fx));
            set_parameter(rclcpp::Parameter("fy", config_.fy));
            set_parameter(rclcpp::Parameter("cx", config_.cx));
            set_parameter(rclcpp::Parameter("cy", config_.cy));
            set_parameter(rclcpp::Parameter("min_samples", config_.min_samples));
            set_parameter(rclcpp::Parameter("max_samples", config_.max_samples));
            set_parameter(rclcpp::Parameter("hand_eye_method", config_.hand_eye_method));
            set_parameter(rclcpp::Parameter("pose_wait_time", config_.pose_wait_time));
            set_parameter(rclcpp::Parameter("target_frame", config_.target_frame));
            set_parameter(rclcpp::Parameter("result_file", config_.result_file));
            set_parameter(rclcpp::Parameter("robot_pose_file", config_.robot_pose_file));
            set_parameter(rclcpp::Parameter("calibration_data_file", config_.calibration_data_file));
            set_parameter(rclcpp::Parameter("window_name", config_.window_name));
            set_parameter(rclcpp::Parameter("show_corners", config_.show_corners));
        }

        config_.rgb_topic = get_parameter("rgb_topic").as_string();
        config_.camera_info_topic = get_parameter("camera_info_topic").as_string();
        config_.joint_state_topic = get_parameter("joint_state_topic").as_string();
        config_.arm_status_topic = get_parameter("arm_status_topic").as_string();
        config_.end_pose_topic = get_parameter("end_pose_topic").as_string();
        config_.pose_goal_topic = get_parameter("pose_goal_topic").as_string();
        config_.planner_mode = get_parameter("planner_mode").as_int();
        config_.base_frame = get_parameter("base_frame").as_string();
        config_.ee_frame = get_parameter("ee_frame").as_string();
        config_.camera_frame = get_parameter("camera_frame").as_string();
        config_.enable_topic = get_parameter("enable_topic").as_string();
        config_.enable_service = get_parameter("enable_service").as_string();
        config_.enable_before_calibration = get_parameter("enable_before_calibration").as_bool();
        config_.board_width = get_parameter("board_width").as_int();
        config_.board_height = get_parameter("board_height").as_int();
        config_.square_size = get_parameter("square_size").as_double();
        config_.use_custom_camera_matrix = get_parameter("use_custom_camera_matrix").as_bool();
        config_.fx = get_parameter("fx").as_double();
        config_.fy = get_parameter("fy").as_double();
        config_.cx = get_parameter("cx").as_double();
        config_.cy = get_parameter("cy").as_double();
        config_.min_samples = get_parameter("min_samples").as_int();
        config_.max_samples = get_parameter("max_samples").as_int();
        config_.hand_eye_method = get_parameter("hand_eye_method").as_int();
        config_.pose_wait_time = get_parameter("pose_wait_time").as_double();
        config_.target_frame = get_parameter("target_frame").as_string();
        config_.result_file = get_parameter("result_file").as_string();
        config_.robot_pose_file = get_parameter("robot_pose_file").as_string();
        config_.calibration_data_file = get_parameter("calibration_data_file").as_string();
        config_.window_name = get_parameter("window_name").as_string();
        config_.show_corners = get_parameter("show_corners").as_bool();

        // 如果未通过配置文件指定 config_dir_,则推导默认路径
        if (config_dir_.empty())
        {
            config_dir_ = getConfigDir();
        }
        RCLCPP_INFO(get_logger(), "配置目录(configs): %s", config_dir_.c_str());
    }

    // 获取 configs 目录路径
    std::string getConfigDir() const
    {
        try
        {
            std::string pkg_prefix = ament_index_cpp::get_package_prefix("rigol_demo");
            fs::path share_dir = fs::path(pkg_prefix) / "share" / "rigol_demo" / "configs";
            if (fs::exists(share_dir))
            {
                return share_dir.string();
            }
            // 回退到源码目录
            fs::path ws_root = fs::path(pkg_prefix).parent_path().parent_path();
            fs::path src_config = ws_root / "src" / "rigol_demo" / "configs";
            if (fs::exists(src_config))
            {
                return src_config.string();
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "获取 configs 目录失败: %s", e.what());
        }
        return fs::current_path().string();
    }

    // ---------- 参数初始化 ----------
    void calibrationParamInit()
    {
        board_size_ = cv::Size(config_.board_width, config_.board_height);
        camera_info_received_ = false;
        calibration_mode_ = CalibrationMode::PENDING;
        saved_poses_count_ = 0;
        saved_images_count_ = 0;
        current_pose_index_ = 0;
        calculate_pose_ = false;
        waiting_at_pose_ = false;
        calibration_status_ = CalibrationStatus::IDLE;
        has_result_ = false;

        // 如果使用自定义内参,直接初始化
        if (config_.use_custom_camera_matrix)
        {
            camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                config_.fx, 0, config_.cx,
                0, config_.fy, config_.cy,
                0, 0, 1);
            dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
            camera_info_received_ = true;
            RCLCPP_INFO(get_logger(), "使用自定义相机内参");
        }
    }

    void chessBoardPointInit()
    {
        board_points_3d_.clear();
        for (int i = 0; i < board_size_.height; i++)
        {
            for (int j = 0; j < board_size_.width; j++)
            {
                board_points_3d_.push_back(cv::Point3f(
                    static_cast<float>(j * config_.square_size),
                    static_cast<float>(i * config_.square_size),
                    0.0f));
            }
        }
    }

    // ---------- 模式切换 ----------
    void enterPrepareMode()
    {
        if (calibration_mode_ == CalibrationMode::PREPARE)
        {
            RCLCPP_WARN(get_logger(), "已在预标定模式");
            return;
        }
        calibration_mode_ = CalibrationMode::PREPARE;
        clearRobotPose();
        saved_poses_.clear();
        saved_poses_count_ = 0;
        calculate_pose_ = false;
        RCLCPP_INFO(get_logger(), "=== 进入预标定模式 ===");
        RCLCPP_INFO(get_logger(), "手动移动机械臂到不同位姿,按 SPACE 采集样本");
    }

    void endPrepareMode()
    {
        if (calibration_mode_ != CalibrationMode::PREPARE)
        {
            RCLCPP_WARN(get_logger(), "当前不在预标定模式");
            return;
        }
        if (saved_poses_count_ < config_.min_samples)
        {
            RCLCPP_WARN(get_logger(), "样本数不足! 当前 %d, 最少需要 %d",
                        saved_poses_count_, config_.min_samples);
            return;
        }
        RCLCPP_INFO(get_logger(), "=== 预标定结束,共保存 %d 组位姿 ===", saved_poses_count_);
        RCLCPP_INFO(get_logger(), "按 'c' 进入正式标定模式");
        calibration_mode_ = CalibrationMode::PENDING;
    }

    void enterCalibrationMode()
    {
        if (calibration_mode_ == CalibrationMode::CALIBRATION)
        {
            RCLCPP_WARN(get_logger(), "已在正式标定模式");
            return;
        }
        // 加载预标定保存的位姿
        loadSavedRobotPoses();
        if (saved_poses_.empty())
        {
            RCLCPP_WARN(get_logger(), "没有保存的位姿,请先进行预标定!");
            return;
        }
        if (config_.enable_before_calibration)
        {
            enableRobot(true);
        }
        calibration_mode_ = CalibrationMode::CALIBRATION;
        clearCalibrationData();
        clearRobotTransform();
        clearImage();
        saved_images_count_ = 0;
        current_pose_index_ = 0;
        calibration_status_ = CalibrationStatus::MOVE;
        calculate_pose_ = true;
        pose_start_time_ = std::chrono::steady_clock::now();
        RCLCPP_INFO(get_logger(), "=== 进入正式标定模式 ===");
        RCLCPP_INFO(get_logger(), "将依次运动到 %zu 个位姿并自动采集图像", saved_poses_.size());
    }

    // ---------- 正式标定状态机 ----------
    void handleCalibrationStateMachine()
    {
        if (calibration_status_ == CalibrationStatus::MOVE)
        {
            moveToNextPose();
        }
        else if (calibration_status_ == CalibrationStatus::WAIT)
        {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - pose_start_time_).count();
            if (elapsed >= config_.pose_wait_time)
            {
                calibration_status_ = CalibrationStatus::DETECT;
                waiting_at_pose_ = false;
            }
        }
        else if (calibration_status_ == CalibrationStatus::DETECT)
        {
            // 检测在 detectAndDrawChessboard 中完成,这里采集数据
            if (!raw_frame_.empty() && !last_corners_.empty() && has_last_detection_)
            {
                if (!raw_frame_.empty() && !last_corners_.empty() && has_last_detection_)
                {
                    calculatePose(last_corners_, std_msgs::msg::Header());
                    const auto &robot_pose = saved_poses_[current_pose_index_ - 1];
                    saveRobotTransform(robot_pose);
                    saveImage(raw_frame_);
                    RCLCPP_INFO(get_logger(), "位姿 %zu 采集完成", current_pose_index_);
                    has_last_detection_ = false;  // 避免重复采集
                }
            }
            calibration_status_ = CalibrationStatus::MOVE;
        }
    }

    // ---------- 数据保存/加载 ----------
    void loadSavedRobotPoses()
    {
        saved_poses_.clear();
        YAML::Node robot_pose_yaml;
        try
        {
            std::string yaml_path = config_dir_ + "/" + config_.robot_pose_file;
            robot_pose_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "加载 robot_pose.yaml 失败: %s", e.what());
            return;
        }

        for (const auto &node : robot_pose_yaml["robot_pose"])
        {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = node["frame_id"] ? node["frame_id"].as<std::string>() : config_.target_frame;

            if (node["position"])
            {
                pose.pose.position.x = node["position"]["x"].as<double>();
                pose.pose.position.y = node["position"]["y"].as<double>();
                pose.pose.position.z = node["position"]["z"].as<double>();
            }
            if (node["orientation"])
            {
                pose.pose.orientation.x = node["orientation"]["x"].as<double>();
                pose.pose.orientation.y = node["orientation"]["y"].as<double>();
                pose.pose.orientation.z = node["orientation"]["z"].as<double>();
                pose.pose.orientation.w = node["orientation"]["w"].as<double>();
            }
            saved_poses_.push_back(pose);
        }

        saved_poses_count_ = static_cast<int>(saved_poses_.size());
        RCLCPP_INFO(get_logger(), "加载 %d 组末端位姿", saved_poses_count_);
    }

    void saveCalibrationData(const geometry_msgs::msg::PoseStamped &pose_msg)
    {
        YAML::Node calibration_data_yaml;
        std::string yaml_path = config_dir_ + "/" + config_.calibration_data_file;

        try
        {
            calibration_data_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            calibration_data_yaml["calibration_data"] = YAML::Node(YAML::NodeType::Sequence);
        }

        YAML::Node new_data;
        new_data["position"]["x"] = pose_msg.pose.position.x;
        new_data["position"]["y"] = pose_msg.pose.position.y;
        new_data["position"]["z"] = pose_msg.pose.position.z;
        new_data["orientation"]["x"] = pose_msg.pose.orientation.x;
        new_data["orientation"]["y"] = pose_msg.pose.orientation.y;
        new_data["orientation"]["z"] = pose_msg.pose.orientation.z;
        new_data["orientation"]["w"] = pose_msg.pose.orientation.w;
        calibration_data_yaml["calibration_data"].push_back(new_data);

        std::ofstream fout(yaml_path);
        if (!fout.is_open()) return;
        fout << calibration_data_yaml;
        fout.close();
    }

    void clearCalibrationData()
    {
        std::string yaml_path = config_dir_ + "/" + config_.calibration_data_file;
        if (fs::exists(yaml_path))
        {
            fs::remove(yaml_path);
        }
    }

    // ---------- 保存/加载机器人末端变换 (base_link -> link6) ----------
    void saveRobotTransform(const geometry_msgs::msg::PoseStamped &pose_msg)
    {
        YAML::Node robot_transform_yaml;
        std::string yaml_path = config_dir_ + "/robot_transform.yaml";

        try
        {
            robot_transform_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            robot_transform_yaml["robot_transform"] = YAML::Node(YAML::NodeType::Sequence);
        }

        YAML::Node new_transform;
        new_transform["translation"]["x"] = pose_msg.pose.position.x;
        new_transform["translation"]["y"] = pose_msg.pose.position.y;
        new_transform["translation"]["z"] = pose_msg.pose.position.z;
        new_transform["rotation"]["x"] = pose_msg.pose.orientation.x;
        new_transform["rotation"]["y"] = pose_msg.pose.orientation.y;
        new_transform["rotation"]["z"] = pose_msg.pose.orientation.z;
        new_transform["rotation"]["w"] = pose_msg.pose.orientation.w;
        robot_transform_yaml["robot_transform"].push_back(new_transform);

        std::ofstream fout(yaml_path);
        if (!fout.is_open()) return;
        fout << robot_transform_yaml;
        fout.close();
    }

    void clearRobotTransform()
    {
        std::string yaml_path = config_dir_ + "/robot_transform.yaml";
        if (fs::exists(yaml_path))
        {
            fs::remove(yaml_path);
        }
    }

    void saveRobotPose()
    {
        std::lock_guard<std::mutex> lock(end_pose_mutex_);
        if (!current_end_pose_)
        {
            RCLCPP_WARN(get_logger(), "未收到末端位姿,无法保存!");
            return;
        }

        geometry_msgs::msg::PoseStamped pose = *current_end_pose_;
        if (pose.header.frame_id.empty())
        {
            pose.header.frame_id = config_.target_frame;
        }

        saved_poses_.push_back(pose);
        saved_poses_count_++;

        RCLCPP_INFO(get_logger(), "样本 #%d 采集成功! 末端位姿 frame=%s pos=(%.3f, %.3f, %.3f)",
                    saved_poses_count_, pose.header.frame_id.c_str(),
                    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        YAML::Node robot_pose_yaml;
        std::string yaml_path = config_dir_ + "/" + config_.robot_pose_file;
        try
        {
            robot_pose_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            robot_pose_yaml["robot_pose"] = YAML::Node(YAML::NodeType::Sequence);
        }

        YAML::Node new_pose;
        new_pose["frame_id"] = pose.header.frame_id;
        new_pose["position"]["x"] = pose.pose.position.x;
        new_pose["position"]["y"] = pose.pose.position.y;
        new_pose["position"]["z"] = pose.pose.position.z;
        new_pose["orientation"]["x"] = pose.pose.orientation.x;
        new_pose["orientation"]["y"] = pose.pose.orientation.y;
        new_pose["orientation"]["z"] = pose.pose.orientation.z;
        new_pose["orientation"]["w"] = pose.pose.orientation.w;
        robot_pose_yaml["robot_pose"].push_back(new_pose);

        std::ofstream fout(yaml_path);
        if (!fout.is_open()) return;
        fout << robot_pose_yaml;
        fout.close();
    }

    bool enableRobot(bool enable)
    {
        if (config_.enable_topic.empty() && config_.enable_service.empty())
        {
            RCLCPP_WARN(get_logger(), "未配置使能接口,跳过机械臂使能");
            return false;
        }

        if (!config_.enable_service.empty() && enable_srv_client_)
        {
            if (!enable_srv_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(get_logger(), "无法连接到 enable_srv,尝试发送 enable_flag 话题");
            }
            else
            {
                auto request = std::make_shared<piper_msgs::srv::Enable::Request>();
                request->enable_request = enable;
                auto result_future = enable_srv_client_->async_send_request(request);
                // 节点已在 spin_thread 中由 rclcpp::spin 持续处理回调,
                // 不能再使用 rclcpp::spin_until_future_complete (会导致
                // "Node has already been added to an executor" 崩溃),
                // 这里直接轮询 future 状态即可。
                auto wait_status = result_future.wait_for(std::chrono::seconds(2));
                if (wait_status == std::future_status::ready)
                {
                    if (result_future.get()->enable_response)
                    {
                        RCLCPP_INFO(get_logger(), "机械臂使能服务调用成功: %s", enable ? "启用" : "禁用");
                        return true;
                    }
                    RCLCPP_WARN(get_logger(), "机械臂使能服务返回失败: %s", enable ? "启用" : "禁用");
                }
                else
                {
                    RCLCPP_WARN(get_logger(), "机械臂使能服务调用超时或失败");
                }
            }
        }

        if (!config_.enable_topic.empty() && enable_flag_pub_)
        {
            std_msgs::msg::Bool msg;
            msg.data = enable;
            enable_flag_pub_->publish(msg);
            RCLCPP_INFO(get_logger(), "已发布 enable_flag: %s", enable ? "true" : "false");
            return true;
        }

        RCLCPP_WARN(get_logger(), "机器臂使能接口不可用");
        return false;
    }

    void clearRobotPose()
    {
        std::string yaml_path = config_dir_ + "/" + config_.robot_pose_file;
        if (fs::exists(yaml_path))
        {
            fs::remove(yaml_path);
        }
    }

    void saveImage(const cv::Mat &image)
    {
        std::string filename = config_dir_ + "/chessboard_" +
                               std::to_string(saved_images_count_) + ".jpg";

        RCLCPP_INFO(get_logger(), "保存图像 #%d", saved_images_count_ + 1);

        if (cv::imwrite(filename, image))
            saved_images_count_++;
    }

    void clearImage()
    {
        for (const auto &entry : fs::directory_iterator(config_dir_))
        {
            if (entry.is_regular_file())
            {
                std::string path = entry.path().string();
                if (path.find("chessboard_") != std::string::npos)
                {
                    fs::remove(entry.path());
                }
            }
        }
    }

    // ---------- 位姿计算 ----------
    void calculatePose(const std::vector<cv::Point2f> &corners, const std_msgs::msg::Header &header)
    {
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(board_points_3d_, corners, camera_matrix_,
                                    dist_coeffs_, rvec, tvec);

        if (success)
        {
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header = header;
            pose_msg.header.frame_id = config_.camera_frame;
            pose_msg.pose.position.x = tvec.at<double>(0);
            pose_msg.pose.position.y = tvec.at<double>(1);
            pose_msg.pose.position.z = tvec.at<double>(2);

            // 旋转向量 -> 旋转矩阵 -> 四元数
            cv::Mat rotation_matrix;
            cv::Rodrigues(rvec, rotation_matrix);
            double trace = rotation_matrix.at<double>(0, 0) + rotation_matrix.at<double>(1, 1) + rotation_matrix.at<double>(2, 2);
            double w, x, y, z;

            if (trace > 0)
            {
                double s = sqrt(trace + 1.0) * 2;
                w = 0.25 * s;
                x = (rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) / s;
                y = (rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) / s;
                z = (rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) / s;
            }
            else if ((rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(1, 1)) &&
                     (rotation_matrix.at<double>(0, 0) > rotation_matrix.at<double>(2, 2)))
            {
                double s = sqrt(1.0 + rotation_matrix.at<double>(0, 0) - rotation_matrix.at<double>(1, 1) - rotation_matrix.at<double>(2, 2)) * 2;
                w = (rotation_matrix.at<double>(2, 1) - rotation_matrix.at<double>(1, 2)) / s;
                x = 0.25 * s;
                y = (rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s;
                z = (rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s;
            }
            else if (rotation_matrix.at<double>(1, 1) > rotation_matrix.at<double>(2, 2))
            {
                double s = sqrt(1.0 + rotation_matrix.at<double>(1, 1) - rotation_matrix.at<double>(0, 0) - rotation_matrix.at<double>(2, 2)) * 2;
                w = (rotation_matrix.at<double>(0, 2) - rotation_matrix.at<double>(2, 0)) / s;
                x = (rotation_matrix.at<double>(0, 1) + rotation_matrix.at<double>(1, 0)) / s;
                y = 0.25 * s;
                z = (rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s;
            }
            else
            {
                double s = sqrt(1.0 + rotation_matrix.at<double>(2, 2) - rotation_matrix.at<double>(0, 0) - rotation_matrix.at<double>(1, 1)) * 2;
                w = (rotation_matrix.at<double>(1, 0) - rotation_matrix.at<double>(0, 1)) / s;
                x = (rotation_matrix.at<double>(0, 2) + rotation_matrix.at<double>(2, 0)) / s;
                y = (rotation_matrix.at<double>(1, 2) + rotation_matrix.at<double>(2, 1)) / s;
                z = 0.25 * s;
            }
            pose_msg.pose.orientation.w = w;
            pose_msg.pose.orientation.x = x;
            pose_msg.pose.orientation.y = y;
            pose_msg.pose.orientation.z = z;

            saveCalibrationData(pose_msg);
        }
    }

    void moveToNextPose()
    {
        if (current_pose_index_ < saved_poses_.size())
        {
            demo_msgs::msg::PoseGoal pose_goal;
            pose_goal.pose = saved_poses_[current_pose_index_];
            if (pose_goal.pose.header.frame_id.empty())
            {
                pose_goal.pose.header.frame_id = config_.target_frame;
            }
            pose_goal.planner_mode = config_.planner_mode;

            pose_goal_pub_->publish(pose_goal);

            calibration_status_ = CalibrationStatus::WAIT;
            pose_start_time_ = std::chrono::steady_clock::now();
            waiting_at_pose_ = true;
            RCLCPP_INFO(get_logger(), "运动到第 %zu/%zu 个位姿 (自由规划)... frame=%s pos=(%.3f, %.3f, %.3f)",
                        current_pose_index_ + 1, saved_poses_.size(),
                        pose_goal.pose.header.frame_id.c_str(),
                        pose_goal.pose.pose.position.x,
                        pose_goal.pose.pose.position.y,
                        pose_goal.pose.pose.position.z);
            current_pose_index_++;
        }
        else
        {
            // 所有位姿采集完成
            waiting_at_pose_ = false;
            calibration_status_ = CalibrationStatus::IDLE;
            RCLCPP_INFO(get_logger(), "所有位姿采集完成! 按 'r' 执行标定计算");
        }
    }

    // ---------- 采集样本 (预标定) ----------
    void captureSample()
    {
        if (calibration_mode_ != CalibrationMode::PREPARE)
        {
            RCLCPP_WARN(get_logger(), "请在预标定模式下采集样本 (按 'p' 进入)");
            return;
        }

        if (saved_poses_count_ >= config_.max_samples)
        {
            RCLCPP_WARN(get_logger(), "已达到最大样本数 %d", config_.max_samples);
            return;
        }

        saveRobotPose();
    }

    // ---------- 删除最后一个样本 ----------
    void deleteLastSample()
    {
        if (calibration_mode_ != CalibrationMode::PREPARE)
        {
            RCLCPP_WARN(get_logger(), "只能在预标定模式下删除样本");
            return;
        }
        if (saved_poses_.empty())
        {
            RCLCPP_WARN(get_logger(), "没有样本可删除");
            return;
        }
        saved_poses_.pop_back();
        saved_poses_count_--;

        YAML::Node robot_pose_yaml;
        robot_pose_yaml["robot_pose"] = YAML::Node(YAML::NodeType::Sequence);
        for (const auto &pose : saved_poses_)
        {
            YAML::Node new_pose;
            new_pose["frame_id"] = pose.header.frame_id;
            new_pose["position"]["x"] = pose.pose.position.x;
            new_pose["position"]["y"] = pose.pose.position.y;
            new_pose["position"]["z"] = pose.pose.position.z;
            new_pose["orientation"]["x"] = pose.pose.orientation.x;
            new_pose["orientation"]["y"] = pose.pose.orientation.y;
            new_pose["orientation"]["z"] = pose.pose.orientation.z;
            new_pose["orientation"]["w"] = pose.pose.orientation.w;
            robot_pose_yaml["robot_pose"].push_back(new_pose);
        }
        std::string yaml_path = config_dir_ + "/" + config_.robot_pose_file;
        std::ofstream fout(yaml_path);
        if (fout.is_open())
        {
            fout << robot_pose_yaml;
            fout.close();
        }
        RCLCPP_INFO(get_logger(), "已删除最后一个样本,当前样本数: %d", saved_poses_count_);
    }

    // ---------- 重置所有数据 ----------
    void resetAll()
    {
        saved_poses_.clear();
        saved_poses_count_ = 0;
        saved_images_count_ = 0;
        current_pose_index_ = 0;
        calculate_pose_ = false;
        waiting_at_pose_ = false;
        calibration_status_ = CalibrationStatus::IDLE;
        calibration_mode_ = CalibrationMode::PENDING;
        has_result_ = false;
        hand_eye_transform_.release();
        clearRobotPose();
        clearCalibrationData();
        clearRobotTransform();
        clearImage();
        RCLCPP_INFO(get_logger(), "已重置所有样本和数据");
    }

    // ---------- 执行手眼标定 ----------
    void runCalibration()
    {
        if (calibration_mode_ != CalibrationMode::CALIBRATION &&
            calibration_status_ != CalibrationStatus::IDLE)
        {
            RCLCPP_WARN(get_logger(), "请等待正式标定采集完成");
            return;
        }

        RCLCPP_INFO(get_logger(), "=== 开始手眼标定计算 ===");

        clearHandEyeTransformer();
        loadCalibrationData();
        loadCalculateRobotPose();

        if (camera_to_target_.empty() || robot_to_gripper_.empty())
        {
            RCLCPP_ERROR(get_logger(), "标定数据为空,无法执行标定!");
            return;
        }

        if (camera_to_target_.size() != robot_to_gripper_.size())
        {
            RCLCPP_WARN(get_logger(), "相机数据(%zu)与机器人数据(%zu)数量不一致!",
                        camera_to_target_.size(), robot_to_gripper_.size());
        }

        validateCameraIntrinsics();
        performCalculation();
        saveHandEyeTransformer();

        calibration_mode_ = CalibrationMode::PENDING;
    }

    // ---------- 结果计算 ----------
    void loadCalibrationData()
    {
        camera_to_target_.clear();
        std::string yaml_path = config_dir_ + "/" + config_.calibration_data_file;
        YAML::Node calibration_data_yaml;
        try
        {
            calibration_data_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "加载 calibration_data.yaml 失败: %s", e.what());
            return;
        }

        YAML::Node calibration_data_node = calibration_data_yaml["calibration_data"];
        for (const auto &node : calibration_data_node)
        {
            if (!node["position"] || !node["orientation"])
            {
                RCLCPP_ERROR(get_logger(), "Invalid node");
                continue;
            }
            double px = node["position"]["x"].as<double>();
            double py = node["position"]["y"].as<double>();
            double pz = node["position"]["z"].as<double>();
            double qx = node["orientation"]["x"].as<double>();
            double qy = node["orientation"]["y"].as<double>();
            double qz = node["orientation"]["z"].as<double>();
            double qw = node["orientation"]["w"].as<double>();

            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            Eigen::Matrix3d R = q.toRotationMatrix();
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.block<3, 3>(0, 0) = R;
            T(0, 3) = px;
            T(1, 3) = py;
            T(2, 3) = pz;

            cv::Mat T_cv(4, 4, CV_64F);
            for (int i = 0; i < 4; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    T_cv.at<double>(i, j) = T(i, j);
                }
            }
            camera_to_target_.push_back(T_cv);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu camera_to_target transformers", camera_to_target_.size());
    }

    void loadCalculateRobotPose()
    {
        robot_to_gripper_.clear();
        std::string yaml_path = config_dir_ + "/robot_transform.yaml";
        YAML::Node robot_transform_yaml;
        try
        {
            robot_transform_yaml = YAML::LoadFile(yaml_path);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "加载 robot_transform.yaml 失败: %s", e.what());
            return;
        }

        YAML::Node robot_transform_node = robot_transform_yaml["robot_transform"];
        for (const auto &node : robot_transform_node)
        {
            if (!node["translation"] || !node["rotation"]) continue;

            double x = node["translation"]["x"].as<double>();
            double y = node["translation"]["y"].as<double>();
            double z = node["translation"]["z"].as<double>();
            double qx = node["rotation"]["x"].as<double>();
            double qy = node["rotation"]["y"].as<double>();
            double qz = node["rotation"]["z"].as<double>();
            double qw = node["rotation"]["w"].as<double>();

            // 四元数转旋转矩阵
            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            Eigen::Matrix3d R = q.toRotationMatrix();

            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    T.at<double>(i, j) = R(i, j);
                }
            }
            T.at<double>(0, 3) = x;
            T.at<double>(1, 3) = y;
            T.at<double>(2, 3) = z;

            robot_to_gripper_.push_back(T);
        }
        RCLCPP_INFO(get_logger(), "Loaded %zu robot_to_gripper transformers", robot_to_gripper_.size());
    }

    double validateCameraIntrinsics()
    {
        std::vector<std::vector<cv::Point3f>> all_obj_points;
        std::vector<std::vector<cv::Point2f>> all_img_points;
        cv::Size image_size;

        int idx = 0;
        while (true)
        {
            std::string filename = config_dir_ + "/chessboard_" + std::to_string(idx) + ".jpg";
            if (!fs::exists(filename)) break;

            cv::Mat img = cv::imread(filename);
            if (img.empty()) { idx++; continue; }

            cv::Mat gray;
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
            image_size = gray.size();

            std::vector<cv::Point2f> corners;
            bool found = cv::findChessboardCorners(gray, board_size_, corners,
                            cv::CALIB_CB_ADAPTIVE_THRESH |
                            cv::CALIB_CB_NORMALIZE_IMAGE);
            if (found)
            {
                cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.001));
                all_obj_points.push_back(board_points_3d_);
                all_img_points.push_back(corners);
            }
            idx++;
        }

        if (all_obj_points.size() < 5)
        {
            RCLCPP_WARN(get_logger(), "内参验证:有效图片不足5张,当前=%zu", all_obj_points.size());
            return -1.0;
        }

        cv::Mat new_camera_matrix, new_dist_coeffs;
        std::vector<cv::Mat> rvecs, tvecs;
        double reproj_err = cv::calibrateCamera(
            all_obj_points, all_img_points, image_size,
            new_camera_matrix, new_dist_coeffs, rvecs, tvecs);

        double dfx = std::abs(new_camera_matrix.at<double>(0, 0) - camera_matrix_.at<double>(0, 0));
        double dfy = std::abs(new_camera_matrix.at<double>(1, 1) - camera_matrix_.at<double>(1, 1));
        double dcx = std::abs(new_camera_matrix.at<double>(0, 2) - camera_matrix_.at<double>(0, 2));
        double dcy = std::abs(new_camera_matrix.at<double>(1, 2) - camera_matrix_.at<double>(1, 2));

        RCLCPP_INFO(get_logger(), "===== 内参验证结果 =====");
        RCLCPP_INFO(get_logger(), "重投影误差: %.6f px", reproj_err);
        RCLCPP_INFO(get_logger(), "fx: 当前=%.6f  重标定=%.6f  差值=%.6f",
                    camera_matrix_.at<double>(0, 0), new_camera_matrix.at<double>(0, 0), dfx);
        RCLCPP_INFO(get_logger(), "fy: 当前=%.6f  重标定=%.6f  差值=%.6f",
                    camera_matrix_.at<double>(1, 1), new_camera_matrix.at<double>(1, 1), dfy);
        RCLCPP_INFO(get_logger(), "cx: 当前=%.6f  重标定=%.6f  差值=%.6f",
                    camera_matrix_.at<double>(0, 2), new_camera_matrix.at<double>(0, 2), dcx);
        RCLCPP_INFO(get_logger(), "cy: 当前=%.6f  重标定=%.6f  差值=%.6f",
                    camera_matrix_.at<double>(1, 2), new_camera_matrix.at<double>(1, 2), dcy);

        if (reproj_err > 1.0)
            RCLCPP_WARN(get_logger(), "警告:重投影误差过大(>1px),内参质量差");
        else if (reproj_err > 0.5)
            RCLCPP_WARN(get_logger(), "提示:重投影误差偏大(>0.5px),建议重新标定内参");
        else
            RCLCPP_INFO(get_logger(), "重投影误差正常");

        if (dfx > 5.0 || dfy > 5.0 || dcx > 5.0 || dcy > 5.0)
            RCLCPP_WARN(get_logger(), "警告:当前内参与重标定结果差异超过5px");
        else
            RCLCPP_INFO(get_logger(), "当前内参与重标定结果吻合");

        RCLCPP_INFO(get_logger(), "========================");
        return reproj_err;
    }

    void performCalculation()
    {
        std::vector<cv::Mat> R_gripper2base, t_gripper2base;
        std::vector<cv::Mat> R_target2cam, t_target2cam;

        for (const auto &robot_transform : robot_to_gripper_)
        {
            cv::Mat R = robot_transform(cv::Rect(0, 0, 3, 3)).clone();
            cv::Mat t = robot_transform(cv::Rect(3, 0, 1, 3)).clone();
            R_gripper2base.push_back(R);
            t_gripper2base.push_back(t);
        }
        for (const auto &camera_transform : camera_to_target_)
        {
            cv::Mat R = camera_transform(cv::Rect(0, 0, 3, 3)).clone();
            cv::Mat t = camera_transform(cv::Rect(3, 0, 1, 3)).clone();
            R_target2cam.push_back(R);
            t_target2cam.push_back(t);
        }

        cv::Mat R_cam2gripper, t_cam2gripper;
        int method = config_.hand_eye_method;
        if (method < 0 || method > 4) method = 0;

        cv::calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            R_cam2gripper, t_cam2gripper,
            static_cast<cv::HandEyeCalibrationMethod>(method));

        hand_eye_transform_ = cv::Mat::eye(4, 4, CV_64F);
        R_cam2gripper.copyTo(hand_eye_transform_(cv::Rect(0, 0, 3, 3)));
        t_cam2gripper.copyTo(hand_eye_transform_(cv::Rect(3, 0, 1, 3)));
        hand_eye_transform_.at<double>(3, 0) = 0.0;
        hand_eye_transform_.at<double>(3, 1) = 0.0;
        hand_eye_transform_.at<double>(3, 2) = 0.0;
        hand_eye_transform_.at<double>(3, 3) = 1.0;

        has_result_ = true;

        // 打印结果
        RCLCPP_INFO(get_logger(), "========== 手眼标定结果 ==========");
        RCLCPP_INFO(get_logger(), "样本数: %zu", robot_to_gripper_.size());
        RCLCPP_INFO(get_logger(), "标定方法: %d", method);
        for (int i = 0; i < 4; i++)
        {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6);
            for (int j = 0; j < 4; j++)
            {
                oss << hand_eye_transform_.at<double>(i, j);
                if (j < 3) oss << ", ";
            }
            RCLCPP_INFO(get_logger(), "第%d行: %s", i + 1, oss.str().c_str());
        }
        RCLCPP_INFO(get_logger(), "==================================");

        // 验证标定结果: 检查各样本算出的标定板在 base 系下的位姿一致性
        validateHandEyeResult();
    }

    // ---------- 验证手眼标定结果 ----------
    // 原理: 标定板在 base 坐标系下的位姿是固定的
    //   T_base_target = T_gripper2base * T_cam2gripper * T_target2cam
    // 用每组样本计算 T_base_target, 检查它们之间的差异 (平移误差 + 旋转误差)
    // 若标定正确, 各样本的 T_base_target 应高度一致
    void validateHandEyeResult()
    {
        size_t n = robot_to_gripper_.size();
        if (n < 2 || camera_to_target_.size() != n)
        {
            RCLCPP_WARN(get_logger(), "样本不足或数量不匹配,无法验证");
            return;
        }

        // 用每组样本计算标定板在 base 系下的位姿
        std::vector<cv::Mat> T_base_target_list;
        for (size_t i = 0; i < n; ++i)
        {
            // T_base_target = T_gripper2base * T_cam2gripper * T_target2cam
            cv::Mat T = robot_to_gripper_[i] * hand_eye_transform_ * camera_to_target_[i];
            T_base_target_list.push_back(T);
        }

        // 计算各样本 T_base_target 与均值的差异
        // 1. 平移均值
        cv::Mat t_mean = cv::Mat::zeros(3, 1, CV_64F);
        for (const auto &T : T_base_target_list)
        {
            t_mean += T(cv::Rect(3, 0, 1, 3));
        }
        t_mean /= static_cast<double>(n);

        // 2. 旋转均值 (用四元数平均的简化版: 先转四元数,取第一个为参考,其余对齐后平均)
        std::vector<Eigen::Quaterniond> quats;
        for (const auto &T : T_base_target_list)
        {
            Eigen::Matrix3d R_eig;
            for (int r = 0; r < 3; ++r)
                for (int c = 0; c < 3; ++c)
                    R_eig(r, c) = T.at<double>(r, c);
            Eigen::Quaterniond q(R_eig);
            q.normalize();
            // 保证四元数在同一半球
            if (quats.empty() || q.dot(quats[0]) >= 0)
                quats.push_back(q);
            else
                quats.push_back(Eigen::Quaterniond(-q.w(), -q.x(), -q.y(), -q.z()));
        }
        Eigen::Quaterniond q_mean = quats[0];
        for (size_t i = 1; i < quats.size(); ++i)
        {
            // 简化平均: 累加后归一化
            q_mean = Eigen::Quaterniond(
                q_mean.w() + quats[i].w(),
                q_mean.x() + quats[i].x(),
                q_mean.y() + quats[i].y(),
                q_mean.z() + quats[i].z());
            q_mean.normalize();
        }

        // 3. 计算各样本与均值的误差
        double max_t_err = 0.0, avg_t_err = 0.0;
        double max_r_err = 0.0, avg_r_err = 0.0;
        for (size_t i = 0; i < n; ++i)
        {
            // 平移误差 (米)
            cv::Mat t_diff = T_base_target_list[i](cv::Rect(3, 0, 1, 3)) - t_mean;
            double t_err = cv::norm(t_diff);
            avg_t_err += t_err;
            max_t_err = std::max(max_t_err, t_err);

            // 旋转误差 (度): 四元数夹角
            Eigen::Quaterniond q_i = quats[i];
            double dot = std::abs(q_i.dot(q_mean));
            dot = std::min(1.0, std::max(-1.0, dot));
            double r_err = 2.0 * std::acos(dot) * 180.0 / CV_PI;
            avg_r_err += r_err;
            max_r_err = std::max(max_r_err, r_err);
        }
        avg_t_err /= static_cast<double>(n);
        avg_r_err /= static_cast<double>(n);

        RCLCPP_INFO(get_logger(), "========== 手眼标定验证 ==========");
        RCLCPP_INFO(get_logger(), "标定板在 base 系下位姿一致性 (各样本与均值差异):");
        RCLCPP_INFO(get_logger(), "  平移误差: 平均 %.4f mm, 最大 %.4f mm",
                    avg_t_err * 1000.0, max_t_err * 1000.0);
        RCLCPP_INFO(get_logger(), "  旋转误差: 平均 %.4f deg, 最大 %.4f deg",
                    avg_r_err, max_r_err);
        RCLCPP_INFO(get_logger(), "标定板在 base 系下位置(均值): (%.4f, %.4f, %.4f) m",
                    t_mean.at<double>(0), t_mean.at<double>(1), t_mean.at<double>(2));

        // 误差评估
        if (max_t_err < 0.005 && max_r_err < 1.0)
        {
            RCLCPP_INFO(get_logger(), "✓ 标定质量优秀 (平移<5mm, 旋转<1°)");
        }
        else if (max_t_err < 0.015 && max_r_err < 3.0)
        {
            RCLCPP_INFO(get_logger(), "○ 标定质量良好 (平移<15mm, 旋转<3°)");
        }
        else if (max_t_err < 0.030 && max_r_err < 5.0)
        {
            RCLCPP_WARN(get_logger(), "△ 标定质量一般 (平移<30mm, 旋转<5°),建议增加样本或检查数据");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "✗ 标定质量差 (平移>30mm 或 旋转>5°),建议重新标定!");
            RCLCPP_ERROR(get_logger(), "  可能原因: 1)样本位姿变化不足 2)棋盘格检测不准 3)TF 时间不同步 4)内参误差大");
        }
        RCLCPP_INFO(get_logger(), "==================================");
    }

    void saveHandEyeTransformer()
    {
        if (!has_result_)
        {
            RCLCPP_WARN(get_logger(), "尚未完成标定,无法保存结果!");
            return;
        }

        fs::path result_path = fs::path(config_dir_) / config_.result_file;
        fs::create_directories(result_path.parent_path());

        cv::Mat R = hand_eye_transform_(cv::Rect(0, 0, 3, 3));
        cv::Mat t = hand_eye_transform_(cv::Rect(3, 0, 1, 3));

        // 旋转向量
        cv::Mat rvec;
        cv::Rodrigues(R, rvec);

        // 四元数
        double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
        double qx, qy, qz, qw;
        if (trace > 0.0)
        {
            double s = 0.5 / std::sqrt(trace + 1.0);
            qw = 0.25 / s;
            qx = (R.at<double>(2, 1) - R.at<double>(1, 2)) * s;
            qy = (R.at<double>(0, 2) - R.at<double>(2, 0)) * s;
            qz = (R.at<double>(1, 0) - R.at<double>(0, 1)) * s;
        }
        else
        {
            if (R.at<double>(0, 0) > R.at<double>(1, 1) && R.at<double>(0, 0) > R.at<double>(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + R.at<double>(0, 0) - R.at<double>(1, 1) - R.at<double>(2, 2));
                qw = (R.at<double>(2, 1) - R.at<double>(1, 2)) / s;
                qx = 0.25 * s;
                qy = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
                qz = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
            }
            else if (R.at<double>(1, 1) > R.at<double>(2, 2))
            {
                double s = 2.0 * std::sqrt(1.0 + R.at<double>(1, 1) - R.at<double>(0, 0) - R.at<double>(2, 2));
                qw = (R.at<double>(0, 2) - R.at<double>(2, 0)) / s;
                qx = (R.at<double>(0, 1) + R.at<double>(1, 0)) / s;
                qy = 0.25 * s;
                qz = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
            }
            else
            {
                double s = 2.0 * std::sqrt(1.0 + R.at<double>(2, 2) - R.at<double>(0, 0) - R.at<double>(1, 1));
                qw = (R.at<double>(1, 0) - R.at<double>(0, 1)) / s;
                qx = (R.at<double>(0, 2) + R.at<double>(2, 0)) / s;
                qy = (R.at<double>(1, 2) + R.at<double>(2, 1)) / s;
                qz = 0.25 * s;
            }
        }

        // 欧拉角 (RPY, ZYX 顺序)
        double roll, pitch, yaw;
        pitch = std::asin(-std::max(-1.0, std::min(1.0, R.at<double>(2, 0))));
        if (std::abs(R.at<double>(2, 0)) < 0.9999)
        {
            roll = std::atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            yaw = std::atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        }
        else
        {
            roll = std::atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
            yaw = 0.0;
        }

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "hand_eye_calibration" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "type" << YAML::Value << "eye_in_hand";
        out << YAML::Key << "base_frame" << YAML::Value << config_.base_frame;
        out << YAML::Key << "ee_frame" << YAML::Value << config_.ee_frame;
        out << YAML::Key << "camera_frame" << YAML::Value << config_.camera_frame;
        out << YAML::Key << "num_samples" << YAML::Value << static_cast<int>(robot_to_gripper_.size());
        out << YAML::Key << "method" << YAML::Value << config_.hand_eye_method;

        out << YAML::Key << "transformation" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "translation" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << t.at<double>(0);
        out << YAML::Key << "y" << YAML::Value << t.at<double>(1);
        out << YAML::Key << "z" << YAML::Value << t.at<double>(2);
        out << YAML::EndMap;

        out << YAML::Key << "rotation_matrix" << YAML::Value << YAML::BeginSeq;
        for (int i = 0; i < 3; ++i)
        {
            out << YAML::Flow << std::vector<double>{
                R.at<double>(i, 0), R.at<double>(i, 1), R.at<double>(i, 2)};
        }
        out << YAML::EndSeq;

        out << YAML::Key << "quaternion" << YAML::Value << YAML::BeginMap;
        out << YAML::Key << "x" << YAML::Value << qx;
        out << YAML::Key << "y" << YAML::Value << qy;
        out << YAML::Key << "z" << YAML::Value << qz;
        out << YAML::Key << "w" << YAML::Value << qw;
        out << YAML::EndMap;

        out << YAML::Key << "rpy_rad" << YAML::Value << YAML::Flow
            << std::vector<double>{roll, pitch, yaw};
        out << YAML::Key << "rpy_deg" << YAML::Value << YAML::Flow
            << std::vector<double>{roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI};
        out << YAML::Key << "rodrigues" << YAML::Value << YAML::Flow
            << std::vector<double>{rvec.at<double>(0), rvec.at<double>(1), rvec.at<double>(2)};
        out << YAML::EndMap;  // transformation

        out << YAML::EndMap;  // hand_eye_calibration

        std::ofstream fout(result_path);
        fout << out.c_str() << std::endl;
        fout.close();

        RCLCPP_INFO(get_logger(), "标定结果已保存到: %s", result_path.string().c_str());
    }

    void clearHandEyeTransformer()
    {
        std::string yaml_path = config_dir_ + "/" + config_.result_file;
        if (fs::exists(yaml_path))
        {
            fs::remove(yaml_path);
        }
    }

    // ---------- 回调函数 ----------
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_ || !config_.use_custom_camera_matrix)
        {
            if (config_.use_custom_camera_matrix)
            {
                camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                    config_.fx, 0, config_.cx,
                    0, config_.fy, config_.cy,
                    0, 0, 1);
            }
            else
            {
                camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
                camera_matrix_.at<double>(0, 0) = msg->k[0];
                camera_matrix_.at<double>(1, 1) = msg->k[4];
                camera_matrix_.at<double>(0, 2) = msg->k[2];
                camera_matrix_.at<double>(1, 2) = msg->k[5];
            }

            dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
            for (size_t i = 0; i < std::min(msg->d.size(), size_t(5)); i++)
            {
                dist_coeffs_.at<double>(0, i) = msg->d[i];
            }

            RCLCPP_INFO(get_logger(), "相机内参获取完成");
            camera_info_received_ = true;
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv::Mat image = imageFromMessage(msg);
        if (image.empty()) return;

        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            rgb_image_ = image;
        }
        frame_cv_.notify_one();
    }

    cv::Mat imageFromMessage(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (msg->encoding == "rgb8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC3,
                          const_cast<uint8_t *>(msg->data.data()));
            cv::Mat converted;
            cv::cvtColor(image, converted, cv::COLOR_RGB2BGR);
            return converted;
        }
        if (msg->encoding == "bgr8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC3,
                          const_cast<uint8_t *>(msg->data.data()));
            return image.clone();
        }
        if (msg->encoding == "mono8")
        {
            cv::Mat image(msg->height, msg->width, CV_8UC1,
                          const_cast<uint8_t *>(msg->data.data()));
            return image.clone();
        }
        RCLCPP_WARN(get_logger(), "Unsupported image encoding: %s", msg->encoding.c_str());
        return cv::Mat();
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        current_joint_state_ = msg;
    }

    void endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(end_pose_mutex_);
        current_end_pose_ = msg;
    }

    void armStatusCallback(const piper_msgs::msg::PiperStatusMsg::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(arm_status_mutex_);
        current_arm_status_ = msg;
    }

    // ---------- 实时检测并绘制棋盘格角点 ----------
    void detectAndDrawChessboard(cv::Mat &image)
    {
        if (!config_.show_corners || image.empty())
        {
            return;
        }

        // 转灰度图
        cv::Mat gray;
        if (image.channels() == 3)
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        else
            gray = image;

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, board_size_, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (found)
        {
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            last_corners_ = corners;
            has_last_detection_ = true;

            // 绘制角点
            cv::drawChessboardCorners(image, board_size_, corners, true);

            // 绘制坐标系
            if (calculate_pose_)
            {
                drawCoordinateSystem(image, corners);
            }
        }
        else
        {
            has_last_detection_ = false;
        }
    }

    void drawCoordinateSystem(cv::Mat &image, const std::vector<cv::Point2f> &corners)
    {
        if (corners.size() < 4) return;
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(board_points_3d_, corners, camera_matrix_,
                                    dist_coeffs_, rvec, tvec);
        if (success)
        {
            std::vector<cv::Point3f> axis_points;
            axis_points.push_back(cv::Point3f(0, 0, 0));
            axis_points.push_back(cv::Point3f(static_cast<float>(3 * config_.square_size), 0, 0));
            axis_points.push_back(cv::Point3f(0, static_cast<float>(3 * config_.square_size), 0));
            axis_points.push_back(cv::Point3f(0, 0, static_cast<float>(-3 * config_.square_size)));

            std::vector<cv::Point2f> projected_points;
            cv::projectPoints(axis_points, rvec, tvec, camera_matrix_, dist_coeffs_, projected_points);

            if (projected_points.size() == 4)
            {
                cv::Point2f origin = projected_points[0];
                cv::Point2f x_axis = projected_points[1];
                cv::Point2f y_axis = projected_points[2];
                cv::Point2f z_axis = projected_points[3];
                cv::arrowedLine(image, origin, x_axis, cv::Scalar(0, 0, 255), 3);
                cv::putText(image, "X", x_axis, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0, 255), 2);
                cv::arrowedLine(image, origin, y_axis, cv::Scalar(0, 255, 0), 3);
                cv::putText(image, "Y", y_axis, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                cv::arrowedLine(image, origin, z_axis, cv::Scalar(255, 0, 0), 3);
                cv::putText(image, "Z", z_axis, cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 0), 2);
            }
        }
    }

    // ---------- 在图像上绘制状态信息 ----------
    void drawOverlay(cv::Mat &image)
    {
        int line_height = 25;
        int y = 30;
        int x = 10;

        // 半透明背景
        cv::Rect roi(0, 0, 520, 320);
        cv::Mat overlay = image(roi).clone();
        cv::rectangle(overlay, cv::Point(0, 0), cv::Point(520, 320),
                      cv::Scalar(0, 0, 0), -1);
        cv::addWeighted(overlay, 0.5, image(roi), 0.5, 0, image(roi));

        // 标题
        cv::putText(image, "Hand-Eye Calibration (Eye-in-Hand)",
                    cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(0, 255, 255), 2);
        y += line_height;

        // 模式显示
        std::string mode_str;
        cv::Scalar mode_color;
        switch (calibration_mode_)
        {
        case CalibrationMode::PENDING:
            mode_str = "MODE: PENDING";
            mode_color = cv::Scalar(128, 128, 128);
            break;
        case CalibrationMode::PREPARE:
            mode_str = "MODE: PREPARE";
            mode_color = cv::Scalar(0, 255, 255);
            break;
        case CalibrationMode::CALIBRATION:
            mode_str = "MODE: CALIBRATION";
            mode_color = cv::Scalar(0, 255, 0);
            break;
        }
        cv::putText(image, mode_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, mode_color, 1);
        y += line_height;

        // 样本数
        std::string sample_str = "Samples: " + std::to_string(saved_poses_count_) +
                                 " / " + std::to_string(config_.max_samples) +
                                 " (min " + std::to_string(config_.min_samples) + ")";
        cv::Scalar sample_color = (saved_poses_count_ >= config_.min_samples)
                                      ? cv::Scalar(0, 255, 0)
                                      : cv::Scalar(0, 200, 255);
        cv::putText(image, sample_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, sample_color, 1);
        y += line_height;

        // 相机内参状态
        std::string info_str = "Camera Info: " + std::string(camera_info_received_ ? "OK" : "Waiting...");
        cv::Scalar info_color = camera_info_received_ ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
        cv::putText(image, info_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, info_color, 1);
        y += line_height;

        // 关节状态接收状态
        bool has_joint = false;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            has_joint = (current_joint_state_ != nullptr);
        }
        std::string joint_str = "Joint State: " + std::string(has_joint ? "OK" : "Waiting...");
        cv::Scalar joint_color = has_joint ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255);
        cv::putText(image, joint_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, joint_color, 1);
        y += line_height;

        // 正式标定进度
        if (calibration_mode_ == CalibrationMode::CALIBRATION)
        {
            std::string pos_info = "Position: " + std::to_string(current_pose_index_) +
                                   "/" + std::to_string(saved_poses_.size()) +
                                   "  Images: " + std::to_string(saved_images_count_);
            cv::putText(image, pos_info, cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
            y += line_height;

            if (waiting_at_pose_)
            {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                    current_time - pose_start_time_).count();
                std::string wait_str = "WAITING: " + std::to_string(elapsed) +
                                       "/" + std::to_string(static_cast<int>(config_.pose_wait_time)) + "s";
                cv::putText(image, wait_str, cv::Point(x, y),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1);
                y += line_height;
            }
        }

        // 标定结果状态
        std::string result_str = "Calibration: " + std::string(has_result_ ? "DONE" : "Not done");
        cv::Scalar result_color = has_result_ ? cv::Scalar(0, 255, 0) : cv::Scalar(128, 128, 128);
        cv::putText(image, result_str, cv::Point(x, y),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, result_color, 1);
        y += line_height;

        // 如果有结果,显示结果
        if (has_result_)
        {
            std::stringstream ss;
            ss << std::fixed << std::setprecision(4);
            ss << "t = [" << hand_eye_transform_.at<double>(0, 3) << ", "
               << hand_eye_transform_.at<double>(1, 3) << ", "
               << hand_eye_transform_.at<double>(2, 3) << "]";
            cv::putText(image, ss.str(), cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 255, 0), 1);
            y += line_height - 5;
        }

        // 操作提示
        y += 5;
        cv::putText(image, "p: Prepare  SPACE: Save  e: End Prepare",
                    cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(255, 255, 255), 1);
        y += 18;
        cv::putText(image, "c: Calibrate  r: Run  d: Delete  s: Save",
                    cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(255, 255, 255), 1);
        y += 18;
        cv::putText(image, "x: Reset  q/ESC: Quit",
                    cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                    cv::Scalar(255, 255, 255), 1);
    }

    // ---------- 成员变量 ----------
    CalibrationConfig config_;
    std::string config_dir_;

    // 图像处理
    std::mutex frame_mutex_;
    std::condition_variable frame_cv_;
    std::atomic<bool> running_{true};
    cv::Mat rgb_image_;
    cv::Mat raw_frame_;

    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_ = false;

    // 棋盘格参数
    cv::Size board_size_;
    std::vector<cv::Point3f> board_points_3d_;

    // 标定状态
    CalibrationMode calibration_mode_ = CalibrationMode::PENDING;
    CalibrationStatus calibration_status_ = CalibrationStatus::IDLE;

    bool waiting_at_pose_ = false;
    std::chrono::steady_clock::time_point pose_start_time_;

    // 机器人状态
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    mutable std::mutex joint_state_mutex_;
    piper_msgs::msg::PiperStatusMsg::SharedPtr current_arm_status_;
    mutable std::mutex arm_status_mutex_;
    geometry_msgs::msg::PoseStamped::SharedPtr current_end_pose_;
    mutable std::mutex end_pose_mutex_;

    // 保存的数据
    std::vector<geometry_msgs::msg::PoseStamped> saved_poses_;
    int saved_poses_count_ = 0;
    int saved_images_count_ = 0;
    size_t current_pose_index_ = 0;
    bool calculate_pose_ = false;

    // 棋盘格检测缓存
    std::vector<cv::Point2f> last_corners_;
    bool has_last_detection_ = false;

    // 结果计算相关
    std::vector<cv::Mat> robot_to_gripper_;
    std::vector<cv::Mat> camera_to_target_;
    cv::Mat hand_eye_transform_;
    bool has_result_ = false;

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<piper_msgs::msg::PiperStatusMsg>::SharedPtr arm_status_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pose_sub_;
    rclcpp::Publisher<demo_msgs::msg::PoseGoal>::SharedPtr pose_goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_flag_pub_;
    rclcpp::Client<piper_msgs::srv::Enable>::SharedPtr enable_srv_client_;
};

// ============================================================
// main
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HandEyeCalibrationNode>();

    // ROS spin 在独立线程执行,保证图像回调能及时触发条件变量
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    // 主循环:由图像回调通过条件变量唤醒
    while (rclcpp::ok())
    {
        if (!node->spinAndUpdate())
        {
            break;
        }
    }

    node->requestShutdown();
    rclcpp::shutdown();
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }

    cv::destroyAllWindows();
    return 0;
}
