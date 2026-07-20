// trajectory_draw_node.cpp
// 轨迹绘制 + 重投影 + 机械臂执行节点
//
// 工作流程:
//   1. 启动后弹出全屏白色画布窗口,用户用鼠标在画布上绘制轨迹
//      - 鼠标左键按下并拖动绘制连续轨迹
//      - 若鼠标松开(轨迹断开),自动清除画布上前一段轨迹,继续画下一段
//      - 按回车(ENTER)确认当前轨迹
//   2. 确认后,节点订阅深度相机 RGB + 深度图像
//      - 通过颜色梯度分割在 RGB 画面中找到屏幕区域(符合面积约束的最大四边形)
//      - 计算屏幕四边形 -> 画布区域的单应矩阵 H,求逆得到画布->图像的重投影矩阵
//      - 将画布上的轨迹离散点重投影到 RGB 图像坐标
//      - 结合深度信息将 2D 像素转换为相机坐标系下的 3D 点
//      - 通过手眼标定结果 (camera -> link6) + TF (base_link -> link6)
//        将相机坐标系 3D 点转换到 base_link 坐标系
//   3. 将 3D 轨迹离散点打包为 demo_msgs/TrackCmd 发布到 /track_cmd
//      由 piper_control_node 接收并控制机械臂顺序执行绘制
//   4. 机械臂绘制前清除屏幕上原有轨迹(通过发布空 TrackCmd 或由本节点擦除屏幕区域)
//
// 颜色梯度分割可视化调参:
//   通过参数 enable_segmentation_debug (默认 false) 控制是否显示分割中间结果
//   启用后会弹出调试窗口,展示梯度图、二值化、轮廓、拟合四边形等中间结果
//   便于调整 canny_threshold, morph_kernel, area_min_ratio, area_max_ratio 等参数
//
// 键盘操作 (画布窗口):
//   鼠标左键 - 按下并拖动绘制轨迹
//   ENTER    - 确认当前轨迹,开始重投影与发布
//   c        - 清除画布
//   q/ESC    - 退出节点

#include <algorithm>
#include <atomic>
#include <array>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
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
#include "demo_msgs/msg/track_cmd.hpp"
#include "demo_msgs/srv/get3_d_points.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "piper_msgs/msg/piper_status_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace fs = std::filesystem;

// ============================================================
// 配置参数
// ============================================================
struct TrajectoryDrawConfig
{
    // 话题
    std::string rgb_topic = "/camera/rgb/image_raw";
    std::string depth_topic = "/camera/depth/image_raw";
    std::string camera_info_topic = "/camera/rgb/camera_info";
    std::string arm_status_topic = "/arm_status";
    std::string track_cmd_topic = "/track_cmd";
    std::string target_pose_topic = "/target_pose";
    // 3D 点反投影服务 (由 d435i_data_publisher_node 提供, 使用 RS_D435i::get3DPoint)
    std::string get_3d_points_service = "/camera/get_3d_points";

    // TF
    std::string base_frame = "base_link";
    std::string ee_frame = "link6";
    std::string camera_frame = "camera_color_optical_frame";

    // 画布
    int canvas_width = 1280;
    int canvas_height = 720;
    std::string canvas_window = "Trajectory Canvas (ENTER to confirm)";
    int brush_thickness = 3;

    // 轨迹离散化
    double sample_step_px = 8.0;       // 画布上相邻离散点的像素间隔
    double z_offset = 0.0;             // 绘制高度偏移(相对屏幕表面,正值抬离屏幕)
    double z_draw_height = 0.005;      // 绘制时距离屏幕表面的高度(m)

    // 工具中心点(TCP):机械臂末端法兰向外的工具长度(m)
    // 轨迹点会沿屏幕法线方向偏移 tool_offset,使工具尖端点触屏幕表面
    double tool_offset = 0.065;        // 默认 65mm
    // 是否使用屏幕平面拟合姿态(true: 由预选点深度拟合平面计算姿态;
    //                            false: 使用固定 pitch=pi 朝下姿态)
    bool use_screen_plane_pose = true;
    // 是否使用与 home 一致的姿态(相机系 RPY=(0,π/2,0),末端 Z 轴朝前)
    // true: 直接用 home 姿态,避免屏幕平面拟合的 RPY 多值性问题,确保 OMPL 可求解
    // false: 使用 use_screen_plane_pose 的结果(计算出的屏幕平面拟合姿态)
    bool use_home_orientation = false;
    // 是否将路径点投影到拟合的屏幕平面上(消除深度噪声,确保所有路径点共面)
    // true:  用预选点拟合的屏幕平面(法向量+中心),将每个路径点沿法线方向投影到平面上
    // false: 使用原始深度反投影 3D 点(各点深度有噪声,不在严格同一平面)
    bool project_points_to_plane = true;

    // 颜色梯度分割参数
    bool enable_segmentation_debug = true;  // 分割可视化调参开关
    // 确认轨迹时是否显示 "Reprojected Trajectory on RGB" 和 "Screen Warped to Canvas" 窗口
    bool show_reproject_windows = false;
    int canny_low = 50;
    int canny_high = 150;
    int morph_kernel = 5;
    double area_min_ratio = 0.05;      // 屏幕区域面积下限(占图像比例)
    double area_max_ratio = 0.95;      // 屏幕区域面积上限(占图像比例)
    double quad_approx_eps = 0.02;     // 多边形逼近精度(周长比例)

    // 深度处理
    int depth_median_kernel = 5;       // 深度图中值滤波核大小
    int depth_invalid_radius = 3;      // 深度无效时邻域平均半径

    // 手眼标定结果文件
    std::string hand_eye_result_file = "hand_eye_result.yaml";

    // 相机内参(若 use_custom_camera_matrix=true 则使用,否则从 camera_info 读取)
    bool use_custom_camera_matrix = true;
    double fx = 904.828258;
    double fy = 903.437855;
    double cx = 642.233283;
    double cy = 353.013553;
};

// ============================================================
// 轨迹绘制节点
// ============================================================
class TrajectoryDrawNode : public rclcpp::Node
{
public:
    TrajectoryDrawNode() : Node("trajectory_draw_node")
    {
        loadConfig();
        loadHandEyeResult();

        // ROS 接口
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            config_.rgb_topic, rclcpp::SensorDataQoS(),
            std::bind(&TrajectoryDrawNode::imageCallback, this, std::placeholders::_1));
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            config_.depth_topic, rclcpp::SensorDataQoS(),
            std::bind(&TrajectoryDrawNode::depthCallback, this, std::placeholders::_1));
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            config_.camera_info_topic, 10,
            std::bind(&TrajectoryDrawNode::cameraInfoCallback, this, std::placeholders::_1));
        arm_status_sub_ = create_subscription<piper_msgs::msg::PiperStatusMsg>(
            config_.arm_status_topic, 10,
            std::bind(&TrajectoryDrawNode::armStatusCallback, this, std::placeholders::_1));

        track_cmd_pub_ = create_publisher<demo_msgs::msg::TrackCmd>(
            config_.track_cmd_topic, 10);
        target_pose_pub_ = create_publisher<demo_msgs::msg::PoseGoal>(
            config_.target_pose_topic, 10);

        // 3D 点反投影服务客户端 (调用 d435i_data_publisher_node 的 RS_D435i::get3DPoint)
        get_3d_points_client_ = create_client<demo_msgs::srv::Get3DPoints>(
            config_.get_3d_points_service);

        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 创建画布窗口并进入全屏
        cv::namedWindow(config_.canvas_window, cv::WINDOW_NORMAL);
        cv::setWindowProperty(config_.canvas_window, cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);

        // 通过 X11 直接查询屏幕分辨率(比 getWindowImageRect 更可靠,
        // 后者在全屏切换完成前会返回旧的小窗口尺寸,导致画布只占左上角)
        queryScreenResolution();
        // 用实际屏幕尺寸覆盖配置中的画布尺寸,使画布 Mat 与屏幕 1:1 对应
        config_.canvas_width = screen_width_;
        config_.canvas_height = screen_height_;
        RCLCPP_INFO(get_logger(), "画布尺寸(=屏幕分辨率): %dx%d", config_.canvas_width, config_.canvas_height);

        // 创建与屏幕等大的画布
        canvas_ = cv::Mat(config_.canvas_height, config_.canvas_width, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::setMouseCallback(config_.canvas_window, TrajectoryDrawNode::mouseCallback, this);

        // 调参模式:创建滑动条窗口
        if (config_.enable_segmentation_debug)
        {
            initDebugTrackbars();
        }

        RCLCPP_INFO(get_logger(), "=== 轨迹绘制节点已启动 ===");
        RCLCPP_INFO(get_logger(), "画布: %dx%d, 在画布上用鼠标绘制轨迹", config_.canvas_width, config_.canvas_height);
        RCLCPP_INFO(get_logger(), "  鼠标左键 - 按下并拖动绘制");
        RCLCPP_INFO(get_logger(), "  ENTER  - 确认轨迹并重投影发布");
        RCLCPP_INFO(get_logger(), "  c      - 清除画布");
        RCLCPP_INFO(get_logger(), "  q/ESC  - 退出");
        RCLCPP_INFO(get_logger(), "颜色梯度分割调参: enable_segmentation_debug=%s",
                    config_.enable_segmentation_debug ? "ON" : "OFF");
    }

    ~TrajectoryDrawNode() override
    {
        cv::destroyAllWindows();
    }

    // 主循环:处理画布显示与键盘输入
    bool spinAndUpdate()
    {
        cv::Mat display = canvas_.clone();
        // 绘制提示文字
        cv::putText(display, "Draw trajectory with mouse, ENTER to confirm, c to clear, q to quit",
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(80, 80, 80), 1);
        // 画布尺寸已等于屏幕分辨率,直接显示即可
        cv::imshow(config_.canvas_window, display);

        // 调参模式:实时运行分割并显示结果
        if (config_.enable_segmentation_debug)
        {
            runSegmentationDebug();
        }

        int key = cv::waitKey(30) & 0xFF;
        if (key == 'q' || key == 27)  // q or ESC
        {
            RCLCPP_INFO(get_logger(), "退出轨迹绘制节点。");
            return false;
        }
        else if (key == 13 || key == '\n')  // ENTER
        {
            confirmTrajectory();
        }
        else if (key == 'c')  // 清除画布
        {
            clearCanvas();
        }
        return true;
    }

    void requestShutdown()
    {
        running_.store(false);
    }

private:
    // ---------- 加载配置 ----------
    void loadConfig()
    {
        // 声明参数
        declare_parameter<std::string>("config_file", "");
        declare_parameter<std::string>("rgb_topic", config_.rgb_topic);
        declare_parameter<std::string>("depth_topic", config_.depth_topic);
        declare_parameter<std::string>("camera_info_topic", config_.camera_info_topic);
        declare_parameter<std::string>("arm_status_topic", config_.arm_status_topic);
        declare_parameter<std::string>("track_cmd_topic", config_.track_cmd_topic);
        declare_parameter<std::string>("target_pose_topic", config_.target_pose_topic);
        declare_parameter<std::string>("get_3d_points_service", config_.get_3d_points_service);
        declare_parameter<std::string>("base_frame", config_.base_frame);
        declare_parameter<std::string>("ee_frame", config_.ee_frame);
        declare_parameter<std::string>("camera_frame", config_.camera_frame);
        declare_parameter<int>("canvas_width", config_.canvas_width);
        declare_parameter<int>("canvas_height", config_.canvas_height);
        declare_parameter<std::string>("canvas_window", config_.canvas_window);
        declare_parameter<int>("brush_thickness", config_.brush_thickness);
        declare_parameter<double>("sample_step_px", config_.sample_step_px);
        declare_parameter<double>("z_offset", config_.z_offset);
        declare_parameter<double>("z_draw_height", config_.z_draw_height);
        declare_parameter<double>("tool_offset", config_.tool_offset);
        declare_parameter<bool>("use_screen_plane_pose", config_.use_screen_plane_pose);
        declare_parameter<bool>("use_home_orientation", config_.use_home_orientation);
        declare_parameter<bool>("project_points_to_plane", config_.project_points_to_plane);
        declare_parameter<bool>("enable_segmentation_debug", config_.enable_segmentation_debug);
        declare_parameter<bool>("show_reproject_windows", config_.show_reproject_windows);
        declare_parameter<int>("canny_low", config_.canny_low);
        declare_parameter<int>("canny_high", config_.canny_high);
        declare_parameter<int>("morph_kernel", config_.morph_kernel);
        declare_parameter<double>("area_min_ratio", config_.area_min_ratio);
        declare_parameter<double>("area_max_ratio", config_.area_max_ratio);
        declare_parameter<double>("quad_approx_eps", config_.quad_approx_eps);
        declare_parameter<int>("depth_median_kernel", config_.depth_median_kernel);
        declare_parameter<int>("depth_invalid_radius", config_.depth_invalid_radius);
        declare_parameter<std::string>("hand_eye_result_file", config_.hand_eye_result_file);
        declare_parameter<bool>("use_custom_camera_matrix", config_.use_custom_camera_matrix);
        declare_parameter<double>("fx", config_.fx);
        declare_parameter<double>("fy", config_.fy);
        declare_parameter<double>("cx", config_.cx);
        declare_parameter<double>("cy", config_.cy);

        // 尝试从 YAML 配置文件加载
        std::string config_file = get_parameter("config_file").as_string();
        if (!config_file.empty() && fs::exists(config_file))
        {
            RCLCPP_INFO(get_logger(), "加载配置文件: %s", config_file.c_str());
            YAML::Node yaml = YAML::LoadFile(config_file);
            config_dir_ = fs::path(config_file).parent_path().string();

            if (yaml["rgb_topic"]) config_.rgb_topic = yaml["rgb_topic"].as<std::string>();
            if (yaml["depth_topic"]) config_.depth_topic = yaml["depth_topic"].as<std::string>();
            if (yaml["camera_info_topic"]) config_.camera_info_topic = yaml["camera_info_topic"].as<std::string>();
            if (yaml["arm_status_topic"]) config_.arm_status_topic = yaml["arm_status_topic"].as<std::string>();
            if (yaml["track_cmd_topic"]) config_.track_cmd_topic = yaml["track_cmd_topic"].as<std::string>();
            if (yaml["target_pose_topic"]) config_.target_pose_topic = yaml["target_pose_topic"].as<std::string>();
            if (yaml["get_3d_points_service"]) config_.get_3d_points_service = yaml["get_3d_points_service"].as<std::string>();
            if (yaml["base_frame"]) config_.base_frame = yaml["base_frame"].as<std::string>();
            if (yaml["ee_frame"]) config_.ee_frame = yaml["ee_frame"].as<std::string>();
            if (yaml["camera_frame"]) config_.camera_frame = yaml["camera_frame"].as<std::string>();
            if (yaml["canvas_width"]) config_.canvas_width = yaml["canvas_width"].as<int>();
            if (yaml["canvas_height"]) config_.canvas_height = yaml["canvas_height"].as<int>();
            if (yaml["canvas_window"]) config_.canvas_window = yaml["canvas_window"].as<std::string>();
            if (yaml["brush_thickness"]) config_.brush_thickness = yaml["brush_thickness"].as<int>();
            if (yaml["sample_step_px"]) config_.sample_step_px = yaml["sample_step_px"].as<double>();
            if (yaml["z_offset"]) config_.z_offset = yaml["z_offset"].as<double>();
            if (yaml["z_draw_height"]) config_.z_draw_height = yaml["z_draw_height"].as<double>();
            if (yaml["tool_offset"]) config_.tool_offset = yaml["tool_offset"].as<double>();
            if (yaml["use_screen_plane_pose"]) config_.use_screen_plane_pose = yaml["use_screen_plane_pose"].as<bool>();
            if (yaml["use_home_orientation"]) config_.use_home_orientation = yaml["use_home_orientation"].as<bool>();
            if (yaml["project_points_to_plane"]) config_.project_points_to_plane = yaml["project_points_to_plane"].as<bool>();
            if (yaml["enable_segmentation_debug"]) config_.enable_segmentation_debug = yaml["enable_segmentation_debug"].as<bool>();
            if (yaml["show_reproject_windows"]) config_.show_reproject_windows = yaml["show_reproject_windows"].as<bool>();
            if (yaml["canny_low"]) config_.canny_low = yaml["canny_low"].as<int>();
            if (yaml["canny_high"]) config_.canny_high = yaml["canny_high"].as<int>();
            if (yaml["morph_kernel"]) config_.morph_kernel = yaml["morph_kernel"].as<int>();
            if (yaml["area_min_ratio"]) config_.area_min_ratio = yaml["area_min_ratio"].as<double>();
            if (yaml["area_max_ratio"]) config_.area_max_ratio = yaml["area_max_ratio"].as<double>();
            if (yaml["quad_approx_eps"]) config_.quad_approx_eps = yaml["quad_approx_eps"].as<double>();
            if (yaml["depth_median_kernel"]) config_.depth_median_kernel = yaml["depth_median_kernel"].as<int>();
            if (yaml["depth_invalid_radius"]) config_.depth_invalid_radius = yaml["depth_invalid_radius"].as<int>();
            if (yaml["hand_eye_result_file"]) config_.hand_eye_result_file = yaml["hand_eye_result_file"].as<std::string>();
            if (yaml["use_custom_camera_matrix"]) config_.use_custom_camera_matrix = yaml["use_custom_camera_matrix"].as<bool>();
            if (yaml["fx"]) config_.fx = yaml["fx"].as<double>();
            if (yaml["fy"]) config_.fy = yaml["fy"].as<double>();
            if (yaml["cx"]) config_.cx = yaml["cx"].as<double>();
            if (yaml["cy"]) config_.cy = yaml["cy"].as<double>();

            // 将 YAML 加载的值同步到 ROS2 参数,使后续 get_parameter 能取到 YAML 值
            // (否则 get_parameter 返回 declare_parameter 的默认值/launch 传入值,会覆盖 YAML)
            // 注意: 仅当 launch 未通过命令行显式覆盖该参数时才同步 YAML 值,
            //       这样命令行参数仍能优先于 YAML。
            // 判断方式: 用 is_declared + 已设置标志不可靠,这里采用"参数值与 declare 默认值相同"
            //          视为未被 launch 覆盖的启发式不可靠,因此采用更直接的方式:
            //          YAML 始终覆盖 launch 默认值(与 hand_eye_calibration_node 行为一致)。
            set_parameter(rclcpp::Parameter("rgb_topic", config_.rgb_topic));
            set_parameter(rclcpp::Parameter("depth_topic", config_.depth_topic));
            set_parameter(rclcpp::Parameter("camera_info_topic", config_.camera_info_topic));
            set_parameter(rclcpp::Parameter("arm_status_topic", config_.arm_status_topic));
            set_parameter(rclcpp::Parameter("track_cmd_topic", config_.track_cmd_topic));
            set_parameter(rclcpp::Parameter("target_pose_topic", config_.target_pose_topic));
            set_parameter(rclcpp::Parameter("get_3d_points_service", config_.get_3d_points_service));
            set_parameter(rclcpp::Parameter("base_frame", config_.base_frame));
            set_parameter(rclcpp::Parameter("ee_frame", config_.ee_frame));
            set_parameter(rclcpp::Parameter("camera_frame", config_.camera_frame));
            set_parameter(rclcpp::Parameter("canvas_width", config_.canvas_width));
            set_parameter(rclcpp::Parameter("canvas_height", config_.canvas_height));
            set_parameter(rclcpp::Parameter("canvas_window", config_.canvas_window));
            set_parameter(rclcpp::Parameter("brush_thickness", config_.brush_thickness));
            set_parameter(rclcpp::Parameter("sample_step_px", config_.sample_step_px));
            set_parameter(rclcpp::Parameter("z_offset", config_.z_offset));
            set_parameter(rclcpp::Parameter("z_draw_height", config_.z_draw_height));
            set_parameter(rclcpp::Parameter("tool_offset", config_.tool_offset));
            set_parameter(rclcpp::Parameter("use_screen_plane_pose", config_.use_screen_plane_pose));
            set_parameter(rclcpp::Parameter("use_home_orientation", config_.use_home_orientation));
            set_parameter(rclcpp::Parameter("project_points_to_plane", config_.project_points_to_plane));
            set_parameter(rclcpp::Parameter("enable_segmentation_debug", config_.enable_segmentation_debug));
            set_parameter(rclcpp::Parameter("show_reproject_windows", config_.show_reproject_windows));
            set_parameter(rclcpp::Parameter("canny_low", config_.canny_low));
            set_parameter(rclcpp::Parameter("canny_high", config_.canny_high));
            set_parameter(rclcpp::Parameter("morph_kernel", config_.morph_kernel));
            set_parameter(rclcpp::Parameter("area_min_ratio", config_.area_min_ratio));
            set_parameter(rclcpp::Parameter("area_max_ratio", config_.area_max_ratio));
            set_parameter(rclcpp::Parameter("quad_approx_eps", config_.quad_approx_eps));
            set_parameter(rclcpp::Parameter("depth_median_kernel", config_.depth_median_kernel));
            set_parameter(rclcpp::Parameter("depth_invalid_radius", config_.depth_invalid_radius));
            set_parameter(rclcpp::Parameter("hand_eye_result_file", config_.hand_eye_result_file));
            set_parameter(rclcpp::Parameter("use_custom_camera_matrix", config_.use_custom_camera_matrix));
            set_parameter(rclcpp::Parameter("fx", config_.fx));
            set_parameter(rclcpp::Parameter("fy", config_.fy));
            set_parameter(rclcpp::Parameter("cx", config_.cx));
            set_parameter(rclcpp::Parameter("cy", config_.cy));
        }

        // 参数覆盖
        config_.rgb_topic = get_parameter("rgb_topic").as_string();
        config_.depth_topic = get_parameter("depth_topic").as_string();
        config_.camera_info_topic = get_parameter("camera_info_topic").as_string();
        config_.arm_status_topic = get_parameter("arm_status_topic").as_string();
        config_.track_cmd_topic = get_parameter("track_cmd_topic").as_string();
        config_.target_pose_topic = get_parameter("target_pose_topic").as_string();
        config_.get_3d_points_service = get_parameter("get_3d_points_service").as_string();
        config_.base_frame = get_parameter("base_frame").as_string();
        config_.ee_frame = get_parameter("ee_frame").as_string();
        config_.camera_frame = get_parameter("camera_frame").as_string();
        config_.canvas_width = get_parameter("canvas_width").as_int();
        config_.canvas_height = get_parameter("canvas_height").as_int();
        config_.canvas_window = get_parameter("canvas_window").as_string();
        config_.brush_thickness = get_parameter("brush_thickness").as_int();
        config_.sample_step_px = get_parameter("sample_step_px").as_double();
        config_.z_offset = get_parameter("z_offset").as_double();
        config_.z_draw_height = get_parameter("z_draw_height").as_double();
        config_.tool_offset = get_parameter("tool_offset").as_double();
        config_.use_screen_plane_pose = get_parameter("use_screen_plane_pose").as_bool();
        config_.use_home_orientation = get_parameter("use_home_orientation").as_bool();
        config_.project_points_to_plane = get_parameter("project_points_to_plane").as_bool();
        config_.enable_segmentation_debug = get_parameter("enable_segmentation_debug").as_bool();
        config_.show_reproject_windows = get_parameter("show_reproject_windows").as_bool();
        config_.canny_low = get_parameter("canny_low").as_int();
        config_.canny_high = get_parameter("canny_high").as_int();
        config_.morph_kernel = get_parameter("morph_kernel").as_int();
        config_.area_min_ratio = get_parameter("area_min_ratio").as_double();
        config_.area_max_ratio = get_parameter("area_max_ratio").as_double();
        config_.quad_approx_eps = get_parameter("quad_approx_eps").as_double();
        config_.depth_median_kernel = get_parameter("depth_median_kernel").as_int();
        config_.depth_invalid_radius = get_parameter("depth_invalid_radius").as_int();
        config_.hand_eye_result_file = get_parameter("hand_eye_result_file").as_string();
        config_.use_custom_camera_matrix = get_parameter("use_custom_camera_matrix").as_bool();
        config_.fx = get_parameter("fx").as_double();
        config_.fy = get_parameter("fy").as_double();
        config_.cx = get_parameter("cx").as_double();
        config_.cy = get_parameter("cy").as_double();

        if (config_dir_.empty())
        {
            config_dir_ = getConfigDir();
        }
        RCLCPP_INFO(get_logger(), "配置目录(configs): %s", config_dir_.c_str());
    }

    std::string getConfigDir() const
    {
        try
        {
            std::string pkg_prefix = ament_index_cpp::get_package_prefix("rigol_demo");
            fs::path share_dir = fs::path(pkg_prefix) / "share" / "rigol_demo" / "configs";
            return share_dir.string();
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "无法获取 configs 目录: %s", e.what());
            return "";
        }
    }

    // ---------- 查询屏幕分辨率 ----------
    // getWindowImageRect 在全屏切换完成前会返回旧的小窗口尺寸,
    // 导致画布只占左上角。通过 xdpyinfo 命令直接查询屏幕尺寸更可靠,
    // 且避免 X11 头文件与 Eigen/OpenCV 的宏冲突 (Success/Status 等)。
    void queryScreenResolution()
    {
        // 优先尝试 xdpyinfo
        if (queryScreenResolutionByCmd("xdpyinfo | grep dimensions", "dimensions:",
                                       screen_width_, screen_height_))
        {
            return;
        }
        // 回退: xrandr 当前连接的屏幕
        if (queryScreenResolutionByCmd("xrandr --current | grep ' connected'", "connected",
                                       screen_width_, screen_height_))
        {
            return;
        }
        // 全部失败:使用配置默认值
        RCLCPP_WARN(get_logger(),
            "无法通过 xdpyinfo/xrandr 查询屏幕尺寸,使用配置默认画布尺寸 %dx%d",
            config_.canvas_width, config_.canvas_height);
        screen_width_ = config_.canvas_width;
        screen_height_ = config_.canvas_height;
    }

    // 执行命令并从输出中解析 "WxH" 格式的分辨率
    bool queryScreenResolutionByCmd(const std::string &cmd, const std::string & /*grep_key*/,
                                    int &out_w, int &out_h)
    {
        FILE *pipe = popen(cmd.c_str(), "r");
        if (pipe == nullptr) return false;

        std::string output;
        char buffer[256];
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
        {
            output += buffer;
        }
        pclose(pipe);

        // 在输出中查找 "WxH" 模式 (如 "1920x1080")
        // xdpyinfo: "dimensions:    1920x1080 pixels ..."
        // xrandr:   "DP-1 connected primary 1920x1080+0+0 ..."
        std::regex re("(\\d+)x(\\d+)");
        std::smatch match;
        if (std::regex_search(output, match, re))
        {
            out_w = std::stoi(match[1].str());
            out_h = std::stoi(match[2].str());
            if (out_w > 0 && out_h > 0)
            {
                RCLCPP_INFO(get_logger(), "查询到屏幕分辨率: %dx%d (via: %s)",
                            out_w, out_h, cmd.c_str());
                return true;
            }
        }
        return false;
    }

    // ---------- 加载手眼标定结果 (camera -> link6) ----------
    void loadHandEyeResult()
    {
        std::string yaml_path = config_dir_ + "/" + config_.hand_eye_result_file;
        if (!fs::exists(yaml_path))
        {
            RCLCPP_WARN(get_logger(), "手眼标定结果文件不存在: %s,将使用单位矩阵", yaml_path.c_str());
            hand_eye_transform_ = cv::Mat::eye(4, 4, CV_64F);
            return;
        }

        try
        {
            YAML::Node yaml = YAML::LoadFile(yaml_path);
            YAML::Node trans = yaml["hand_eye_calibration"]["transformation"];
            if (!trans)
            {
                RCLCPP_ERROR(get_logger(), "手眼标定结果格式错误:缺少 transformation 字段");
                hand_eye_transform_ = cv::Mat::eye(4, 4, CV_64F);
                return;
            }

            double tx = trans["translation"]["x"].as<double>();
            double ty = trans["translation"]["y"].as<double>();
            double tz = trans["translation"]["z"].as<double>();

            cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
            YAML::Node rot = trans["rotation_matrix"];
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    R.at<double>(i, j) = rot[i][j].as<double>();
                }
            }

            hand_eye_transform_ = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(hand_eye_transform_(cv::Rect(0, 0, 3, 3)));
            hand_eye_transform_.at<double>(0, 3) = tx;
            hand_eye_transform_.at<double>(1, 3) = ty;
            hand_eye_transform_.at<double>(2, 3) = tz;

            has_hand_eye_ = true;
            RCLCPP_INFO(get_logger(), "手眼标定结果已加载 (camera -> link6): t=[%.4f, %.4f, %.4f]",
                        tx, ty, tz);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "加载手眼标定结果失败: %s", e.what());
            hand_eye_transform_ = cv::Mat::eye(4, 4, CV_64F);
        }
    }

    // ---------- 鼠标回调 ----------
    static void mouseCallback(int event, int x, int y, int flags, void *userdata)
    {
        auto *self = static_cast<TrajectoryDrawNode *>(userdata);
        self->handleMouse(event, x, y, flags);
    }

    void handleMouse(int event, int x, int y, int /*flags*/)
    {
        std::lock_guard<std::mutex> lock(canvas_mutex_);
        // 画布尺寸已等于屏幕分辨率,鼠标坐标直接对应画布坐标
        int cx = std::clamp(x, 0, config_.canvas_width - 1);
        int cy = std::clamp(y, 0, config_.canvas_height - 1);

        if (event == cv::EVENT_LBUTTONDOWN)
        {
            // 鼠标按下:开始新一段轨迹,清除前一段
            current_stroke_.clear();
            current_stroke_.emplace_back(cx, cy);
            last_point_ = cv::Point(cx, cy);
            drawing_ = true;
            // 清除画布上前一段轨迹(重置为白色)
            canvas_.setTo(cv::Scalar(255, 255, 255));
            cv::circle(canvas_, last_point_, config_.brush_thickness / 2, cv::Scalar(0, 0, 255), -1);
        }
        else if (event == cv::EVENT_MOUSEMOVE && drawing_)
        {
            cv::line(canvas_, last_point_, cv::Point(cx, cy), cv::Scalar(0, 0, 255), config_.brush_thickness, cv::LINE_AA);
            current_stroke_.emplace_back(cx, cy);
            last_point_ = cv::Point(cx, cy);
        }
        else if (event == cv::EVENT_LBUTTONUP)
        {
            drawing_ = false;
            // 轨迹断开:保留 current_stroke_,下次按下时会清除画布
        }
    }

    void clearCanvas()
    {
        std::lock_guard<std::mutex> lock(canvas_mutex_);
        canvas_.setTo(cv::Scalar(255, 255, 255));
        current_stroke_.clear();
        RCLCPP_INFO(get_logger(), "画布已清除。");
    }

    // ---------- 确认轨迹:重投影 + 发布 ----------
    void confirmTrajectory()
    {
        std::vector<cv::Point> stroke;
        {
            std::lock_guard<std::mutex> lock(canvas_mutex_);
            stroke = current_stroke_;
        }

        if (stroke.size() < 2)
        {
            RCLCPP_WARN(get_logger(), "轨迹点数不足 (%zu),请先在画布上绘制轨迹。", stroke.size());
            return;
        }

        if (!has_rgb_ || !has_depth_)
        {
            RCLCPP_WARN(get_logger(), "尚未收到 RGB/深度图像,无法重投影。请确保深度相机节点已启动。");
            return;
        }

        if (!has_camera_info_)
        {
            RCLCPP_WARN(get_logger(), "尚未收到相机内参,无法重投影。");
            return;
        }

        RCLCPP_INFO(get_logger(), "确认轨迹,共 %zu 个画布点,开始重投影...", stroke.size());

        // 1. 离散化画布轨迹
        std::vector<cv::Point2f> canvas_pts = discretizeStroke(stroke);
        RCLCPP_INFO(get_logger(), "离散化后得到 %zu 个点。", canvas_pts.size());

        // 2. 在 RGB 图像中检测屏幕区域(颜色梯度分割)
        cv::Mat rgb;
        cv::Mat depth_raw;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            rgb = rgb_image_.clone();
            depth_raw = depth_image_.clone();
        }

        std::vector<cv::Point2f> screen_quad;
        if (!detectScreenRegion(rgb, screen_quad))
        {
            RCLCPP_ERROR(get_logger(), "未能检测到屏幕区域,请调整分割参数或相机视角。");
            return;
        }
        RCLCPP_INFO(get_logger(), "检测到屏幕四边形: [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]",
                    screen_quad[0].x, screen_quad[0].y, screen_quad[1].x, screen_quad[1].y,
                    screen_quad[2].x, screen_quad[2].y, screen_quad[3].x, screen_quad[3].y);

        // 3. 计算画布区域 -> 屏幕四边形的单应矩阵
        //    画布四角 (0,0) (W,0) (W,H) (0,H) 对应屏幕四边形
        std::vector<cv::Point2f> canvas_corners = {
            cv::Point2f(0, 0),
            cv::Point2f(static_cast<float>(config_.canvas_width), 0),
            cv::Point2f(static_cast<float>(config_.canvas_width), static_cast<float>(config_.canvas_height)),
            cv::Point2f(0, static_cast<float>(config_.canvas_height))
        };

        // 对屏幕四边形按左上、右上、右下、左下排序,与画布四角对应
        std::vector<cv::Point2f> ordered_screen = orderQuadCorners(screen_quad);

        // H: 画布坐标 -> 屏幕图像坐标
        cv::Mat H = cv::getPerspectiveTransform(canvas_corners, ordered_screen);
        // H_inv: 屏幕图像坐标 -> 画布坐标 (此处需要画布->图像,直接用 H)
        // 题目要求:屏幕四边形到画布区域的变换矩阵求逆可用于画布上轨迹二维坐标重投影到深度相机RGB画面
        // 即 H_screen_to_canvas 的逆 = H_canvas_to_screen = H
        cv::Mat H_inv = H.inv();  // 屏幕 -> 画布

        RCLCPP_INFO(get_logger(), "计算单应矩阵 H (画布->图像) 完成。");

        // 4. 将画布离散点重投影到 RGB 图像坐标
        std::vector<cv::Point2f> image_pts;
        cv::perspectiveTransform(canvas_pts, image_pts, H);

        // 可视化:在 RGB 图像上绘制重投影点
        cv::Mat rgb_with_pts = rgb.clone();
        for (size_t i = 0; i < image_pts.size(); ++i)
        {
            cv::circle(rgb_with_pts, image_pts[i], 2, cv::Scalar(0, 0, 255), -1);
            if (i > 0)
            {
                cv::line(rgb_with_pts, image_pts[i - 1], image_pts[i], cv::Scalar(0, 255, 0), 1);
            }
        }
        // 绘制屏幕四边形
        for (int i = 0; i < 4; ++i)
        {
            cv::line(rgb_with_pts, ordered_screen[i], ordered_screen[(i + 1) % 4], cv::Scalar(255, 0, 0), 2);
        }
        if (config_.show_reproject_windows)
        {
            cv::imshow("Reprojected Trajectory on RGB", rgb_with_pts);
        }

        // 显示屏幕区域重投影到画布大小的结果
        cv::Mat screen_warped;
        cv::warpPerspective(rgb, screen_warped, H_inv,
                            cv::Size(config_.canvas_width, config_.canvas_height));
        if (config_.show_reproject_windows)
        {
            cv::imshow("Screen Warped to Canvas", screen_warped);
        }

        // 5. 结合深度信息将 2D 像素转换为相机坐标系 3D 点
        std::vector<cv::Point3f> camera_pts = deprojectToCamera(image_pts, depth_raw);
        if (camera_pts.empty())
        {
            RCLCPP_ERROR(get_logger(), "3D 点转换失败(深度无效)。");
            return;
        }
        RCLCPP_INFO(get_logger(), "转换为相机坐标系 3D 点 %zu 个。", camera_pts.size());

        // 6. 计算屏幕平面姿态(相机坐标系下拟合,返回相机坐标系法向量)
        //    预选点: 4 角点 + 4 边中点 + 1 中心点,共 9 个点
        //    normal 指向屏幕前方(远离相机,与视线方向一致)
        cv::Vec3d plane_normal_cam(0.0, 0.0, 1.0);  // 默认指向屏幕前方
        cv::Vec3d plane_center_cam(0.0, 0.0, 0.0);
        bool has_plane_pose = false;

        // 7. 计算相机坐标系下的目标姿态 RPY (工具 Z 轴沿 -normal 指向屏幕)
        cv::Vec3d camera_target_rpy(0.0, M_PI/2.0, 0.0);

        // 始终拟合屏幕平面法向量(用于位置偏移)
        // 姿态使用计算出的屏幕平面拟合姿态(use_screen_plane_pose=true 时)
        if (config_.use_home_orientation)
        {
            // use_home_orientation=true: 姿态用 base 系 home RPY,但位置偏移用拟合法向量
            if (config_.use_screen_plane_pose) {
                camera_target_rpy = computeScreenPlanePose(H, depth_raw, plane_normal_cam, plane_center_cam);
                has_plane_pose = true;
            }
            RCLCPP_INFO(get_logger(), "使用 home 一致姿态(base 系 RPY=(0,π/2,0)),但位置偏移用拟合法向量 normal=(%.3f,%.3f,%.3f)",
                        plane_normal_cam[0], plane_normal_cam[1], plane_normal_cam[2]);
        }
        else if (config_.use_screen_plane_pose)
        {
            // 使用计算出的屏幕平面拟合姿态
            camera_target_rpy = computeScreenPlanePose(H, depth_raw, plane_normal_cam, plane_center_cam);
            has_plane_pose = true;
            RCLCPP_INFO(get_logger(), "使用屏幕平面拟合姿态(相机系 RPY=(%.3f,%.3f,%.3f)),法向量 normal=(%.3f,%.3f,%.3f)",
                        camera_target_rpy[0], camera_target_rpy[1], camera_target_rpy[2],
                        plane_normal_cam[0], plane_normal_cam[1], plane_normal_cam[2]);
        }
        
        /*if (has_plane_pose)
        {
            camera_target_rpy = normalToRPY(plane_normal_cam);
        }*/

        // 7.5 将路径点投影到拟合的屏幕平面上(消除深度噪声,确保所有路径点共面)
        //     投影公式: P_proj = P - ((P - C) · n) * n
        //     其中 n 为平面单位法向量, C 为平面上一点(拟合质心)
        if (config_.project_points_to_plane && has_plane_pose)
        {
            double mean_dist_before = 0.0;
            for (const auto &p : camera_pts)
            {
                mean_dist_before += std::abs(
                    (p.x - plane_center_cam[0]) * plane_normal_cam[0] +
                    (p.y - plane_center_cam[1]) * plane_normal_cam[1] +
                    (p.z - plane_center_cam[2]) * plane_normal_cam[2]);
            }
            mean_dist_before /= static_cast<double>(camera_pts.size());

            projectPointsToPlane(camera_pts, plane_normal_cam, plane_center_cam);

            RCLCPP_INFO(get_logger(),
                "已将 %zu 个路径点投影到拟合屏幕平面 (normal=(%.3f,%.3f,%.3f), center=(%.3f,%.3f,%.3f)), "
                "投影前平均离面距离=%.4f m",
                camera_pts.size(),
                plane_normal_cam[0], plane_normal_cam[1], plane_normal_cam[2],
                plane_center_cam[0], plane_center_cam[1], plane_center_cam[2],
                mean_dist_before);
        }
        else if (config_.project_points_to_plane && !has_plane_pose)
        {
            RCLCPP_WARN(get_logger(), "未拟合屏幕平面,跳过路径点平面投影(使用原始深度反投影 3D 点)。");
        }

        // 8. 清除屏幕上原有轨迹(发布空 TrackCmd 通知,或由擦除屏幕区域)
        clearScreenTrajectory();

        // 9. 打包为 TrackCmd 发布
        //    每个轨迹点:相机坐标系下 (位置 + 姿态) 通过手眼变换转到 base_link
        //    位置沿 -normal 抬离 tool_offset (normal 指向屏幕前方,工具朝相机侧抬离)
        //    轨迹结构: [起点过渡点] + [路径点...] + [终点过渡点]
        //    过渡点 = 路径点再沿 -normal 向外偏移 4mm(approach_offset),
        //    用于 home → 过渡点(OMPL) → 路径点(直线) ... 路径点(直线) → 过渡点(OMPL) → home
        demo_msgs::msg::TrackCmd cmd;

        // 辅助 lambda:相机系 3D 点 + 姿态 -> base 系 TargetPose
        auto camToBasePose = [&](const cv::Point3f &p_cam_offset,
                                   const cv::Vec3d &rpy) -> demo_msgs::msg::TargetPose {
            demo_msgs::msg::TargetPose pose;
            std::array<double, 6> base_pose;
            if (config_.use_home_orientation) {
                base_pose = transformPoseCameraToBaseWithBaseOrientation(
                    p_cam_offset, 0.0, M_PI/2.0, 0.0);
            } else {
                base_pose = transformPoseCameraToBase(p_cam_offset, rpy);
            }
            pose.x = base_pose[0]; pose.y = base_pose[1]; pose.z = base_pose[2];
            pose.roll = base_pose[3]; pose.pitch = base_pose[4]; pose.yaw = base_pose[5];
            return pose;
        };

        // 起点过渡点:第一个路径点在屏幕的位置 + 沿 -normal 再向外偏移 approach_offset
        // 路径点已偏移 tool_offset,过渡点再偏移 approach_offset(总共 tool_offset + approach_offset)
        const double approach_offset = 0.004;  // 屏幕前 4mm
        {
            const auto &p_first = camera_pts.front();
            cv::Point3f p_approach(
                p_first.x - plane_normal_cam[0] * (config_.tool_offset + approach_offset),
                p_first.y - plane_normal_cam[1] * (config_.tool_offset + approach_offset),
                p_first.z - plane_normal_cam[2] * (config_.tool_offset + approach_offset));
            cmd.poses.push_back(camToBasePose(p_approach, camera_target_rpy));
            RCLCPP_INFO(get_logger(), "起点过渡点(屏幕前%.0fmm): pos=(%.3f,%.3f,%.3f)",
                        approach_offset * 1000, p_approach.x, p_approach.y, p_approach.z);
        }

        // 路径点
        for (size_t i = 0; i < camera_pts.size(); ++i)
        {
            const auto &p_cam = camera_pts[i];
            cv::Point3f p_cam_offset(
                p_cam.x - plane_normal_cam[0] * config_.tool_offset,
                p_cam.y - plane_normal_cam[1] * config_.tool_offset,
                p_cam.z - plane_normal_cam[2] * config_.tool_offset);
            cmd.poses.push_back(camToBasePose(p_cam_offset, camera_target_rpy));
            RCLCPP_INFO(get_logger(), "点 %zu 位置: pos=(%.3f,%.3f,%.3f)",
                    i + 1, p_cam_offset.x, p_cam_offset.y, p_cam_offset.z);
        }

        // 终点过渡点:最后一个路径点在屏幕的位置 + 沿 -normal 再向外偏移 approach_offset
        {
            const auto &p_last = camera_pts.back();
            cv::Point3f p_retreat(
                p_last.x - plane_normal_cam[0] * (config_.tool_offset + approach_offset),
                p_last.y - plane_normal_cam[1] * (config_.tool_offset + approach_offset),
                p_last.z - plane_normal_cam[2] * (config_.tool_offset + approach_offset));
            cmd.poses.push_back(camToBasePose(p_retreat, camera_target_rpy));
            RCLCPP_INFO(get_logger(), "终点过渡点(屏幕前%.0fmm): pos=(%.3f,%.3f,%.3f)",
                        approach_offset * 1000, p_retreat.x, p_retreat.y, p_retreat.z);
        }
        RCLCPP_INFO(get_logger(), "首点 base 位姿: pos=(%.3f,%.3f,%.3f) RPY=(%.3f,%.3f,%.3f)",
                    cmd.poses.front().x, cmd.poses.front().y, cmd.poses.front().z,
                    cmd.poses.front().roll, cmd.poses.front().pitch, cmd.poses.front().yaw);
        track_cmd_pub_->publish(cmd);
        RCLCPP_INFO(get_logger(), "已发布 TrackCmd 到 %s,共 %zu 个目标点。",
                    config_.track_cmd_topic.c_str(), cmd.poses.size());
        RCLCPP_INFO(get_logger(), "屏幕姿态: rpy=(%.3f,%.3f,%.3f) ",
                    camera_target_rpy[0], camera_target_rpy[1], camera_target_rpy[2]);
        RCLCPP_INFO(get_logger(), "屏幕法向量: normal=(%.3f,%.3f,%.3f) ",
                    plane_normal_cam[0], plane_normal_cam[1], plane_normal_cam[2]);

        // 清除画布,准备下一段
        clearCanvas();
    }

    // ---------- 离散化画布轨迹 ----------
    std::vector<cv::Point2f> discretizeStroke(const std::vector<cv::Point> &stroke)
    {
        std::vector<cv::Point2f> pts;
        if (stroke.empty()) return pts;

        double accum = 0.0;
        pts.emplace_back(static_cast<float>(stroke[0].x), static_cast<float>(stroke[0].y));
        cv::Point prev = stroke[0];

        for (size_t i = 1; i < stroke.size(); ++i)
        {
            double dx = stroke[i].x - prev.x;
            double dy = stroke[i].y - prev.y;
            double seg = std::sqrt(dx * dx + dy * dy);
            accum += seg;
            if (accum >= config_.sample_step_px)
            {
                pts.emplace_back(static_cast<float>(stroke[i].x), static_cast<float>(stroke[i].y));
                accum = 0.0;
            }
            prev = stroke[i];
        }
        // 确保最后一个点包含
        cv::Point2f last(static_cast<float>(stroke.back().x), static_cast<float>(stroke.back().y));
        if (pts.empty() || cv::norm(pts.back() - last) > 1.0)
        {
            pts.push_back(last);
        }
        return pts;
    }

    // ---------- 颜色梯度分割检测屏幕区域 ----------
    bool detectScreenRegion(const cv::Mat &rgb, std::vector<cv::Point2f> &screen_quad)
    {
        cv::Mat gray;
        cv::cvtColor(rgb, gray, cv::COLOR_BGR2GRAY);

        // 颜色梯度:使用 Canny 边缘检测
        cv::Mat edges;
        cv::Canny(gray, edges, config_.canny_low, config_.canny_high);

        // 形态学闭操作连接边缘
        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_RECT, cv::Size(config_.morph_kernel, config_.morph_kernel));
        cv::Mat closed;
        cv::morphologyEx(edges, closed, cv::MORPH_CLOSE, kernel);

        // 查找轮廓
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(closed, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double img_area = static_cast<double>(rgb.rows * rgb.cols);
        double min_area = img_area * config_.area_min_ratio;
        double max_area = img_area * config_.area_max_ratio;

        // 筛选符合面积约束的最大四边形
        double max_quad_area = 0.0;
        std::vector<cv::Point> best_quad;
        bool found = false;

        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            if (area < min_area || area > max_area) continue;

            // 多边形逼近
            double peri = cv::arcLength(contour, true);
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contour, approx, config_.quad_approx_eps * peri, true);

            if (approx.size() == 4 && cv::isContourConvex(approx))
            {
                if (area > max_quad_area)
                {
                    max_quad_area = area;
                    best_quad = approx;
                    found = true;
                }
            }
        }

        if (config_.enable_segmentation_debug)
        {
            showSegmentationDebug(rgb, gray, edges, closed, contours, best_quad, found);
        }

        if (!found)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "未找到符合面积约束的四边形区域。");
            return false;
        }

        screen_quad.clear();
        for (const auto &p : best_quad)
        {
            screen_quad.emplace_back(static_cast<float>(p.x), static_cast<float>(p.y));
        }
        return true;
    }

    // ---------- 分割可视化调参 ----------
    void showSegmentationDebug(const cv::Mat &rgb, const cv::Mat &gray,
                               const cv::Mat &edges, const cv::Mat &closed,
                               const std::vector<std::vector<cv::Point>> &contours,
                               const std::vector<cv::Point> &best_quad, bool found)
    {
        cv::Mat debug = rgb.clone();

        // 绘制所有轮廓
        cv::drawContours(debug, contours, -1, cv::Scalar(0, 255, 255), 1);

        // 绘制最佳四边形
        if (found)
        {
            for (int i = 0; i < 4; ++i)
            {
                cv::line(debug, best_quad[i], best_quad[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
            }
        }

        // 拼接显示:灰度图 | 边缘图 | 闭操作图 | 结果图
        cv::Mat gray_bgr, edges_bgr, closed_bgr;
        cv::cvtColor(gray, gray_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(edges, edges_bgr, cv::COLOR_GRAY2BGR);
        cv::cvtColor(closed, closed_bgr, cv::COLOR_GRAY2BGR);

        cv::Mat top, bottom, combined;
        cv::hconcat(gray_bgr, edges_bgr, top);
        cv::hconcat(closed_bgr, debug, bottom);
        cv::vconcat(top, bottom, combined);

        cv::putText(combined, "Gray | Edges | Closed | Result",
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        // 缩放以适应屏幕
        cv::Mat combined_scaled;
        cv::resize(combined, combined_scaled, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
        cv::imshow("Segmentation Debug", combined_scaled);
    }

    // ---------- 初始化调参滑动条 ----------
    void initDebugTrackbars()
    {
        cv::namedWindow("Tuning Controls", cv::WINDOW_NORMAL);
        cv::resizeWindow("Tuning Controls", 400, 300);

        // 预创建调试显示窗口
        cv::namedWindow("Segmentation Debug", cv::WINDOW_NORMAL);
        cv::namedWindow("Screen Warped to Canvas", cv::WINDOW_NORMAL);

        // 从当前配置初始化滑动条值
        tb_canny_low_ = config_.canny_low;
        tb_canny_high_ = config_.canny_high;
        tb_morph_kernel_ = config_.morph_kernel;
        tb_area_min_ = static_cast<int>(config_.area_min_ratio * 100.0);
        tb_area_max_ = static_cast<int>(config_.area_max_ratio * 100.0);
        tb_quad_eps_ = static_cast<int>(config_.quad_approx_eps * 1000.0);

        cv::createTrackbar("Canny Low", "Tuning Controls", &tb_canny_low_, 255);
        cv::createTrackbar("Canny High", "Tuning Controls", &tb_canny_high_, 255);
        cv::createTrackbar("Morph Kernel", "Tuning Controls", &tb_morph_kernel_, 31);
        cv::createTrackbar("Area Min (x100)", "Tuning Controls", &tb_area_min_, 100);
        cv::createTrackbar("Area Max (x100)", "Tuning Controls", &tb_area_max_, 100);
        cv::createTrackbar("Quad Eps (x1000)", "Tuning Controls", &tb_quad_eps_, 100);

        RCLCPP_INFO(get_logger(),
            "调参窗口已创建: Tuning Controls (滑动条) + Segmentation Debug (分割中间结果) + "
            "Screen Warped to Canvas (屏幕重投影到画布大小)");
    }

    // ---------- 从滑动条更新配置 ----------
    void updateConfigFromTrackbars()
    {
        config_.canny_low = tb_canny_low_;
        config_.canny_high = std::max(tb_canny_high_, tb_canny_low_ + 1);
        // 形态学核必须为正奇数
        int k = tb_morph_kernel_;
        if (k < 1) k = 1;
        if (k % 2 == 0) k += 1;
        config_.morph_kernel = k;
        config_.area_min_ratio = std::clamp(tb_area_min_ / 100.0, 0.0, 1.0);
        config_.area_max_ratio = std::clamp(tb_area_max_ / 100.0, 0.0, 1.0);
        config_.quad_approx_eps = std::clamp(tb_quad_eps_ / 1000.0, 0.001, 0.1);
    }

    // ---------- 实时分割调参循环 ----------
    void runSegmentationDebug()
    {
        if (!has_rgb_) return;

        // 从滑动条读取最新参数
        updateConfigFromTrackbars();

        cv::Mat rgb;
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            rgb = rgb_image_.clone();
        }
        if (rgb.empty()) return;

        // 运行分割 (detectScreenRegion 内部会调用 showSegmentationDebug 显示中间结果)
        std::vector<cv::Point2f> screen_quad;
        if (!detectScreenRegion(rgb, screen_quad))
        {
            return;
        }

        // 计算屏幕->画布的单应矩阵,将屏幕区域重投影到画布大小
        std::vector<cv::Point2f> canvas_corners = {
            cv::Point2f(0, 0),
            cv::Point2f(static_cast<float>(config_.canvas_width), 0),
            cv::Point2f(static_cast<float>(config_.canvas_width),
                        static_cast<float>(config_.canvas_height)),
            cv::Point2f(0, static_cast<float>(config_.canvas_height))
        };
        std::vector<cv::Point2f> ordered_screen = orderQuadCorners(screen_quad);

        // H: 画布->屏幕; H_inv: 屏幕->画布
        cv::Mat H = cv::getPerspectiveTransform(canvas_corners, ordered_screen);
        cv::Mat H_inv = H.inv();

        // 将 RGB 图像中的屏幕区域 warp 到画布大小
        cv::Mat screen_warped;
        cv::warpPerspective(rgb, screen_warped, H_inv,
                            cv::Size(config_.canvas_width, config_.canvas_height));

        // 在 warped 图上绘制画布网格 (便于对比对齐)
        for (int x = 0; x <= config_.canvas_width; x += 100)
        {
            cv::line(screen_warped, cv::Point(x, 0), cv::Point(x, config_.canvas_height),
                     cv::Scalar(0, 255, 255), 1);
        }
        for (int y = 0; y <= config_.canvas_height; y += 100)
        {
            cv::line(screen_warped, cv::Point(0, y), cv::Point(config_.canvas_width, y),
                     cv::Scalar(0, 255, 255), 1);
        }
        cv::putText(screen_warped, "Screen reprojected to canvas size",
                    cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 1);
        cv::imshow("Screen Warped to Canvas", screen_warped);
    }

    // ---------- 四边形角点排序:左上、右上、右下、左下 ----------
    std::vector<cv::Point2f> orderQuadCorners(const std::vector<cv::Point2f> &quad)
    {
        std::vector<cv::Point2f> ordered(4);
        // 按坐标和排序:左上最小,右下最大
        std::vector<cv::Point2f> pts = quad;
        // 左上: x+y 最小
        auto lt = std::min_element(pts.begin(), pts.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return (a.x + a.y) < (b.x + b.y); });
        ordered[0] = *lt;
        // 右下: x+y 最大
        auto rb = std::max_element(pts.begin(), pts.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return (a.x + a.y) < (b.x + b.y); });
        ordered[2] = *rb;
        // 右上: x-y 最大
        auto rt = std::max_element(pts.begin(), pts.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return (a.x - a.y) < (b.x - b.y); });
        ordered[1] = *rt;
        // 左下: x-y 最小
        auto lb = std::min_element(pts.begin(), pts.end(),
            [](const cv::Point2f &a, const cv::Point2f &b) { return (a.x - a.y) < (b.x - b.y); });
        ordered[3] = *lb;
        return ordered;
    }

    // ---------- 2D 像素 -> 相机坐标系 3D 点 ----------
    // 通过调用 d435i_data_publisher_node 的 /camera/get_3d_points 服务,
    // 使用 RS_D435i::get3DPoint (基于 librealsense rs2_deproject_pixel_to_point)
    // 完成反投影,避免在本节点重复实现深度反投影逻辑。
    std::vector<cv::Point3f> deprojectToCamera(const std::vector<cv::Point2f> &image_pts,
                                               const cv::Mat & /*depth_raw*/)
    {
        std::vector<cv::Point3f> camera_pts;
        if (image_pts.empty())
        {
            return camera_pts;
        }
        if (!get_3d_points_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(get_logger(),
                "3D 点反投影服务 %s 不可用,请确保 d435i_data_publisher_node 已启动",
                config_.get_3d_points_service.c_str());
            return camera_pts;
        }

        auto request = std::make_shared<demo_msgs::srv::Get3DPoints::Request>();
        request->pixels.reserve(image_pts.size());
        for (const auto &pt : image_pts)
        {
            geometry_msgs::msg::Point pix;
            pix.x = pt.x;
            pix.y = pt.y;
            pix.z = 0.0;
            request->pixels.push_back(pix);
        }

        auto future = get_3d_points_client_->async_send_request(request);
        // 节点已在 spin 线程中持续处理回调,直接轮询 future 即可
        auto status = future.wait_for(std::chrono::seconds(5));
        if (status != std::future_status::ready)
        {
            RCLCPP_ERROR(get_logger(), "3D 点反投影服务调用超时");
            return camera_pts;
        }
        auto response = future.get();
        if (!response)
        {
            RCLCPP_ERROR(get_logger(), "3D 点反投影服务返回空响应");
            return camera_pts;
        }

        camera_pts.reserve(response->points.size());
        for (size_t i = 0; i < response->points.size(); ++i)
        {
            const auto &pt = response->points[i];
            camera_pts.emplace_back(pt.x, pt.y, pt.z);
            RCLCPP_INFO(get_logger(),
                "  像素(%.1f, %.1f) -> 3D(%.4f, %.4f, %.4f) m",
                image_pts[i].x, image_pts[i].y, pt.x, pt.y, pt.z);
        }
        RCLCPP_INFO(get_logger(), "反投影完成: 请求 %zu 点, 有效 %u 点",
                    image_pts.size(), response->valid_count);
        return camera_pts;
    }

    // 获取深度值(无效时邻域平均) —— 保留用于本地深度查询备用
    float getDepthAt(const cv::Mat &depth, int x, int y)
    {
        uint16_t d = depth.at<uint16_t>(y, x);
        if (d != 0)
        {
            return static_cast<float>(d) * 0.001f;  // mm -> m
        }
        // 邻域平均
        int r = config_.depth_invalid_radius;
        float sum = 0.0f;
        int cnt = 0;
        for (int yy = std::max(0, y - r); yy <= std::min(depth.rows - 1, y + r); ++yy)
        {
            for (int xx = std::max(0, x - r); xx <= std::min(depth.cols - 1, x + r); ++xx)
            {
                uint16_t dd = depth.at<uint16_t>(yy, xx);
                if (dd != 0)
                {
                    sum += static_cast<float>(dd) * 0.001f;
                    ++cnt;
                }
            }
        }
        return cnt > 0 ? sum / cnt : 0.0f;
    }

    void logCvMat(const std::string &name, const cv::Mat &mat)
    {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(6);
        for (int r = 0; r < mat.rows; ++r)
        {
            for (int c = 0; c < mat.cols; ++c)
            {
                oss << mat.at<double>(r, c);
                if (c + 1 < mat.cols)
                {
                    oss << ", ";
                }
            }
            if (r + 1 < mat.rows)
            {
                oss << "\n";
            }
        }
        RCLCPP_INFO(get_logger(), "%s:\n%s", name.c_str(), oss.str().c_str());
    }

    cv::Mat buildCameraOpticalToBaseRotation() const
    {
        return (cv::Mat_<double>(3, 3) <<
            0.0, 0.0, 1.0,
            -1.0, 0.0, 0.0,
            0.0, -1.0, 0.0);
    }

    Eigen::Isometry3f cvMatToIsometry3f(const cv::Mat &T_cv)
    {
        assert(T_cv.rows == 4 && T_cv.cols == 4);

        Eigen::Matrix4f T;

        if (T_cv.type() == CV_32F)
        {
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    T(i, j) = T_cv.at<float>(i, j);
        }
        else if (T_cv.type() == CV_64F)
        {
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    T(i, j) = static_cast<float>(T_cv.at<double>(i, j));
        }
        else
        {
            throw std::runtime_error("Unsupported cv::Mat type.");
        }

        return Eigen::Isometry3f(T);
    }

    // ---------- 相机坐标系位姿(位置+姿态) -> base_link flange 位姿 ----------
    // 参考 transformTargetPose 的变换方式:
    //   T_flange_camera = inv(hand_eye_transform_)   (link6->camera)
    //   T_base_flange   = TF(base_link->link6)
    //   T_base_camera   = T_base_flange * T_flange_camera
    //   T_camera_target = 相机坐标系下目标位姿 (pos + RPY)
    //   T_base_target   = T_base_camera * T_camera_target
    //   T_base_flange_new = T_base_target  (无 TCP 时即 flange 目标位姿)
    // 返回 base_link 下 flange 的 (x,y,z,roll,pitch,yaw)
    std::array<double, 6> transformPoseCameraToBase(
        const cv::Point3f &camera_pos, const cv::Vec3d &camera_rpy)
    {
        std::array<double, 6> result = {0, 0, 0, 0, 0, 0};

        // 获取 base_link -> link6 的 TF
        geometry_msgs::msg::TransformStamped tf_base_ee;
        try
        {
            tf_base_ee = tf_buffer_->lookupTransform(
                config_.base_frame, config_.ee_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(get_logger(), "无法获取 TF %s -> %s: %s",
                         config_.base_frame.c_str(), config_.ee_frame.c_str(), e.what());
            return result;
        }

        Eigen::Isometry3f T_base_flange = tfToI3f(tf_base_ee);
        Eigen::Isometry3f T_flange_camera = cvMatToIsometry3f(hand_eye_transform_);

        float roll = camera_rpy[0];
        float pitch = camera_rpy[1];
        float yaw = camera_rpy[2];

        Eigen::AngleAxisf Rz(yaw, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());

        Eigen::Isometry3f T_camera_target = Eigen::Isometry3f::Identity();
        T_camera_target.linear() = (Rz * Ry * Rx).toRotationMatrix();
        T_camera_target.translation() = Eigen::Vector3f(
            camera_pos.x,
            camera_pos.y,
            camera_pos.z);

        Eigen::Isometry3f T_base_target = T_base_flange * T_flange_camera * T_camera_target;


        result[0] = T_base_target.translation().x();
        result[1] = T_base_target.translation().y();
        result[2] = T_base_target.translation().z();

        Eigen::Matrix3f R = T_base_target.rotation();
        // 末端 Z 轴方向(旋转矩阵第3列)
        Eigen::Vector3f end_z_axis = R.col(2);

        // home pose 末端 Z 轴在 base 系下 ≈ (1, 0, 0)(朝前,base X 正方向)
        // 若目标末端 Z 轴与 home 方向相反(点积 < 0),说明旋转矩阵需要调整
        // 调整方法:绕末端 Z 轴旋转 π(R = R * Rz(π)),这样末端 Z 轴方向不变,
        // 但 X/Y 轴翻转,使末端 Z 轴朝向与 home 一致(朝前而非朝后)
        // 注意:绕末端 Z 轴旋转 π 不改变末端 Z 轴方向,只改变 X/Y 轴方向
        // 但这里的问题是末端 Z 轴本身朝向错误,需要绕世界 X 轴翻转
        //
        // 实际上:eulerAngles(2,1,0) 的多值性导致同一旋转矩阵可能返回
        // pitch≈π/2(正确) 或 pitch≈±π(错误)。正确的做法是确保提取的 RPY
        // 用 setRPY 转回后与原旋转矩阵等价,且 pitch 在 [-π/2, π/2] 范围内
        // (与 home pose pitch=π/2 同侧)。
        //
        // 策略:若末端 Z 轴的 X 分量 < 0(朝后,与 home 相反),
        // 则绕世界 Y 轴旋转 π(R = Ry(π) * R),使末端 Z 轴翻转到朝前,
        // 然后重新提取 RPY。这等价于把 pitch 从 ±π 翻到 0 附近,
        // 或从 ±π/2 翻到 ∓π/2(但保持末端 Z 轴朝前)。
        if (end_z_axis.x() < 0.0f) {
            // 绕世界 Y 轴旋转 π:R_new = Ry(π) * R
            // 这会使末端 Z 轴方向翻转:(x,y,z) -> (-x, y, -z)
            // 即末端 Z 轴从朝后变成朝前
            Eigen::AngleAxisf Ry_pi(static_cast<float>(M_PI), Eigen::Vector3f::UnitY());
            R = Ry_pi.toRotationMatrix() * R;
            end_z_axis = R.col(2);
        }

        Eigen::Vector3f ypr = R.eulerAngles(2, 1, 0);
        result[3] = ypr.z();   // roll
        result[4] = ypr.y();   // pitch
        result[5] = ypr.x();   // yaw
        
        RCLCPP_INFO_STREAM(get_logger(), "\n===== T_base_flange =====\n" << T_base_flange.matrix());
        RCLCPP_INFO_STREAM(get_logger(), "\n===== T_flange_camera =====\n" << T_flange_camera.matrix());
        RCLCPP_INFO_STREAM(get_logger(), "\n===== T_camera_target =====\n" << T_camera_target.matrix());
        RCLCPP_INFO_STREAM(get_logger(), "\n===== T_base_target =====\n" << T_base_target.matrix());
        RCLCPP_INFO(get_logger(), "末端 Z 轴(base系): [%.3f, %.3f, %.3f], RPY=(%.3f, %.3f, %.3f)",
                    end_z_axis.x(), end_z_axis.y(), end_z_axis.z(),
                    result[3], result[4], result[5]);
        
        return result;
    }

    // ---------- 相机坐标系位置 -> base_link flange 位姿(姿态直接用 base 系 RPY) ----------
    // 用于 use_home_orientation=true 场景:home 姿态是 base 系下的 RPY=(0,π/2,0),
    // 不是相机系。此函数只变换位置(相机系->base系),姿态直接用传入的 base 系 RPY。
    // 变换链:
    //   T_base_camera = T_base_flange * T_flange_camera
    //   p_base = T_base_camera * p_camera  (仅位置变换)
    //   R_base_target = setRPY(base_roll, base_pitch, base_yaw)  (姿态直接用 base 系 RPY)
    std::array<double, 6> transformPoseCameraToBaseWithBaseOrientation(
        const cv::Point3f &camera_pos,
        double base_roll, double base_pitch, double base_yaw)
    {
        std::array<double, 6> result = {0, 0, 0, 0, 0, 0};

        // 获取 base_link -> link6 的 TF
        geometry_msgs::msg::TransformStamped tf_base_ee;
        try
        {
            tf_base_ee = tf_buffer_->lookupTransform(
                config_.base_frame, config_.ee_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_ERROR(get_logger(), "无法获取 TF %s -> %s: %s",
                         config_.base_frame.c_str(), config_.ee_frame.c_str(), e.what());
            return result;
        }

        Eigen::Isometry3f T_base_flange = tfToI3f(tf_base_ee);
        Eigen::Isometry3f T_flange_camera = cvMatToIsometry3f(hand_eye_transform_);
        Eigen::Isometry3f T_base_camera = T_base_flange * T_flange_camera;

        // 仅变换位置:相机系 3D 点 -> base 系 3D 点
        Eigen::Vector3f p_cam(camera_pos.x, camera_pos.y, camera_pos.z);
        Eigen::Vector3f p_base = T_base_camera * p_cam;
        result[0] = p_base.x();
        result[1] = p_base.y();
        result[2] = p_base.z();

        // 姿态直接用 base 系 RPY(不经过相机系变换)
        result[3] = base_roll;
        result[4] = base_pitch;
        result[5] = base_yaw;

        RCLCPP_INFO(get_logger(), "使用 base 系姿态 RPY=(%.3f,%.3f,%.3f), 位置 pos=(%.3f,%.3f,%.3f)",
                    base_roll, base_pitch, base_yaw, p_base.x(), p_base.y(), p_base.z());
        return result;
    }

    // geometry_msgs::TransformStamped -> cv::Mat (4x4)
    cv::Mat tfToCvMat(const geometry_msgs::msg::TransformStamped &tf)
    {
        cv::Mat mat = cv::Mat::eye(4, 4, CV_64F);
        const auto &t = tf.transform.translation;
        const auto &q = tf.transform.rotation;

        // 四元数 -> 旋转矩阵
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 R(quat);
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                mat.at<double>(i, j) = R[i][j];
            }
        }
        mat.at<double>(0, 3) = t.x;
        mat.at<double>(1, 3) = t.y;
        mat.at<double>(2, 3) = t.z;
        return mat;
    }

    Eigen::Isometry3f tfToI3f(const geometry_msgs::msg::TransformStamped &tf)
    {
        const auto &t = tf.transform.translation;
        const auto &q = tf.transform.rotation;

        // 四元数 -> Eigen旋转矩阵
        Eigen::Quaternionf quat(
            static_cast<float>(q.w),
            static_cast<float>(q.x),
            static_cast<float>(q.y),
            static_cast<float>(q.z));

        Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
        T.linear() = quat.toRotationMatrix();
        T.translation() = Eigen::Vector3f(
            static_cast<float>(t.x),
            static_cast<float>(t.y),
            static_cast<float>(t.z));

        return T;
    }

    cv::Vec3d fitPlaneRPY(std::vector<cv::Point3f> &pts,
                      float *out_rms = nullptr,
                      double sigma_thresh = 2.0,
                      int max_iters = 10,
                      cv::Vec3d *out_centroid = nullptr)
    {
        // --- Step 1: remove outliers ---
        removeOutliersByPlane(pts, sigma_thresh, max_iters);

        if (pts.size() < 3)
            throw std::runtime_error("Too few inlier points remain after outlier removal.");

        // --- Step 2: fit plane normal on clean inliers ---
        cv::Vec3d normal = fitPlaneNormalPCA(pts, out_rms, out_centroid);

        // --- Step 3: build fully-defined orientation frame ---
        // new_z = normal (toward camera, negative Z component)
        cv::Vec3d new_z = normal; // already normalized and sign-corrected in fitPlaneNormalPCA

        // Use world Y (down) = [0,1,0] as reference to build X axis.
        // Fall back to world X if normal is nearly parallel to world Y.
        cv::Vec3d ref_y(0.0, 1.0, 0.0);
        if (fabs(new_z.dot(ref_y)) > 0.99)
            ref_y = cv::Vec3d(1.0, 0.0, 0.0);

        // new_x = ref_y cross new_z  →  roughly rightward
        cv::Vec3d new_x = ref_y.cross(new_z);
        new_x /= cv::norm(new_x);

        // new_y = new_z cross new_x  →  roughly downward (right-hand rule)
        cv::Vec3d new_y = new_z.cross(new_x);
        new_y /= cv::norm(new_y);

        // --- Step 4: assemble rotation matrix R = [new_x | new_y | new_z] (columns) ---
        double rot_data[9] = {
            new_x[0], new_y[0], new_z[0],
            new_x[1], new_y[1], new_z[1],
            new_x[2], new_y[2], new_z[2]};
        cv::Mat R = cv::Mat(3, 3, CV_64F, rot_data).clone();

        // --- Step 5: extract ZYX Euler angles ---
        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        // R(2,0) = -sin(pitch)
        // R(2,1) =  cos(pitch)*sin(roll)
        // R(2,2) =  cos(pitch)*cos(roll)
        // R(1,0) =  cos(pitch)*sin(yaw)
        // R(0,0) =  cos(pitch)*cos(yaw)
        double sin_pitch = -R.at<double>(2, 0);
        sin_pitch = std::max(-1.0, std::min(1.0, sin_pitch)); // clamp for asin safety
        double pitch = asin(sin_pitch);

        double roll, yaw;
        if (fabs(cos(pitch)) > 1e-6)
        {
            roll = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            yaw = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        }
        else
        {
            // Gimbal lock: pitch ≈ ±90°, yaw absorbed into roll
            roll = atan2(-R.at<double>(0, 1), R.at<double>(1, 1));
            yaw = 0.0;
        }

        return cv::Vec3d(roll, pitch, yaw);
    }

    // ---------- 计算屏幕平面姿态 ----------
    // 预选画布上的点(4 角点 + 4 边中点 + 1 中心点),通过 H 重投影到 RGB 图像,
    // 结合深度得到相机坐标系 3D 点,在相机坐标系下用 PCA 拟合平面,得到法向量
    // normal 返回相机坐标系下指向屏幕前方(远离相机, Z>0)的单位法向量
    // center 返回相机坐标系下平面中心
    cv::Vec3d computeScreenPlanePose(const cv::Mat &H, const cv::Mat &depth_raw, cv::Vec3d &normal, cv::Vec3d &center)
    {
        const float W = static_cast<float>(config_.canvas_width);
        const float Hc = static_cast<float>(config_.canvas_height);

        // 预选画布点: 4 角点 + 4 边中点 + 1 中心点
        std::vector<cv::Point2f> canvas_probe_pts = {
            cv::Point2f(0, 0), cv::Point2f(W, 0),          // 左上、右上
            cv::Point2f(W, Hc), cv::Point2f(0, Hc),        // 右下、左下
            cv::Point2f(W * 0.5f, 0), cv::Point2f(W, Hc * 0.5f),   // 上边中、右边中
            cv::Point2f(W * 0.5f, Hc), cv::Point2f(0, Hc * 0.5f),  // 下边中、左边中
            cv::Point2f(W * 0.5f, Hc * 0.5f)               // 中心
        };

        // 重投影到 RGB 图像坐标
        std::vector<cv::Point2f> image_probe_pts;
        cv::perspectiveTransform(canvas_probe_pts, image_probe_pts, H);

        // 结合深度转换为相机坐标系 3D 点 (在相机坐标系下拟合平面)
        std::vector<cv::Point3f> camera_probe_pts =
            deprojectToCamera(image_probe_pts, depth_raw);
        if (camera_probe_pts.size() < 3)
        {
            RCLCPP_WARN(get_logger(), "屏幕平面拟合:有效 3D 点不足 (%zu/9),拟合失败。",
                        camera_probe_pts.size());
            return cv::Vec3d(0, M_PI/2.0f, 0);
        }
        float rms;
        cv::Vec3d plane_centroid(0.0, 0.0, 0.0);
        cv::Vec3d screen_rpy = fitPlaneRPY(camera_probe_pts, &rms, 2.0, 10, &plane_centroid);

        // 设置平面中心(相机坐标系下拟合平面的质心,用于路径点投影)
        center = plane_centroid;

        float roll  = screen_rpy[0];
        float pitch = screen_rpy[1];
        float yaw   = screen_rpy[2];

        Eigen::AngleAxisf Rz(yaw, Eigen::Vector3f::UnitZ());
        Eigen::AngleAxisf Ry(pitch, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf Rx(roll, Eigen::Vector3f::UnitX());

        Eigen::Matrix3f R = (Rz * Ry * Rx).toRotationMatrix();

        // 屏幕法向量（局部 z 轴）
        Eigen::Vector3f normal_ = R.col(2);

        // 法向量应指向屏幕前方(远离相机,与相机视线方向一致,即 z>0)
        // 相机坐标系: X右 Y下 Z前(朝向屏幕),屏幕在相机前方,
        // 屏幕法向量(屏幕平面的 +Z 方向)应与相机视线方向同向(z>0)
        if (normal_.z() < 0)
            normal_ = -normal_;
        normal[0] = normal_.x();
        normal[1] = normal_.y();
        normal[2] = normal_.z();
        return screen_rpy;
        /*
        // 离群点剔除 (基于平面距离的迭代 sigma-clipping)
        removeOutliersByPlane(camera_probe_pts, 2.0, 10);
        if (camera_probe_pts.size() < 3)
        {
            RCLCPP_WARN(get_logger(), "屏幕平面拟合:离群点剔除后点不足,拟合失败。");
            return false;
        }

        // PCA/SVD 拟合平面法向量 (相机坐标系)
        // 构造中心化数据矩阵
        cv::Vec3d c(0.0, 0.0, 0.0);
        for (const auto &p : camera_probe_pts)
        {
            c[0] += p.x; c[1] += p.y; c[2] += p.z;
        }
        c = c / static_cast<double>(camera_probe_pts.size());

        cv::Mat A(static_cast<int>(camera_probe_pts.size()), 3, CV_64F);
        for (size_t i = 0; i < camera_probe_pts.size(); ++i)
        {
            A.at<double>(static_cast<int>(i), 0) = camera_probe_pts[i].x - c[0];
            A.at<double>(static_cast<int>(i), 1) = camera_probe_pts[i].y - c[1];
            A.at<double>(static_cast<int>(i), 2) = camera_probe_pts[i].z - c[2];
        }
        cv::Mat svd_w, svd_u, svd_vt;
        cv::SVD::compute(A, svd_w, svd_u, svd_vt, cv::SVD::MODIFY_A);

        // 法向量 = Vt 最后一行 (最小奇异值对应的右奇异向量)
        cv::Vec3d n(svd_vt.at<double>(2, 0), svd_vt.at<double>(2, 1), svd_vt.at<double>(2, 2));
        double norm_len = cv::norm(n);
        if (norm_len < 1e-9)
        {
            RCLCPP_WARN(get_logger(), "屏幕平面拟合:法向量退化,拟合失败。");
            return false;
        }
        n = n / norm_len;  // 单位化

        // 调整法向量方向:使其指向屏幕前方(远离相机)
        // 相机视线方向 = 相机Z+ = (0,0,1),屏幕在相机前方
        // 法向量应与相机视线方向成锐角(指向屏幕前方)
        // 用平面中心 c 的方向(相机到屏幕中心)作为视线方向的鲁棒估计,
        // 避免垂直屏幕时 n[2]≈0 导致符号不稳定
        cv::Vec3d view_dir = c;
        double vl = cv::norm(view_dir);
        if (vl > 1e-9)
        {
            view_dir = view_dir / vl;
            if (n.dot(view_dir) < 0)
            {
                n = -n;
            }
        }
        else
        {
            if (n[2] < 0) n = -n;
        }
        RCLCPP_INFO(get_logger(),
            "屏幕平面法向量(相机系): n=[%.3f, %.3f, %.3f], center=[%.3f, %.3f, %.3f]",
            n[0], n[1], n[2], c[0], c[1], c[2]);

        normal = n;
        center = c;
        */
    }

    cv::Vec3f fitPlaneNormalPCA(std::vector<cv::Point3f> &pts, float *out_rms = nullptr,
                            cv::Vec3d *out_centroid = nullptr)
    {
        if (pts.size() < 3)
            throw std::runtime_error("At least 3 points are required to fit a plane.");

        // --- Compute centroid ---
        cv::Vec3d centroid(0.0, 0.0, 0.0);
        for (const auto &p : pts)
        {
            centroid[0] += p.x;
            centroid[1] += p.y;
            centroid[2] += p.z;
        }
        centroid /= static_cast<double>(pts.size());

        // 输出平面质心(用于路径点投影)
        if (out_centroid)
        {
            (*out_centroid)[0] = centroid[0];
            (*out_centroid)[1] = centroid[1];
            (*out_centroid)[2] = centroid[2];
        }

        // --- Build centered data matrix (CV_64F) ---
        cv::Mat A(static_cast<int>(pts.size()), 3, CV_64F);
        for (size_t i = 0; i < pts.size(); ++i)
        {
            A.at<double>(i, 0) = pts[i].x - centroid[0];
            A.at<double>(i, 1) = pts[i].y - centroid[1];
            A.at<double>(i, 2) = pts[i].z - centroid[2];
        }

        // --- SVD: normal = last row of Vt (smallest singular value) ---
        cv::Mat W, U, Vt;
        cv::SVD::compute(A, W, U, Vt, cv::SVD::MODIFY_A);

        cv::Vec3d normal(
            Vt.at<double>(2, 0),
            Vt.at<double>(2, 1),
            Vt.at<double>(2, 2));

        double n_norm = cv::norm(normal);
        if (n_norm > 1e-12)
            normal /= n_norm;
        else
            throw std::runtime_error("Degenerate case: near-zero normal vector.");

        // --- Optional RMS residual (all double, no type mismatch) ---
        if (out_rms)
        {
            double sum2 = 0.0;
            for (const auto &p : pts)
            {
                cv::Vec3d v(p.x - centroid[0], p.y - centroid[1], p.z - centroid[2]);
                double dist = std::abs(v.dot(normal));
                sum2 += dist * dist;
            }
            *out_rms = std::sqrt(sum2 / static_cast<double>(pts.size()));
        }
        // 法向量方向统一:指向屏幕前方(远离相机, z>0)
        // 相机坐标系 Z 轴朝向屏幕,屏幕在相机前方,法向量应与视线方向同向
        if (normal[2] < 0)
        {
            normal[0] = -normal[0];
            normal[1] = -normal[1];
            normal[2] = -normal[2];
        }
        return normal;
    }

    // ---------- 将 3D 点投影到拟合平面上 ----------
    // 平面由单位法向量 n 和平面上一点 C 定义
    // 投影公式: P_proj = P - ((P - C) · n) * n
    // 即把点沿法线方向移动到平面上,消除沿法线方向的深度噪声
    void projectPointsToPlane(std::vector<cv::Point3f> &pts,
                              const cv::Vec3d &normal,
                              const cv::Vec3d &center)
    {
        // 单位化法向量(防御性,确保投影距离正确)
        double n_len = cv::norm(normal);
        if (n_len < 1e-9)
        {
            RCLCPP_WARN(get_logger(), "projectPointsToPlane: 法向量退化(长度≈0),跳过投影。");
            return;
        }
        cv::Vec3d n = normal / n_len;

        for (auto &p : pts)
        {
            // 点到平面的有向距离 d = (P - C) · n
            double d = (p.x - center[0]) * n[0] +
                       (p.y - center[1]) * n[1] +
                       (p.z - center[2]) * n[2];
            // 沿 -n 方向移动 d,使点落到平面上
            p.x = static_cast<float>(p.x - d * n[0]);
            p.y = static_cast<float>(p.y - d * n[1]);
            p.z = static_cast<float>(p.z - d * n[2]);
        }
    }

    // ---------- 离群点剔除 (基于平面距离的迭代 sigma-clipping) ----------
    void removeOutliersByPlane(std::vector<cv::Point3f> &pts,
                           double sigma_thresh = 2.0,
                           int max_iters = 10)
    {
        if (pts.size() < 3)
            return;

        for (int iter = 0; iter < max_iters; ++iter)
        {
            // --- Fit current plane ---
            float rms;
            cv::Vec3d normal = fitPlaneNormalPCA(pts, &rms);

            // --- Compute centroid ---
            cv::Vec3d centroid(0.0, 0.0, 0.0);
            for (const auto &p : pts)
            {
                centroid[0] += p.x;
                centroid[1] += p.y;
                centroid[2] += p.z;
            }
            centroid /= static_cast<double>(pts.size());

            // --- Compute each point's distance to plane ---
            std::vector<double> dists(pts.size());
            for (size_t i = 0; i < pts.size(); ++i)
            {
                cv::Vec3d v(pts[i].x - centroid[0],
                            pts[i].y - centroid[1],
                            pts[i].z - centroid[2]);
                dists[i] = std::abs(v.dot(normal));
            }

            // --- Compute mean and stddev of distances ---
            double mean = 0.0;
            for (double d : dists)
                mean += d;
            mean /= static_cast<double>(dists.size());

            double var = 0.0;
            for (double d : dists)
                var += (d - mean) * (d - mean);
            var /= static_cast<double>(dists.size());
            double stddev = std::sqrt(var);

            // --- Reject points beyond threshold ---
            double threshold = mean + sigma_thresh * stddev;

            std::vector<cv::Point3f> inliers;
            inliers.reserve(pts.size());
            for (size_t i = 0; i < pts.size(); ++i)
            {
                if (dists[i] <= threshold)
                    inliers.push_back(pts[i]);
            }

            // --- Stop if nothing was removed ---
            if (inliers.size() == pts.size())
                break;

            // --- Safety: require at least 3 points to continue ---
            if (inliers.size() < 3)
            {
                // Reject went too far — keep last valid set and stop
                break;
            }

            pts = std::move(inliers);
        }
    }

    // ---------- 由相机坐标系下平面法向量计算末端 RPY (参考 fitPlaneRPY) ----------
    // flange 对应机械臂 endpose,屏幕的 RPY 即末端的 RPY
    // 法向量 normal 指向屏幕前方(远离相机,与视线方向一致)
    // 相机坐标系: X右, Y下, Z前(朝向屏幕)
    // 构建末端坐标系(使末端姿态接近 home pose: pitch≈π/2, 末端Z朝前):
    //   new_z = normal (末端 Z 轴指向屏幕前方,工具沿 +Z 点触屏幕)
    //   ref   = (1,0,0) 相机X正方向 (与 home pose 在相机系的 X 轴方向一致)
    //   new_x = ref × new_z
    //   new_y = new_z × new_x
    // 返回相机坐标系下的 RPY (roll, pitch, yaw),后续通过 transformPoseCameraToBase 变换
    cv::Vec3d normalToRPY(const cv::Vec3d &normal)
    {
        // 末端 Z 轴目标方向 = normal (指向屏幕前方,工具沿 +Z 点触屏幕)
        cv::Vec3d new_z = normal;
        double zl = cv::norm(new_z);
        if (zl < 1e-9)
        {
            return cv::Vec3d(0.0, M_PI, 0.0);  // 回退朝下
        }
        new_z = new_z / zl;

        // 参考轴:相机坐标系 X 轴 (1,0,0) 向右;若法向量与 X 近似平行则改用 Y
        cv::Vec3d ref(1.0, 0.0, 0.0);
        if (std::abs(new_z.dot(ref)) > 0.99)
        {
            ref = cv::Vec3d(0.0, 1.0, 0.0);
        }

        // new_x = ref × new_z
        cv::Vec3d new_x = ref.cross(new_z);
        new_x = new_x / cv::norm(new_x);
        // new_y = new_z × new_x (右手定则)
        cv::Vec3d new_y = new_z.cross(new_x);
        new_y = new_y / cv::norm(new_y);

        // 构造旋转矩阵 R = [new_x | new_y | new_z] (列向量)
        Eigen::Matrix3d R;
        R << new_x[0], new_y[0], new_z[0],
             new_x[1], new_y[1], new_z[1],
             new_x[2], new_y[2], new_z[2];

        // 提取 ZYX 欧拉角: R = Rz(yaw)*Ry(pitch)*Rx(roll)
        // eulerAngles(2,1,0) 返回 (yaw, pitch, roll)
        Eigen::Vector3d ypr = R.eulerAngles(2, 1, 0);
        double roll = ypr.z();
        double pitch = ypr.y();
        double yaw = ypr.x();

        RCLCPP_INFO(get_logger(),
            "normalToRPY(相机系): normal=[%.3f,%.3f,%.3f] new_z=[%.3f,%.3f,%.3f] -> RPY(%.3f,%.3f,%.3f)",
            normal[0], normal[1], normal[2],
            new_z[0], new_z[1], new_z[2],
            roll, pitch, yaw);
        return cv::Vec3d(roll, pitch, yaw);
    }

    // ---------- 清除屏幕上原有轨迹 ----------
    void clearScreenTrajectory()
    {
        RCLCPP_INFO(get_logger(), "清除屏幕上原有轨迹(发布空 TrackCmd 通知)。");
        // 发布空 TrackCmd 通知外部清除屏幕
        // 实际屏幕擦除应由外部设备完成,这里仅做日志提示
        // 若需要机械臂回 home 再绘制,可在此发布 home pose
    }

    // ---------- 回调函数 ----------
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        try
        {
            if (msg->encoding == "rgb8")
            {
                cv::cvtColor(cv_bridge_to_mat(msg), rgb_image_, cv::COLOR_RGB2BGR);
            }
            else if (msg->encoding == "bgr8")
            {
                rgb_image_ = cv_bridge_to_mat(msg).clone();
            }
            else
            {
                rgb_image_ = cv_bridge_to_mat(msg).clone();
            }
            has_rgb_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "RGB 图像转换失败: %s", e.what());
        }
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        try
        {
            if (msg->encoding == "16UC1")
            {
                depth_image_ = cv_bridge_to_mat(msg).clone();
            }
            else
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                                     "深度图编码 %s 不支持,期望 16UC1", msg->encoding.c_str());
                return;
            }
            has_depth_ = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(get_logger(), "深度图像转换失败: %s", e.what());
        }
    }

    // 简易 cv_bridge 替代:直接从 Image 消息构造 cv::Mat
    cv::Mat cv_bridge_to_mat(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        int type = 0;
        if (msg->encoding == "rgb8" || msg->encoding == "bgr8")
        {
            type = CV_8UC3;
        }
        else if (msg->encoding == "mono8")
        {
            type = CV_8UC1;
        }
        else if (msg->encoding == "16UC1")
        {
            type = CV_16UC1;
        }
        else if (msg->encoding == "32FC1")
        {
            type = CV_32FC1;
        }
        else
        {
            // 默认按 8UC3 处理
            type = CV_8UC3;
        }
        return cv::Mat(static_cast<int>(msg->height), static_cast<int>(msg->width),
                       type, const_cast<uint8_t *>(msg->data.data()), msg->step);
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (has_camera_info_) return;

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
        for (size_t i = 0; i < std::min(msg->d.size(), size_t(5)); ++i)
        {
            dist_coeffs_.at<double>(0, i) = msg->d[i];
        }

        has_camera_info_ = true;
        RCLCPP_INFO(get_logger(), "相机内参获取完成: fx=%.2f fy=%.2f cx=%.2f cy=%.2f",
                    camera_matrix_.at<double>(0, 0), camera_matrix_.at<double>(1, 1),
                    camera_matrix_.at<double>(0, 2), camera_matrix_.at<double>(1, 2));
    }

    void armStatusCallback(const piper_msgs::msg::PiperStatusMsg::SharedPtr msg)
    {
        last_arm_status_ = *msg;
        has_arm_status_ = true;
    }

    // ---------- 成员变量 ----------
    TrajectoryDrawConfig config_;
    std::string config_dir_;

    // 画布
    cv::Mat canvas_;
    std::mutex canvas_mutex_;
    std::vector<cv::Point> current_stroke_;
    cv::Point last_point_;
    bool drawing_ = false;
    // 全屏窗口实际尺寸(屏幕分辨率),用于缩放画布与映射鼠标坐标
    int screen_width_ = 1280;
    int screen_height_ = 720;

    // 图像
    cv::Mat rgb_image_;
    cv::Mat depth_image_;
    std::mutex image_mutex_;
    bool has_rgb_ = false;
    bool has_depth_ = false;
    bool has_camera_info_ = false;

    // 相机参数
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // 手眼标定
    cv::Mat hand_eye_transform_;
    bool has_hand_eye_ = false;

    // 机械臂状态
    piper_msgs::msg::PiperStatusMsg last_arm_status_;
    bool has_arm_status_ = false;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<piper_msgs::msg::PiperStatusMsg>::SharedPtr arm_status_sub_;
    rclcpp::Publisher<demo_msgs::msg::TrackCmd>::SharedPtr track_cmd_pub_;
    rclcpp::Publisher<demo_msgs::msg::PoseGoal>::SharedPtr target_pose_pub_;
    rclcpp::Client<demo_msgs::srv::Get3DPoints>::SharedPtr get_3d_points_client_;

    std::atomic<bool> running_{true};

    // 调参滑动条值 (OpenCV trackbar 直接读写这些变量)
    int tb_canny_low_ = 50;
    int tb_canny_high_ = 150;
    int tb_morph_kernel_ = 5;
    int tb_area_min_ = 5;     // 实际值 = tb_area_min_ / 100
    int tb_area_max_ = 95;    // 实际值 = tb_area_max_ / 100
    int tb_quad_eps_ = 20;    // 实际值 = tb_quad_eps_ / 1000
};

// ============================================================
// main
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryDrawNode>();

    // ROS spin 在独立线程执行,保证图像回调及时触发
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });

    // 主循环:画布显示与键盘输入
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
    return 0;
}
