// moveit_pose_goal_node.cpp
// 接收目标位姿(PoseStamped)后,调用 piper_no_gripper_moveit 的 move_group
// 进行 plan & execute 的节点
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "demo_msgs/msg/pose_goal.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

class MoveitPoseGoalNode : public rclcpp::Node
{
public:
    MoveitPoseGoalNode() : Node("moveit_pose_goal_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("planning_group", "arm");
        this->declare_parameter<std::string>("end_effector_link", "link6");
        this->declare_parameter<std::string>("pose_topic", "/target_pose");
        this->declare_parameter<double>("planning_time", 5.0);
        this->declare_parameter<double>("goal_position_tolerance", 0.001);
        this->declare_parameter<double>("goal_orientation_tolerance", 0.001);
        this->declare_parameter<int>("plan_attempts", 5);
        this->declare_parameter<double>("max_velocity_scaling_factor", 1.0);
        this->declare_parameter<double>("max_acceleration_scaling_factor", 1.0);
        // 直线运动的最大笛卡尔速度(目标点之间使用 Pilz LIN 时生效)
        this->declare_parameter<double>("cartesian_speed", 0.1);

        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();
        planning_time_ = this->get_parameter("planning_time").as_double();
        goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
        goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();
        plan_attempts_ = this->get_parameter("plan_attempts").as_int();
        max_velocity_scaling_factor_ = this->get_parameter("max_velocity_scaling_factor").as_double();
        max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();
        cartesian_speed_ = this->get_parameter("cartesian_speed").as_double();

        // 订阅目标位姿话题(PoseGoal:含位姿 + 规划模式)
        pose_sub_ = this->create_subscription<demo_msgs::msg::PoseGoal>(
            pose_topic_, 10,
            std::bind(&MoveitPoseGoalNode::poseCallback, this, std::placeholders::_1));

        has_new_goal_ = false;
        running_ = true;

        // 启动规划执行线程(避免阻塞 executor)
        planning_thread_ = std::thread(&MoveitPoseGoalNode::planningLoop, this);

        RCLCPP_INFO(
            this->get_logger(),
            "MoveitPoseGoalNode 已启动,等待目标位姿话题: %s (planning_group=%s, end_effector=%s)",
            pose_topic_.c_str(), planning_group_.c_str(), end_effector_link_.c_str());
    }

    ~MoveitPoseGoalNode() override
    {
        running_ = false;
        cv_.notify_all();
        if (planning_thread_.joinable())
        {
            planning_thread_.join();
        }
    }

    // 初始化 MoveGroupInterface(需要在 executor 开始 spin 之后调用,
    // 因为构造时需要与 move_group 节点建立 action client 连接)
    void initMoveGroup()
    {
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

        move_group_->setEndEffectorLink(end_effector_link_);
        move_group_->setPlanningTime(planning_time_);
        move_group_->setGoalPositionTolerance(goal_position_tolerance_);
        move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
        move_group_->setNumPlanningAttempts(plan_attempts_);
        move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor_);
        move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);

        // 默认使用 OMPL 自由路径规划;直线模式由每个目标的 planner_mode 动态切换
        move_group_->setPlanningPipelineId("ompl");
        move_group_->setPlannerId("RRTConnect");

        RCLCPP_INFO(this->get_logger(),
            "规划模式:按目标动态切换 | OMPL(自由路径, mode=0) / Pilz LIN(直线, mode=1) | "
            "cartesian_speed=%.3f m/s", cartesian_speed_);

        RCLCPP_INFO(
            this->get_logger(),
            "MoveGroupInterface 初始化完成 | planning frame: %s | end-effector: %s | reference frame: %s",
            move_group_->getPlanningFrame().c_str(),
            move_group_->getEndEffectorLink().c_str(),
            move_group_->getPoseReferenceFrame().c_str());
    }

private:
    void poseCallback(const demo_msgs::msg::PoseGoal::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_goal_ = *msg;
            if (target_goal_.pose.header.frame_id.empty())
            {
                target_goal_.pose.header.frame_id = "base_link";
            }
            has_new_goal_ = true;
        }
        cv_.notify_one();
        RCLCPP_INFO(
            this->get_logger(),
            "收到目标位姿 | frame=%s | mode=%s | pos=(%.3f, %.3f, %.3f) | quat=(%.3f, %.3f, %.3f, %.3f)",
            target_goal_.pose.header.frame_id.c_str(),
            target_goal_.planner_mode == 1 ? "LIN(直线)" : "OMPL(自由)",
            target_goal_.pose.pose.position.x, target_goal_.pose.pose.position.y, target_goal_.pose.pose.position.z,
            target_goal_.pose.pose.orientation.x, target_goal_.pose.pose.orientation.y,
            target_goal_.pose.pose.orientation.z, target_goal_.pose.pose.orientation.w);
    }

    // 规划执行循环:从队列取出最新目标并执行
    void planningLoop()
    {
        while (running_)
        {
            demo_msgs::msg::PoseGoal goal;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return has_new_goal_ || !running_; });
                if (!running_)
                {
                    break;
                }
                goal = target_goal_;
                has_new_goal_ = false;
            }

            executeGoal(goal);
        }
    }

    // 对单个目标位姿进行 plan & execute
    void executeGoal(const demo_msgs::msg::PoseGoal& goal)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 尚未初始化,无法执行");
            return;
        }

        const auto& pose_stamped = goal.pose;
        // 设置位姿参考坐标系(若为空则使用默认 planning frame)
        if (!pose_stamped.header.frame_id.empty())
        {
            move_group_->setPoseReferenceFrame(pose_stamped.header.frame_id);
        }

        // 根据目标的 planner_mode 动态切换规划方式
        //   mode=1 → computeCartesianPath(笛卡尔直线,用于目标点之间)
        //   mode=0 → OMPL RRTConnect(自由路径,用于回 home / 从 home 出发)
        const bool use_linear = (goal.planner_mode == 1);

        auto start_time = this->now();
        moveit::core::MoveItErrorCode exec_result;

        if (use_linear)
        {
            // ===== 笛卡尔直线规划(computeCartesianPath)=====
            // Pilz LIN 是完整规划管道;computeCartesianPath 是基于 IK 的路径采样,
            // 不做碰撞检测,需要手动添加时间参数化。
            // 这里用"当前位姿 → 目标位姿"两个点构造 waypoints。
            move_group_->setStartStateToCurrentState();

            geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.push_back(start_pose);
            waypoints.push_back(pose_stamped.pose);

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.01;          // 末端步长 1cm
            const double jump_threshold = 0.0;     // 0 表示禁用跳变检测
            double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            double plan_duration = (this->now() - start_time).seconds();

            if (fraction < 0.9)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "笛卡尔直线规划失败 (覆盖率: %.1f%% < 90%%),耗时 %.3f s",
                    fraction * 100.0, plan_duration);
                move_group_->clearPoseTargets();
                return;
            }

            // computeCartesianPath 不带时间参数,需要手动添加时间参数化
            // 使用 IterativeParabolicTimeParameterization 给轨迹加上速度/加速度/时间
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), planning_group_);
            rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool time_param_ok = iptp.computeTimeStamps(rt, max_velocity_scaling_factor_, max_acceleration_scaling_factor_);
            if (!time_param_ok)
            {
                RCLCPP_WARN(this->get_logger(), "笛卡尔轨迹时间参数化失败,使用原始轨迹");
            }
            rt.getRobotTrajectoryMsg(trajectory);

            RCLCPP_INFO(this->get_logger(),
                "笛卡尔直线规划成功,覆盖率: %.1f%%,耗时 %.3f s,轨迹点数: %zu,开始执行...",
                fraction * 100.0, plan_duration, trajectory.joint_trajectory.points.size());

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            exec_result = move_group_->execute(plan);
        }
        else
        {
            // ===== OMPL 自由路径规划 =====
            move_group_->setPlanningPipelineId("ompl");
            move_group_->setPlannerId("RRTConnect");
            move_group_->setPoseTarget(pose_stamped.pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);
            double plan_duration = (this->now() - start_time).seconds();

            if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "自由(OMPL)规划失败 (错误码: %d),耗时 %.3f s",
                    static_cast<int>(plan_result.val), plan_duration);
                move_group_->clearPoseTargets();
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                "自由(OMPL)规划成功,耗时 %.3f s,轨迹点数: %zu,开始执行...",
                plan_duration, plan.trajectory_.joint_trajectory.points.size());

            exec_result = move_group_->execute(plan);
        }

        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "执行完成 ✓");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(),
                "执行失败 (错误码: %d)", static_cast<int>(exec_result.val));
        }

        move_group_->clearPoseTargets();
    }

    // 参数
    std::string planning_group_;
    std::string end_effector_link_;
    std::string pose_topic_;
    double planning_time_;
    double goal_position_tolerance_;
    double goal_orientation_tolerance_;
    int plan_attempts_;
    double max_velocity_scaling_factor_;
    double max_acceleration_scaling_factor_;
    double cartesian_speed_ = 0.1;

    // ROS 接口
    rclcpp::Subscription<demo_msgs::msg::PoseGoal>::SharedPtr pose_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // 规划线程同步
    std::thread planning_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
    bool has_new_goal_;
    demo_msgs::msg::PoseGoal target_goal_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // 使用多线程执行器,保证 action client 回调能正常处理
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<MoveitPoseGoalNode>();
    executor.add_node(node);

    // 先启动 spin 线程,让节点就绪
    std::thread spin_thread([&executor]() { executor.spin(); });

    // 等待 move_group action server 就绪后初始化 MoveGroupInterface
    std::this_thread::sleep_for(std::chrono::seconds(1));
    node->initMoveGroup();

    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
