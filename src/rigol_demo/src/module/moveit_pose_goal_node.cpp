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
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
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
            "收到目标位姿 | frame=%s | mode=%s | pos=(%.3f, %.3f, %.3f) | quat=(%.3f, %.3f, %.3f, %.3f) | waypoints=%zu",
            target_goal_.pose.header.frame_id.c_str(),
            target_goal_.planner_mode == 2 ? "BATCH(批量直线)" :
            (target_goal_.planner_mode == 1 ? "LIN(直线)" : "OMPL(自由)"),
            target_goal_.pose.pose.position.x, target_goal_.pose.pose.position.y, target_goal_.pose.pose.position.z,
            target_goal_.pose.pose.orientation.x, target_goal_.pose.pose.orientation.y,
            target_goal_.pose.pose.orientation.z, target_goal_.pose.pose.orientation.w,
            target_goal_.waypoints.size());
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
        //   mode=0 → OMPL RRTConnect(自由路径,用于回 home / 从 home 出发)
        //   mode=1 → computeCartesianPath 单段直线(两点之间)
        //   mode=2 → computeCartesianPath 多段直线(整条轨迹,从当前位姿依次经过所有 waypoints)
        const bool use_linear = (goal.planner_mode == 1);
        const bool use_batch  = (goal.planner_mode == 2);

        auto start_time = this->now();
        moveit::core::MoveItErrorCode exec_result;

        if (use_batch)
        {
            // ===== 批量笛卡尔直线规划(整条轨迹一次执行)=====
            // 从当前位姿开始,依次直线经过 goal.waypoints 中的所有点。
            // 仅规划/执行一次,避免逐点规划造成的卡顿。
            move_group_->setStartStateToCurrentState();

            geometry_msgs::msg::Pose start_pose = move_group_->getCurrentPose().pose;
            std::vector<geometry_msgs::msg::Pose> waypoints;
            waypoints.reserve(goal.waypoints.size() + 1);
            waypoints.push_back(start_pose);  // 起点 = 当前位姿
            for (const auto & wp : goal.waypoints) {
                waypoints.push_back(wp);
            }

            moveit_msgs::msg::RobotTrajectory trajectory;
            const double eef_step = 0.005;          // 末端步长 5mm(批量轨迹更密集,保证平滑)
            const double jump_threshold = 0.0;     // 0 表示禁用跳变检测
            double fraction = move_group_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

            double plan_duration = (this->now() - start_time).seconds();

            if (fraction < 0.9)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "批量笛卡尔轨迹规划失败 (覆盖率: %.1f%% < 90%%),耗时 %.3f s, waypoints=%zu",
                    fraction * 100.0, plan_duration, goal.waypoints.size());
                move_group_->clearPoseTargets();
                return;
            }

            // 手动添加时间参数化(computeCartesianPath 不带时间)
            robot_trajectory::RobotTrajectory rt(move_group_->getRobotModel(), planning_group_);
            rt.setRobotTrajectoryMsg(*move_group_->getCurrentState(), trajectory);

            trajectory_processing::IterativeParabolicTimeParameterization iptp;
            bool time_param_ok = iptp.computeTimeStamps(rt, max_velocity_scaling_factor_, max_acceleration_scaling_factor_);
            if (!time_param_ok)
            {
                RCLCPP_WARN(this->get_logger(), "批量轨迹时间参数化失败,使用原始轨迹");
            }
            rt.getRobotTrajectoryMsg(trajectory);

            RCLCPP_INFO(this->get_logger(),
                "批量笛卡尔轨迹规划成功,覆盖率: %.1f%%,耗时 %.3f s,轨迹点数: %zu,waypoints: %zu,开始执行...",
                fraction * 100.0, plan_duration, trajectory.joint_trajectory.points.size(), goal.waypoints.size());

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            exec_result = move_group_->execute(plan);
        }
        else if (use_linear)
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

            // 判断是否为 home pose(home pose 固定不变,不加路径约束)
            // home pose: pos=(0.048, 0.000, 0.250), quat≈(0, 0.707, 0, 0.707)
            const auto & p = pose_stamped.pose.position;
            const auto & q = pose_stamped.pose.orientation;
            const bool is_home_pose =
                std::abs(p.x - 0.048) < 0.01 &&
                std::abs(p.y - 0.000) < 0.01 &&
                std::abs(p.z - 0.250) < 0.01 &&
                std::abs(q.x - 0.000) < 0.01 &&
                std::abs(q.y - 0.707) < 0.01 &&
                std::abs(q.z - 0.000) < 0.01 &&
                std::abs(q.w - 0.707) < 0.01;

            // 保存原始 tolerance,规划后恢复
            const double orig_orientation_tol = goal_orientation_tolerance_;

            if (!is_home_pose)
            {
                // 非 home(轨迹起点):允许末端绕 joint6 轴(末端工具 Z 轴)自由旋转,
                // 只要末端 Z 轴方向(垂直屏幕)不变即可。
                // 实现:设置路径约束(OrientationConstraint 锁定末端 Z 轴方向),
                //       同时放宽 goal_orientation_tolerance 到 π(允许绕 Z 轴任意旋转)。
                //       这样 OMPL 会在保持末端 Z 轴方向的前提下自由搜索绕 Z 轴的旋转,
                //       大幅增加可求解姿态,避免 "Unable to sample any valid states" 错误。
                moveit_msgs::msg::OrientationConstraint oc;
                oc.link_name = end_effector_link_;
                oc.header.frame_id = pose_stamped.header.frame_id;
                oc.orientation = pose_stamped.pose.orientation;  // 目标姿态(末端 Z 轴方向)
                // 仅约束 X/Y 轴方向(等价于锁定 Z 轴方向),允许绕 Z 轴自由旋转
                // absolute_x/y_tolerance 放宽到 0.2(~11.5°),允许起始状态与目标有偏差时仍可规划
                oc.absolute_x_axis_tolerance = 0.2;   // ~11.5° 锁定 X 轴方向
                oc.absolute_y_axis_tolerance = 0.2;   // ~11.5° 锁定 Y 轴方向
                oc.absolute_z_axis_tolerance = 2.0 * M_PI;  // 允许绕 Z 轴任意旋转
                oc.weight = 1.0;

                moveit_msgs::msg::Constraints path_constraints;
                path_constraints.orientation_constraints.push_back(oc);
                path_constraints.name = "keep_end_z_axis";
                move_group_->setPathConstraints(path_constraints);

                // 放宽目标姿态容差:允许绕 Z 轴任意旋转(只要末端 Z 轴方向正确)
                move_group_->setGoalOrientationTolerance(2.0 * M_PI);
                RCLCPP_INFO(this->get_logger(), "非 home 目标:启用末端 Z 轴方向约束 + 绕 Z 自由旋转");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "home pose:使用标准 OMPL 规划(无路径约束)");
            }

            move_group_->setPoseTarget(pose_stamped.pose);

            moveit::planning_interface::MoveGroupInterface::Plan plan;
            moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);
            double plan_duration = (this->now() - start_time).seconds();

            // 恢复原始 tolerance 和清除路径约束
            move_group_->setGoalOrientationTolerance(orig_orientation_tol);
            move_group_->clearPathConstraints();

            if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
            {
                RCLCPP_ERROR(this->get_logger(),
                    "自由(OMPL)规划失败 (错误码: %d),耗时 %.3f s",
                    static_cast<int>(plan_result.val), plan_duration);
                move_group_->clearPoseTargets();
                return;
            }

            RCLCPP_INFO(this->get_logger(),
                "自由(OMPL)规划成功%s,耗时 %.3f s,轨迹点数: %zu,开始执行...",
                is_home_pose ? "(home)" : "(末端 Z 轴方向约束 + 绕 Z 自由旋转)",
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
