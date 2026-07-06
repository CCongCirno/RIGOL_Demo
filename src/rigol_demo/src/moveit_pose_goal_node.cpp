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
#include "moveit/move_group_interface/move_group_interface.h"

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

        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        pose_topic_ = this->get_parameter("pose_topic").as_string();
        planning_time_ = this->get_parameter("planning_time").as_double();
        goal_position_tolerance_ = this->get_parameter("goal_position_tolerance").as_double();
        goal_orientation_tolerance_ = this->get_parameter("goal_orientation_tolerance").as_double();
        plan_attempts_ = this->get_parameter("plan_attempts").as_int();
        max_velocity_scaling_factor_ = this->get_parameter("max_velocity_scaling_factor").as_double();
        max_acceleration_scaling_factor_ = this->get_parameter("max_acceleration_scaling_factor").as_double();

        // 订阅目标位姿话题
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
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

        RCLCPP_INFO(
            this->get_logger(),
            "MoveGroupInterface 初始化完成 | planning frame: %s | end-effector: %s | reference frame: %s",
            move_group_->getPlanningFrame().c_str(),
            move_group_->getEndEffectorLink().c_str(),
            move_group_->getPoseReferenceFrame().c_str());
    }

private:
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            target_pose_ = *msg;
            if (target_pose_.header.frame_id.empty())
            {
                target_pose_.header.frame_id = "base_link";
            }
            has_new_goal_ = true;
        }
        cv_.notify_one();
        RCLCPP_INFO(
            this->get_logger(),
            "收到目标位姿 | frame=%s | pos=(%.3f, %.3f, %.3f) | quat=(%.3f, %.3f, %.3f, %.3f)",
            target_pose_.header.frame_id.c_str(),
            target_pose_.pose.position.x, target_pose_.pose.position.y, target_pose_.pose.position.z,
            target_pose_.pose.orientation.x, target_pose_.pose.orientation.y,
            target_pose_.pose.orientation.z, target_pose_.pose.orientation.w);
    }

    // 规划执行循环:从队列取出最新目标并执行
    void planningLoop()
    {
        while (running_)
        {
            geometry_msgs::msg::PoseStamped goal;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                cv_.wait(lock, [this] { return has_new_goal_ || !running_; });
                if (!running_)
                {
                    break;
                }
                goal = target_pose_;
                has_new_goal_ = false;
            }

            executeGoal(goal);
        }
    }

    // 对单个目标位姿进行 plan & execute
    void executeGoal(const geometry_msgs::msg::PoseStamped& goal)
    {
        if (!move_group_)
        {
            RCLCPP_ERROR(this->get_logger(), "MoveGroupInterface 尚未初始化,无法执行");
            return;
        }

        // 设置位姿参考坐标系(若为空则使用默认 planning frame)
        if (!goal.header.frame_id.empty())
        {
            move_group_->setPoseReferenceFrame(goal.header.frame_id);
        }

        move_group_->setPoseTarget(goal.pose);

        // 规划
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto start_time = this->now();
        moveit::core::MoveItErrorCode plan_result = move_group_->plan(plan);
        double plan_duration = (this->now() - start_time).seconds();

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "规划失败 (错误码: %d),耗时 %.3f s", static_cast<int>(plan_result.val), plan_duration);
            move_group_->clearPoseTargets();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "规划成功,耗时 %.3f s,轨迹点数: %zu,开始执行...",
            plan_duration, plan.trajectory_.joint_trajectory.points.size());

        // 执行
        moveit::core::MoveItErrorCode exec_result = move_group_->execute(plan);
        if (exec_result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "执行完成 ✓");
        }
        else
        {
            RCLCPP_ERROR(
                this->get_logger(),
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

    // ROS 接口
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

    // 规划线程同步
    std::thread planning_thread_;
    std::mutex mutex_;
    std::condition_variable cv_;
    std::atomic<bool> running_;
    bool has_new_goal_;
    geometry_msgs::msg::PoseStamped target_pose_;
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
