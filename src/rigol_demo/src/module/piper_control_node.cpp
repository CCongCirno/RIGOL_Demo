#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>

#include "demo_msgs/msg/target_pose.hpp"
#include "demo_msgs/msg/track_cmd.hpp"
#include "demo_msgs/msg/pose_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "piper_msgs/msg/piper_status_msg.hpp"
#include "piper_msgs/msg/pos_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class PiperControlNode : public rclcpp::Node
{
public:
	PiperControlNode()
	: Node("piper_control_node")
	{
		// 发布 PoseGoal 到 /target_pose,由 moveit_pose_goal_node 执行 MoveIt 规划
		// planner_mode 由 publishCurrentTarget 根据是否为 home pose 动态设置:
		//   home pose(回初始 / 从初始出发)→ 0(OMPL 自由路径)
		//   用户目标点之间 → 1(Pilz LIN 直线)
		target_pose_pub_ = create_publisher<demo_msgs::msg::PoseGoal>("/target_pose", 10);
		status_sub_ = create_subscription<piper_msgs::msg::PiperStatusMsg>(
			"/arm_status", 10,
			std::bind(&PiperControlNode::statusCallback, this, std::placeholders::_1));
		track_cmd_sub_ = create_subscription<demo_msgs::msg::TrackCmd>(
			"/track_cmd", 10,
			std::bind(&PiperControlNode::trackCmdCallback, this, std::placeholders::_1));

		command_period_ = declare_parameter<double>("command_period", 0.1);
		move_timeout_ = declare_parameter<double>("move_timeout", 10.0);
		settle_time_ = declare_parameter<double>("settle_time", 0.5);
		arrival_ignore_time_ = declare_parameter<double>("arrival_ignore_time", 0.3);
		gripper_ = declare_parameter<double>("gripper", 0.0);
		mode1_ = declare_parameter<int>("mode1", 0);
		mode2_ = declare_parameter<int>("mode2", 0);
		// 目标位姿参考坐标系(默认 base_link)
		target_frame_ = declare_parameter<std::string>("target_frame", "base_link");

		command_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<double>(command_period_)),
			std::bind(&PiperControlNode::controlLoop, this));

		RCLCPP_INFO(get_logger(), "Resetting Piper to home pose on startup.");
		enqueueHomePose();
	}

private:
	enum class MotionState
	{
		Idle,
		Moving,
		Settling,
	};

	static demo_msgs::msg::TargetPose homePose()
	{
		demo_msgs::msg::TargetPose pose;
		pose.x = 0.056;
		pose.y = 0.000;
		pose.z = 0.237;
		pose.roll = 0.00;
		pose.pitch = 1.571;
		pose.yaw = 0.00;
		return pose;
	}

	void statusCallback(const piper_msgs::msg::PiperStatusMsg::SharedPtr msg)
	{
		last_status_ = *msg;
		has_status_ = true;

		if (msg->arm_status != 0x00) {
			RCLCPP_WARN_THROTTLE(
				get_logger(), *get_clock(), 2000,
				"Piper arm_status is abnormal: %u, err_code: %ld",
				msg->arm_status, msg->err_code);

			if (shouldAbortCurrentTrack(msg->arm_status)) {
				abortCurrentTrackAndReturnHome(msg->arm_status);
			}
		}
	}

	void trackCmdCallback(const demo_msgs::msg::TrackCmd::SharedPtr msg)
	{
		if (msg->poses.empty()) {
			RCLCPP_WARN(get_logger(), "Received empty /track_cmd, ignoring.");
			return;
		}

		target_queue_.clear();
		for (const auto & pose : msg->poses) {
			target_queue_.push_back(pose);
		}
		target_queue_.push_back(homePose());

		returning_home_after_abort_ = false;
		motion_state_ = MotionState::Idle;
		RCLCPP_INFO(
			get_logger(),
			"Received %zu target poses, home pose appended for return motion.",
			msg->poses.size());
	}

	void controlLoop()
	{
		if (has_status_ && last_status_.arm_status != 0x00) {
			if (shouldAbortCurrentTrack(last_status_.arm_status)) {
				abortCurrentTrackAndReturnHome(last_status_.arm_status);
			} else {
				RCLCPP_WARN_THROTTLE(
					get_logger(), *get_clock(), 2000,
					"Skip command publishing because arm_status is abnormal: %u",
					last_status_.arm_status);
				return;
			}
		}

		switch (motion_state_) {
			case MotionState::Idle:
				startNextTarget();
				break;
			case MotionState::Moving:
				publishCurrentTarget();
				if (has_status_ && last_status_.motion_status == 0x01) {
					saw_motion_in_progress_ = true;
				}
				if (targetReached()) {
					if (returning_home_after_abort_ && isHomePose(current_target_)) {
						returning_home_after_abort_ = false;
					}
					motion_state_ = MotionState::Settling;
					settle_started_at_ = now();
				} else if (targetTimedOut()) {
					if (isHomePose(current_target_)) {
						RCLCPP_ERROR(get_logger(), "Home return timed out. Stop current command queue.");
						target_queue_.clear();
						returning_home_after_abort_ = false;
						motion_state_ = MotionState::Idle;
					} else {
						abortCurrentTrackAndReturnHome(0xff);
					}
				}
				break;
			case MotionState::Settling:
				if ((now() - settle_started_at_).seconds() >= settle_time_) {
					motion_state_ = MotionState::Idle;
				}
				break;
		}
	}

	void startNextTarget()
	{
		if (target_queue_.empty()) {
			return;
		}

		// 记录上一个已完成的目标,用于判断 home↔目标点 的运动段
		if (motion_state_ == MotionState::Idle && has_status_) {
			previous_target_ = current_target_;
			has_previous_target_ = true;
		}

		current_target_ = target_queue_.front();
		target_queue_.pop_front();
		target_started_at_ = now();
		saw_motion_in_progress_ = false;
		motion_state_ = MotionState::Moving;
		publishCurrentTarget();

		RCLCPP_INFO(
			get_logger(),
			"Moving to pose: x=%.3f y=%.3f z=%.3f roll=%.3f pitch=%.3f yaw=%.3f, remaining=%zu",
			current_target_.x, current_target_.y, current_target_.z,
			current_target_.roll, current_target_.pitch, current_target_.yaw,
			target_queue_.size());
	}

	bool targetReached() const
	{
		const bool ignore_stale_arrival = (now() - target_started_at_).seconds() < arrival_ignore_time_;
		return has_status_ && last_status_.motion_status == 0x00 &&
					 (saw_motion_in_progress_ || !ignore_stale_arrival);
	}

	bool targetTimedOut() const
	{
		const bool timed_out = (now() - target_started_at_).seconds() >= move_timeout_;
		if (timed_out) {
			RCLCPP_WARN(
				get_logger(),
				"Target motion timed out after %.1f seconds.",
				move_timeout_);
		}
		return timed_out;
	}

	bool shouldAbortCurrentTrack(const uint8_t arm_status) const
	{
		return arm_status == 0x02 || arm_status == 0x03 || arm_status == 0x04;
	}

	bool isHomePose(const demo_msgs::msg::TargetPose & pose) const
	{
		const auto home = homePose();
		constexpr double epsilon = 1e-6;
		return std::abs(pose.x - home.x) < epsilon &&
					 std::abs(pose.y - home.y) < epsilon &&
					 std::abs(pose.z - home.z) < epsilon &&
					 std::abs(pose.roll - home.roll) < epsilon &&
					 std::abs(pose.pitch - home.pitch) < epsilon &&
					 std::abs(pose.yaw - home.yaw) < epsilon;
	}

	void abortCurrentTrackAndReturnHome(const uint8_t arm_status)
	{
		if (returning_home_after_abort_) {
			return;
		}

		target_queue_.clear();
		if (!isHomePose(current_target_)) {
			target_queue_.push_back(homePose());
		}
		returning_home_after_abort_ = true;
		motion_state_ = MotionState::Idle;
		saw_motion_in_progress_ = false;

		RCLCPP_WARN(
			get_logger(),
			"Aborting current track because arm_status=%u. Returning to home pose.",
			arm_status);
	}

	void publishCurrentTarget()
	{
		// 将 TargetPose(欧拉角 roll/pitch/yaw)转换为 PoseGoal(含四元数 + 规划模式)
		// 发布到 /target_pose,由 moveit_pose_goal_node 调用 MoveIt 进行规划执行
		//
		// 规划模式选择(基于"当前目标"与"上一个目标"是否为 home):
		//   - home → 目标1(上一个为 home)        → OMPL 自由路径(mode=0)
		//   - 目标1 → 目标2(都不是 home)         → Pilz LIN 直线(mode=1)
		//   - 目标N → home(当前为 home)          → OMPL 自由路径(mode=0)
		//   - 启动 → home(无上一个目标)          → OMPL 自由路径(mode=0)
		demo_msgs::msg::PoseGoal goal_msg;
		goal_msg.pose.header.stamp = now();
		goal_msg.pose.header.frame_id = target_frame_;
		goal_msg.pose.pose.position.x = current_target_.x;
		goal_msg.pose.pose.position.y = current_target_.y;
		goal_msg.pose.pose.position.z = current_target_.z;

		// 欧拉角(RPY,弧度)转四元数
		tf2::Quaternion quat;
		quat.setRPY(current_target_.roll, current_target_.pitch, current_target_.yaw);
		goal_msg.pose.pose.orientation.x = quat.x();
		goal_msg.pose.pose.orientation.y = quat.y();
		goal_msg.pose.pose.orientation.z = quat.z();
		goal_msg.pose.pose.orientation.w = quat.w();

		// 仅当当前目标和上一个目标都不是 home 时,才使用直线规划
		const bool current_is_home = isHomePose(current_target_);
		const bool prev_is_home = has_previous_target_ && isHomePose(previous_target_);
		const bool use_linear = !current_is_home && !prev_is_home;
		goal_msg.planner_mode = use_linear ? 1 : 0;

		target_pose_pub_->publish(goal_msg);
	}

	void enqueueHomePose()
	{
		target_queue_.push_back(homePose());
		returning_home_after_abort_ = false;
		motion_state_ = MotionState::Idle;
	}

	rclcpp::Publisher<demo_msgs::msg::PoseGoal>::SharedPtr target_pose_pub_;
	rclcpp::Subscription<piper_msgs::msg::PiperStatusMsg>::SharedPtr status_sub_;
	rclcpp::Subscription<demo_msgs::msg::TrackCmd>::SharedPtr track_cmd_sub_;
	rclcpp::TimerBase::SharedPtr command_timer_;

	std::deque<demo_msgs::msg::TargetPose> target_queue_;
	demo_msgs::msg::TargetPose current_target_;
	demo_msgs::msg::TargetPose previous_target_;
	bool has_previous_target_ = false;
	piper_msgs::msg::PiperStatusMsg last_status_;

	MotionState motion_state_ = MotionState::Idle;
	rclcpp::Time target_started_at_;
	rclcpp::Time settle_started_at_;
	bool has_status_ = false;

	double command_period_ = 0.1;
	double move_timeout_ = 10.0;
	double settle_time_ = 0.5;
	double arrival_ignore_time_ = 0.3;
	double gripper_ = 0.0;
	int mode1_ = 0;
	int mode2_ = 0;
	std::string target_frame_ = "base_link";
	bool saw_motion_in_progress_ = false;
	bool returning_home_after_abort_ = false;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PiperControlNode>());
	rclcpp::shutdown();
	return 0;
}
