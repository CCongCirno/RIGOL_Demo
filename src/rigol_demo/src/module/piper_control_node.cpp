#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "demo_msgs/msg/target_pose.hpp"
#include "demo_msgs/msg/track_cmd.hpp"
#include "demo_msgs/msg/pose_goal.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
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
		// 订阅末端实际位姿(由 piper_ctrl_single_node 发布),用于位置到位判定
		// 防止 OMPL 规划期间 motion_status==0x00 导致误判完成
		end_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
			"/end_pose_stamped", 10,
			std::bind(&PiperControlNode::endPoseCallback, this, std::placeholders::_1));

		command_period_ = declare_parameter<double>("command_period", 0.1);
		move_timeout_ = declare_parameter<double>("move_timeout", 10.0);
		settle_time_ = declare_parameter<double>("settle_time", 0.5);
		arrival_ignore_time_ = declare_parameter<double>("arrival_ignore_time", 0.3);
		gripper_ = declare_parameter<double>("gripper", 0.0);
		mode1_ = declare_parameter<int>("mode1", 0);
		mode2_ = declare_parameter<int>("mode2", 0);
		// 目标位姿参考坐标系(默认 base_link)
		target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
		// 批量模式:把整条轨迹合并为一次笛卡尔轨迹下发,避免逐点规划造成的卡顿
		// - batch_mode=true: home→轨迹起点(OMPL) → 整条轨迹一次 LIN 批量执行 → home(OMPL)
		// - batch_mode=false: 逐点下发(旧行为,每个点单独规划+settle)
		batch_mode_ = declare_parameter<bool>("batch_mode", true);
		// 批量执行超时(秒):整条轨迹执行的最大允许时间,防止卡死
		batch_move_timeout_ = declare_parameter<double>("batch_move_timeout", 60.0);

		command_timer_ = create_wall_timer(
			std::chrono::duration_cast<std::chrono::nanoseconds>(
				std::chrono::duration<double>(command_period_)),
			std::bind(&PiperControlNode::controlLoop, this));

		// 延迟下发 home pose:启动时 moveit_pose_goal_node 的 MoveGroupInterface
		// 可能尚未初始化完成,立即下发 home 会被丢弃 → 10s 超时。
		// 用一个定时器延迟 3s 后再下发,确保 moveit 节点就绪。
		home_delay_timer_ = create_wall_timer(
			std::chrono::seconds(3),
			[this]() {
				RCLCPP_INFO(get_logger(), "MoveGroup should be ready now, resetting Piper to home pose.");
				enqueueHomePose();
				home_delay_timer_->cancel();  // 只执行一次
			});

		RCLCPP_INFO(get_logger(), "Piper control node started, will reset to home pose in 3 seconds.");
	}

private:
	enum class MotionState
	{
		Idle,
		Moving,
		Settling,
	};

	// 批量轨迹执行阶段(仅 batch_mode=true 时使用)
	// None      : 未在批量阶段(或已完成/已中止)
	// Executing : 批量轨迹已下发,正在等待 moveit 执行完成
	// Done      : 批量执行完成,准备回 home
	enum class BatchPhase
	{
		None,
		Executing,
		Done,
	};

	static demo_msgs::msg::TargetPose homePose()
	{
		demo_msgs::msg::TargetPose pose;
		pose.x = 0.047635;
		pose.y = 0.000;
		pose.z = 0.25;
		pose.roll = 0.00;
		pose.pitch = 1.57079;
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

	// 末端位姿回调:记录最新末端位姿(用于位置到位判定)
	void endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(end_pose_mutex_);
		last_end_pose_ = *msg;
		has_end_pose_ = true;
	}

	void trackCmdCallback(const demo_msgs::msg::TrackCmd::SharedPtr msg)
	{
		if (msg->poses.empty()) {
			RCLCPP_WARN(get_logger(), "Received empty /track_cmd, ignoring.");
			return;
		}

		// 保存原始轨迹(用于批量模式构建 waypoints,以及非批量模式逐点执行)
		latest_track_poses_.clear();
		for (const auto & pose : msg->poses) {
			latest_track_poses_.push_back(pose);
		}

		target_queue_.clear();
		returning_home_after_abort_ = false;
		motion_state_ = MotionState::Idle;
		batch_phase_ = BatchPhase::None;
		batch_waypoints_sent_ = false;

		if (batch_mode_ && latest_track_poses_.size() >= 3) {
			// 批量模式(含过渡点):
			//   TrackCmd 结构 = [起点过渡点] + [路径点...] + [终点过渡点]
			//   执行流程:
			//     home → 起点过渡点(OMPL 自由规划)
			//          → 起点过渡点→路径点1..N(笛卡尔直线,一次性批量规划)
			//          → 路径点N→终点过渡点(笛卡尔直线,单独规划)
			//          → 终点过渡点→home(OMPL 自由规划)
			//   队列只放 home 和起点过渡点,中间段路径点由 batch_phase_ 触发批量下发,
			//   终点过渡点在批量段完成后单独入队。
			target_queue_.push_back(homePose());                          // 1. 先回 home
			target_queue_.push_back(latest_track_poses_.front());         // 2. 到起点过渡点(OMPL)
			RCLCPP_INFO(
				get_logger(),
				"批量模式(含过渡点): 收到 %zu 个点(含首尾过渡点), home→起点过渡(OMPL)→路径点段批量直线→终点过渡(直线)→home。",
				latest_track_poses_.size());
		} else if (batch_mode_ && latest_track_poses_.size() >= 2) {
			// 批量模式(无过渡点,兼容旧版): home → 轨迹起点(OMPL) → 整条轨迹(批量直线) → home(OMPL)
			target_queue_.push_back(homePose());
			target_queue_.push_back(latest_track_poses_.front());
			RCLCPP_INFO(
				get_logger(),
				"批量模式: 收到 %zu 个轨迹点, 将合并为一次笛卡尔轨迹执行(无逐点停顿)。",
				latest_track_poses_.size());
		} else {
			// 非批量模式:逐点入队,每个点单独规划+settle(旧行为)
			for (const auto & pose : latest_track_poses_) {
				target_queue_.push_back(pose);
			}
			target_queue_.push_back(homePose());
			RCLCPP_INFO(
				get_logger(),
				"非批量模式: 收到 %zu 个轨迹点, 逐点下发执行。",
				latest_track_poses_.size());
		}
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

		// 批量执行阶段:整条轨迹已作为一条笛卡尔轨迹下发,等待执行完成
		// 注意:批量目标只在 publishBatchTrack 中发送一次,此处绝不重发!
		//   重发会触发 moveit 节点 poseCallback 置 has_new_goal_=true,
		//   当前 executeGoal 返回后 planningLoop 取出暂存目标再次执行 = 轨迹执行两次。
		//
		// 完成判定:必须 saw_motion_in_progress_==true(机械臂真正动过)后才允许判完成。
		//   不能用 !ignore_stale_arrival 兜底——批量轨迹下发后 moveit 还在规划/execute action
		//   往返期间,机械臂 motion_status 仍是 0x00(没开始动),0.3s 后会误判完成。
		if (batch_phase_ == BatchPhase::Executing) {
			if (has_status_ && last_status_.motion_status == 0x01) {
				saw_motion_in_progress_ = true;
			}
			if (targetReached()) {
				RCLCPP_INFO(get_logger(), "批量轨迹执行完成 ✓,准备进入终点过渡点和返回 home。");
				batch_phase_ = BatchPhase::Done;
				batch_waypoints_sent_ = false;
				motion_state_ = MotionState::Settling;
				settle_started_at_ = now();
				// 先入队终点过渡点,Settling 结束后再继续执行终点过渡点 → home
				target_queue_.push_back(latest_track_poses_.back());
				target_queue_.push_back(homePose());
			} else if (targetTimedOut()) {
				RCLCPP_ERROR(get_logger(),
					"批量轨迹执行超时(%.1f s),中止并返回 home。", batch_move_timeout_);
				abortCurrentTrackAndReturnHome(0xff);
			}
			return;
		}

		switch (motion_state_) {
			case MotionState::Idle:
				startNextTarget();
				break;
			case MotionState::Moving:
				// 单点目标只在进入 Moving 时发布一次(startNextTarget 已发布),
				// 不再每周期重发——重发会导致 moveit 节点 has_new_goal_ 持续为 true,
				// executeGoal 返回后 planningLoop 取出暂存目标重复执行。
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
						// 彻底重置批量状态,避免队列空时误触发批量下发
						batch_phase_ = BatchPhase::None;
						batch_waypoints_sent_ = false;
						latest_track_poses_.clear();
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
		// 批量模式:home 和起点过渡点都已执行完,队列空 → 触发批量轨迹下发
		if (target_queue_.empty()) {
			if (batch_mode_ && batch_phase_ == BatchPhase::None && !batch_waypoints_sent_
					&& latest_track_poses_.size() >= 3) {
				publishBatchTrack();
				return;
			}
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

	// 批量下发整条轨迹:把 latest_track_poses_ 转为 Pose 数组,
	// 用 planner_mode=2 发布一次,moveit_pose_goal_node 会从当前位姿开始
	// 依次直线经过所有 waypoints(仅规划/执行一次,无逐点停顿)
	//
	// TrackCmd 结构 = [起点过渡点] + [路径点...] + [终点过渡点]
	// 起点过渡点已通过 OMPL 到达(当前位姿),批量直线从路径点1开始,
	// 依次经过路径点,到终点过渡点结束。
	// waypoints = [路径点1, ..., 路径点N, 终点过渡点](跳过起点过渡点)
	void publishBatchTrack()
	{
		if (latest_track_poses_.size() < 2) {
			RCLCPP_ERROR(get_logger(), "批量下发失败:轨迹点不足 %zu", latest_track_poses_.size());
			return;
		}

		demo_msgs::msg::PoseGoal goal_msg;
		goal_msg.pose.header.stamp = now();
		goal_msg.pose.header.frame_id = target_frame_;
		// pose 字段填第一个 waypoint(路径点1,便于 moveit 节点日志/参考)
		const auto & first_wp = latest_track_poses_[1];  // 跳过起点过渡点 [0]
		goal_msg.pose.pose.position.x = first_wp.x;
		goal_msg.pose.pose.position.y = first_wp.y;
		goal_msg.pose.pose.position.z = first_wp.z;
		tf2::Quaternion quat;
		quat.setRPY(first_wp.roll, first_wp.pitch, first_wp.yaw);
		goal_msg.pose.pose.orientation.x = quat.x();
		goal_msg.pose.pose.orientation.y = quat.y();
		goal_msg.pose.pose.orientation.z = quat.z();
		goal_msg.pose.pose.orientation.w = quat.w();

		// waypoints:跳过起点过渡点 [0](已通过 OMPL 到达,作为当前位姿/起点)
	// 依次经过 [路径点1, ..., 路径点N]
	const size_t num_waypoints = latest_track_poses_.size() - 2;  // 跳过起点过渡点和终点过渡点
	if (num_waypoints == 0) {
		RCLCPP_ERROR(get_logger(), "批量下发失败: 路径点不足, 需要至少 1 个路径点");
		return;
	}
	goal_msg.waypoints.reserve(num_waypoints);
	for (size_t i = 1; i + 1 < latest_track_poses_.size(); ++i) {
		const auto & tp = latest_track_poses_[i];
		geometry_msgs::msg::Pose p;
		p.position.x = tp.x;
		p.position.y = tp.y;
		p.position.z = tp.z;
		tf2::Quaternion q;
		q.setRPY(tp.roll, tp.pitch, tp.yaw);
		p.orientation.x = q.x();
		p.orientation.y = q.y();
		p.orientation.z = q.z();
		p.orientation.w = q.w();
		goal_msg.waypoints.push_back(p);
	}
	goal_msg.planner_mode = 2;  // 批量笛卡尔轨迹模式

		batch_phase_ = BatchPhase::Executing;
		batch_waypoints_sent_ = true;
		target_started_at_ = now();
		saw_motion_in_progress_ = false;
		// 用批量超时覆盖单点超时
		batch_deadline_ = now() + rclcpp::Duration::from_seconds(batch_move_timeout_);
		current_target_ = latest_track_poses_.back();  // 用于 publishCurrentTarget 持续发布
		motion_state_ = MotionState::Moving;

		target_pose_pub_->publish(goal_msg);
		RCLCPP_INFO(
			get_logger(),
			"批量下发轨迹: %zu 个 waypoints, planner_mode=2, 超时 %.1f s",
			goal_msg.waypoints.size(), batch_move_timeout_);
	}

	bool targetReached() const
	{
		// 完成判定:motion_status==0x00(到位) 且 超过 arrival_ignore_time_(过滤刚下发时的瞬时 0x00 残留)。
		//
		// 批量阶段额外要求 saw_motion_in_progress_==true:批量轨迹下发后 moveit 还在规划/
		// execute action 往返期间,机械臂 motion_status 仍是 0x00(没开始动),若仅靠
		// arrival_ignore_time_ 会在 0.3s 后误判完成 → 提前入队 home → 批量轨迹根本没执行。
		//
		// 单点目标(非批量)区分 home 和轨迹起点:
		//   - home pose:不要求 saw_motion_in_progress_(home 规划快 0.03s,0.3s 内开始执行,
		//     但 controlLoop 0.1s 周期可能错过 motion_status==0x01 瞬间 → 永远无法判定完成 → 超时)
		//   - 轨迹起点(非 home):要求 saw_motion_in_progress_(OMPL 规划期间最长 5s,
		//     motion_status==0x00 且 saw_motion_in_progress_==false → 不会误判完成。
		//     规划完成后机械臂运动,motion_status 变为 0x01 → saw_motion_in_progress_=true,
		//     运动到位 motion_status 回到 0x00 → 判定完成)
		const bool ignore_stale_arrival = (now() - target_started_at_).seconds() < arrival_ignore_time_;
		const bool base_reached = has_status_ && last_status_.motion_status == 0x00 && !ignore_stale_arrival;
		if (batch_phase_ == BatchPhase::Executing) {
			return base_reached && saw_motion_in_progress_;
		}
		// 单点目标:home 不要求 saw_motion_in_progress_,非 home 要求
		if (isHomePose(current_target_)) {
			return base_reached;
		}
		return base_reached && saw_motion_in_progress_;
	}

	bool targetTimedOut() const
	{
		// 批量执行阶段使用批量超时
		const double timeout = (batch_phase_ == BatchPhase::Executing)
			? batch_move_timeout_ : move_timeout_;
		const bool timed_out = (now() - target_started_at_).seconds() >= timeout;
		if (timed_out) {
			RCLCPP_WARN(
				get_logger(),
				"Target motion timed out after %.1f seconds.",
				timeout);
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
		// 重置批量状态,避免残留触发
		batch_phase_ = BatchPhase::None;
		batch_waypoints_sent_ = false;
		latest_track_poses_.clear();

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
		//   - 目标1 → 目标2(都不是 home)         → 单点 LIN(mode=1)
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
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pose_sub_;
	rclcpp::TimerBase::SharedPtr command_timer_;
	rclcpp::TimerBase::SharedPtr home_delay_timer_;  // 启动延迟下发 home

	std::deque<demo_msgs::msg::TargetPose> target_queue_;
	demo_msgs::msg::TargetPose current_target_;
	demo_msgs::msg::TargetPose previous_target_;
	bool has_previous_target_ = false;
	piper_msgs::msg::PiperStatusMsg last_status_;

	// 末端实际位姿(由 /end_pose_stamped 提供,用于位置到位判定)
	geometry_msgs::msg::PoseStamped last_end_pose_;
	mutable std::mutex end_pose_mutex_;
	bool has_end_pose_ = false;

	// 批量模式:保存最近一次 /track_cmd 的轨迹点,用于构建 waypoints
	std::vector<demo_msgs::msg::TargetPose> latest_track_poses_;
	BatchPhase batch_phase_ = BatchPhase::None;
	bool batch_waypoints_sent_ = false;
	rclcpp::Time batch_deadline_;

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
	bool batch_mode_ = true;
	double batch_move_timeout_ = 60.0;
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PiperControlNode>());
	rclcpp::shutdown();
	return 0;
}
