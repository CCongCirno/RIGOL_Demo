# 目标位姿 + 规划模式标志
# 由 piper_control_node 发布,moveit_pose_goal_node 订阅
geometry_msgs/PoseStamped pose

# 规划模式:
#   0 = 自由路径(OMPL RRTConnect),用于回 home / 从 home 出发
#   1 = 直线运动(单点 LIN),用于目标点之间的运动
#   2 = 多段直线运动(批量轨迹),waypoints 为完整轨迹点序列,
#       moveit_pose_goal_node 会从当前位姿开始依次直线经过所有 waypoints,
#       仅规划/执行一次,避免逐点规划造成的卡顿
uint8 planner_mode

# 批量模式(planner_mode=2)使用:完整轨迹的末端位姿序列(不含起始点)
# 节点会从当前位姿开始,依次直线经过所有 waypoints
geometry_msgs/Pose[] waypoints
