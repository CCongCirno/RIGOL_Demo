# 目标位姿 + 规划模式标志
# 由 piper_control_node 发布,moveit_pose_goal_node 订阅
geometry_msgs/PoseStamped pose

# 规划模式:
#   0 = 自由路径(OMPL RRTConnect),用于回 home / 从 home 出发
#   1 = 直线运动(Pilz LIN),用于目标点之间的运动
uint8 planner_mode
