cmake_minimum_required(VERSION 3.8)
project(rigol_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 强制使用 C++17(MoveIt2 要求)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_common REQUIRED)
find_package(realsense2 REQUIRED)
set(RS_DEPENDENCIES realsense2)
find_package(OpenCV REQUIRED)


include_directories(
  include/${PROJECT_NAME}
  include/${PROJECT_NAME}/devices/camera
  ${OpenCV_INCLUDE_DIRS}
)

add_library(d435i-video-capture SHARED src/devices/camera/d435i_video_capture.cpp)
list(APPEND DEMO_LIBS d435i-video-capture)

# 接收目标位姿后调用 piper_no_gripper_moveit 进行 plan & execute 的节点
add_executable(moveit_pose_goal_node src/moveit_pose_goal_node.cpp)
set_target_properties(moveit_pose_goal_node PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)
ament_target_dependencies(moveit_pose_goal_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# D435i 图像测试节点
add_executable(d435i_image_test src/test/d435i_image_test.cpp)
set_target_properties(d435i_image_test PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)
target_link_libraries(d435i_image_test
  ${RS_DEPENDENCIES}
  ${DEMO_LIBS}
  ${OpenCV_LIBS}
)

install(TARGETS
  ${DEMO_LIBS}
  moveit_pose_goal_node
  d435i_image_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
