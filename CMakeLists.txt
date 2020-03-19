cmake_minimum_required(VERSION 3.5)
project(dds_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

#set(CMAKE_CXX_EXTENSIONS OFF)
set(CycloneDDS_IDLC_ALWAYS OFF)
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/dashing/share/")


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

message(STATUS "Configuring Twist...")
add_library(Twist_lib src/Twist.cxx)
target_link_libraries(Twist_lib fastcdr fastrtps)

message(STATUS "Configuring Vector3...")
add_library(Vector3_lib src/Vector3.cxx)
target_link_libraries(Vector3_lib fastcdr fastrtps)

message(STATUS "Configuring Time...")
add_library(Time_lib src/Time.cxx)
target_link_libraries(Time_lib fastcdr fastrtps)

message(STATUS "Configuring Header...")
add_library(Header_lib src/Header.cxx)
target_link_libraries(Header_lib fastcdr fastrtps)

message(STATUS "Configuring Point...")
add_library(Point_lib src/Point.cxx)
target_link_libraries(Point_lib fastcdr fastrtps)

message(STATUS "Configuring Quaternion...")
add_library(Quaternion_lib src/Quaternion.cxx)
target_link_libraries(Quaternion_lib fastcdr fastrtps)

message(STATUS "Configuring Pose...")
add_library(Pose_lib src/Pose.cxx)
target_link_libraries(Pose_lib Quaternion_lib Point_lib fastcdr fastrtps)

message(STATUS "Configuring PoseStamped...")
add_library(PoseStamped_lib src/PoseStamped.cxx)
target_link_libraries(PoseStamped_lib Pose_lib fastcdr fastrtps)

message(STATUS "Configuring BatteryState...")
add_library(BatteryState_fastrtps_lib src/BatteryState.cxx)
target_link_libraries(BatteryState_fastrtps_lib fastcdr fastrtps)

message(STATUS "Configuring Transform...")
add_library(Transform_lib src/Transform.cxx)
target_link_libraries(Transform_lib fastcdr fastrtps Quaternion_lib Vector3_lib)

message(STATUS "Configuring TransformStamped...")
add_library(TransformStamped_lib src/TransformStamped.cxx)
target_link_libraries(TransformStamped_lib fastcdr fastrtps Transform_lib Header_lib)

message(STATUS "Configuring TFMessage...")
add_library(TFMessage_lib src/TFMessage.cxx)
target_link_libraries(TFMessage_lib fastcdr fastrtps TransformStamped_lib)

add_executable(test_broadcaster src/tf_broadcaster_test.cpp)
ament_target_dependencies(test_broadcaster rclcpp  tf2_ros tf2) 

add_executable(test_subscriber src/subsciber_test.cpp)
ament_target_dependencies(test_subscriber rclcpp std_msgs sensor_msgs) 

add_executable(dds_bridge src/dds_bridge.cpp src/BatteryStatePubSubTypes.cxx src/BatteryStateListener.cpp src/TwistPubSubTypes.cxx src/PoseStampedPubSubTypes.cxx src/PoseStampedListener.cpp)
target_link_libraries(dds_bridge fastcdr fastrtps BatteryState_fastrtps_lib Header_lib Time_lib Twist_lib Vector3_lib PoseStamped_lib)
ament_target_dependencies(dds_bridge fastcdr fastrtps rclcpp std_msgs sensor_msgs)

ament_package()

install(TARGETS test_subscriber dds_bridge test_broadcaster
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)