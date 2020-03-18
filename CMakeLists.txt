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

message(STATUS "Configuring Time...")
add_library(Time_lib src/Time.cxx)
target_link_libraries(Time_lib fastcdr fastrtps)

message(STATUS "Configuring Header...")
add_library(Header_lib src/Header.cxx)
target_link_libraries(Header_lib fastcdr fastrtps)

message(STATUS "Configuring BatteryState...")
add_library(BatteryState_fastrtps_lib src/BatteryState.cxx)
target_link_libraries(BatteryState_fastrtps_lib fastcdr fastrtps)

add_executable(test_subscriber src/subsciber_test.cpp)
ament_target_dependencies(test_subscriber rclcpp std_msgs sensor_msgs) 

add_executable(dds_cooperation src/dds_bridge.cpp src/BatteryStatePubSubTypes.cxx src/BatteryStateListener.cpp) 
target_link_libraries(dds_cooperation fastcdr fastrtps BatteryState_fastrtps_lib Header_lib Time_lib)
ament_target_dependencies(dds_cooperation fastcdr fastrtps rclcpp std_msgs sensor_msgs)

ament_package()

install(TARGETS test_subscriber dds_cooperation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)