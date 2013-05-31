cmake_minimum_required(VERSION 2.8.3)
project(alarm)
find_package(catkin REQUIRED COMPONENTS std_msgs auv_sensor_msgs rospy roscpp)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(alarm src/alarm.cpp)
target_link_libraries(alarm ${catkin_LIBRARIES})
