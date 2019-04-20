cmake_minimum_required(VERSION 2.8)
project(youbot_move)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy geometry_msgs sensor_msgs std_msgs) # genmsg

## Declare a catkin package
catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)

set(SOURCE
   main.cpp    
)

set_source_files_properties(${SOURCE} PROPERTIES LANGUAGE CXX )
add_executable(${PROJECT_NAME} ${SOURCE})	
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


 