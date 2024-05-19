cmake_minimum_required(VERSION 3.8)
project(group10_final)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# include folder whish has all the header files
include_directories(include/group10_final)



# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ros2_aruco_interfaces
  mage_msgs
  nav_msgs
  nav2_msgs
)

# target_link_libraries(your_executable_name
#   ${tf2_LIBRARIES}
#   ${tf2_ros_LIBRARIES}
# )

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()




add_executable(aruco_subs src/aruco_subs.cpp)
ament_target_dependencies(aruco_subs ${FRAME_DEMO_INCLUDE_DEPENDS})

install(TARGETS aruco_subs
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)



#-----------------------------
# Installation
#-----------------------------


install(TARGETS
aruco_subs
# subscriber
# maze_travel
# turtlebotcontrol
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()



