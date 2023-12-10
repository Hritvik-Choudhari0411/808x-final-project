cmake_minimum_required(VERSION 3.8)
project(808x-final-project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)  
find_package(tf2_sensor_msgs REQUIRED)    
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  set(
    src/manipulation.cpp
    src/navigation.cpp
    src/perception.cpp
    src/simulation.cpp
    src/main.cpp
  )

endif()

add_executable(my_main src/main.cpp src/manipulation.cpp src/navigation.cpp src/perception.cpp src/simulation.cpp)
ament_target_dependencies(my_main
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  nav_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs 
  tf2_sensor_msgs  
)
include_directories(
    ${CMAKE_SOURCE_DIR}/include
)

ament_package()
