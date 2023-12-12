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

find_package(Doxygen)
if ( DOXYGEN_FOUND )
set( DOXYGEN_OUTPUT_DIRECTORY     ${PROJECT_SOURCE_DIR}/docs )
set( DOXYGEN_COLLABORATION_GRAPH  YES )
set( DOXYGEN_EXTRACT_ALL          YES )
set( DOXYGEN_CLASS_DIAGRAMS       YES )
set( DOXYGEN_HIDE_UNDOC_RELATIONS NO )
set( DOXYGEN_HAVE_DOT             YES )
set( DOXYGEN_CLASS_GRAPH          YES )
set( DOXYGEN_CALL_GRAPH           YES )
set( DOXYGEN_CALLER_GRAPH         YES )
set( DOXYGEN_COLLABORATION_GRAPH  YES )
set( DOXYGEN_BUILTIN_STL_SUPPORT  YES )
set( DOXYGEN_EXTRACT_PRIVATE      YES )
set( DOXYGEN_EXTRACT_PACKAGE      YES )
set( DOXYGEN_EXTRACT_STATIC       YES )
set( DOXYGEN_EXTRACT_LOCALMETHODS YES )
set( DOXYGEN_UML_LOOK             YES )
set( DOXYGEN_UML_LIMIT_NUM_FIELDS 50 )
set( DOXYGEN_TEMPLATE_RELATIONS   YES )
set( DOXYGEN_DOT_GRAPH_MAX_NODES  100 )
set( DOXYGEN_MAX_DOT_GRAPH_DEPTH  0 )
set( DOXYGEN_DOT_TRANSPARENT      YES )
set( DOXYGEN_INCLUDE_GRAPH        YES )
set( DOXYGEN_INCLUDED_BY_GRAPH    YES )
set( DOXYGEN_CALL_GRAPH           YES )
set( DOXYGEN_CLASS_DIAGRAMS       YES )
set( DOXYGEN_CLASS_GRAPH          YES )
else()
  message( FATAL_ERROR "Doxygen needs to be installed to generate the doxygen documentation" )
endif()

# create a target to build documentation
doxygen_add_docs(docs           # target name
  # List of files or directories
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
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
