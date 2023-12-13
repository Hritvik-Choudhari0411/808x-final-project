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

  set(TESTFILES
    test/main.cpp
    test/test_manipulation.cpp
  )

  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp std_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs tf2_sensor_msgs OpenCV sensor_msgs gazebo_ros pluginlib nav_msgs gazebo_msgs)

  install(TARGETS
  test_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


endif()

add_executable(libratech src/main.cpp src/manipulation.cpp src/navigation.cpp src/perception.cpp src/simulation.cpp)
ament_target_dependencies(libratech
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

option(WANT_COVERAGE "this option enable coverage" ON)
if(NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  set(WANT_COVERAGE OFF)
endif()

if(WANT_COVERAGE)
  message("Enabling coverage")
  list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake-modules")
  include(CodeCoverage)
  # The following line come from cmake-modules/CodeCoverage.cmake
  append_coverage_compiler_flags() # will add "--coverage" compilation flag
  # Exclude certain directories from coverage analysis
  set(COVERAGE_EXCLUDES
    ${PROJECT_BINARY_DIR}
    )
endif()

#
# c++ Boilerplate Modification Starts Here
# ref: https://iamsorush.com/posts/cpp-cmake-essential/
# ref: https://cmake.org/cmake/help/latest/guide/tutorial/index.html
#

# create targets for building code coverage reports
if(WANT_COVERAGE)
  # unit test coverage:
  setup_target_for_coverage_lcov ( # use lcov to generate coverage report
    NAME test_coverage          # target name
    # EXECUTABLE ctest || true    # what to run insdie the build directory?
    #                             #   '|| true' --> continue coverage even if ctest failed
    EXECUTABLE ctest     # what to run insdie the build directory?
    EXCLUDE
      "src/*"            # Unit test does not run app, so don't analyze it
      "*gtest*"          # Don't analyze googleTest code
      "/usr/include/*"   # Don't analyze system headers
      "opencv/*"         # Don't analyze OpenCV headers
      "opencv_contrib/*" # Don't analyze OpenCV headers
      "*core*"           # Don't analyze OpenCV headers
      "/usr/local/include/*" # Don't analyze system headers
      "include/*"
    )

  # code coverage for the app (shell-app):
  # Use gcovr insetad of lcov if you want to see branch coverage as well
  setup_target_for_coverage_gcovr_html ( # use gcovr to generate coverage report
    NAME app_coverage          # target name
    EXECUTABLE tracker       # what to run insdie the build directory?
    EXCLUDE
    "test/test.cpp"    # App does not run unit test, so don't analyze it
    "test/main.cpp"    # App does not run unit test, so don't analyze it
    )

endif()

# Sanity check:
# can also do "cmake -S ./ -B build/ -LAH" to print all variables
message(STATUS "CMAKE_BUILD_TYPE = ${CMAKE_BUILD_TYPE}")
message(STATUS "WANT_COVERAGE    = ${WANT_COVERAGE}")

ament_package()
