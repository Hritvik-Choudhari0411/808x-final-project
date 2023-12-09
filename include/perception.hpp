/**
 * @file perception.hpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Perception class declaration file
 * @version 0.1
 * @date 2022-12-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/logging.hpp"

#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

using TWIST = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using std::chrono::duration;
using namespace std::chrono_literals;
using ODOM = nav_msgs::msg::Odometry;

/**
 * @brief Perception class for detecting disposal bins using image processing.
 * 
 * This class is responsible for detecting disposal bins, moving the robot towards the bin
 * using camera and LiDAR data, and managing various flags for control logic.
 */
class Perception : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Perception object.
     * 
     * Initializes the Perception node and sets up necessary parameters.
     */
    Perception();

    /**
     * @brief Detects the disposal bin using image processing.
     * 
     * @return true If the bin is detected.
     * @return false If the bin is not detected.
     */
    bool detect_shelf();

    /**
     * @brief Moves the robot towards the bin using camera and LiDAR data.
     * 
     * @return true If the robot reaches the bin.
     * @return false If the robot doesn't reach the bin.
     */
    bool go_to_shelf();

    /**
     * @brief Callback function to subscribe to the odometry topic.
     * 
     * @param msg Odometry message.
     */
    void odom_callback(const ODOM::SharedPtr msg);

    /**
     * @brief Callback function to read the image from the robot.
     * 
     * @param msg Image message.
     */
    void img_sensor_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

 private:
    cv::Mat img_data; ///< Image data obtained from the robot.
    sensor_msgs::msg::LaserScan lidar_data; ///< LiDAR data obtained from the robot.
    rclcpp::NodeOptions node_opt; ///< Node options for Perception.
    image_transport::Subscriber sub; ///< Image subscriber.
    rclcpp::Node::SharedPtr img_node; ///< Image node for handling image data.
    rclcpp::Publisher<TWIST>::SharedPtr vel_pub; ///< Twist publisher for robot velocity.
    rclcpp::Node::SharedPtr percep_odom_node; ///< Node for handling odometry data.
    rclcpp::Subscription<ODOM>::SharedPtr odom_sub; ///< Odometry subscriber.
    bool rotate_R_flag; ///< Flag indicating right rotation.
    bool rotate_L_flag; ///< Flag indicating left rotation.
    bool move_forward; ///< Flag indicating forward movement.
    bool stop_flag; ///< Flag indicating stop condition.
    bool next_location; ///< Flag indicating the next location.
    double current_yaw; ///< Current yaw angle of the robot.
    double init_yaw; ///< Initial yaw angle of the robot.
};