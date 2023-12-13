/**
 * @file perception.hpp
 * @brief Declaration file for the Perception class, responsible for detecting books using image processing.
 * @author Hritvik Choudhari (hac@umd.edu)
 * @author Kshitij Karnawat (@KshitijKarnawat)
 * @version 0.1
 * @date 2023-12-09
 * 
 * @copyright Copyright (c) 2023
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
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>

/**
 * @brief Perception class for detecting books using image processing.
 * 
 * This class is responsible for detecting books, moving the robot towards the book
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
     * @brief Detects the book using image processing.
     * 
     * @return true If the book is detected.
     * @return false If the book is not detected.
     */
    bool detect_book();

    /**
     * @brief Moves the robot towards the book using camera and LiDAR data.
     * 
     * @return true If the robot reaches the book.
     * @return false If the robot doesn't reach the book.
     */
    bool go_to_book();

    /**
     * @brief Callback function to subscribe to the odometry topic while searching.
     * 
     * @param msg Odometry message.
     */
    void odom_callback_search(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Callback function to read the image from the robot.
     * 
     * @param msg Image message.
     */
    void img_sensor_callback(
      const sensor_msgs::msg::Image::ConstSharedPtr& msg);

 private:
    cv::Mat img_data;  // Image data obtained from the robot.

    // LiDAR data obtained from the robot.
    sensor_msgs::msg::LaserScan lidar_data;
    rclcpp::NodeOptions node_opt;  // Node options for Perception.
    image_transport::Subscriber sub;  // Image subscriber.
    rclcpp::Node::SharedPtr img_node;  // Image node for handling image data.

    // Twist publisher for robot velocity.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    // Node for handling odometry data.
    rclcpp::Node::SharedPtr percep_odom_node;

    // Odometry subscriber.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    bool rotate_R_flag;  // Flag indicating right rotation.
    bool rotate_L_flag;  // Flag indicating left rotation.
    bool move_forward;  // Flag indicating forward movement.
    bool stop_flag;  // Flag indicating stop condition.
    bool next_location;  // Flag indicating the next location.
    double current_yaw;  // Current yaw angle of the robot.
    double init_yaw;  // Initial yaw angle of the robot.
};
