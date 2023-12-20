/**
 * @file navigation.cpp
 * @brief Navigation class implementation (code stubs)
 * @author Hritvik Choudhari (hac@umd.edu)
 * @author Kshitij Karnawat (@KshitijKarnawat)
 * @version 0.1
 * @date 2023-12-10
 * 
 * @copyright Copyright (c) 2023
 */

#include "../include/navigation.hpp"
#include <cmath>
#include <iostream>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <chrono>

using namespace std::chrono_literals;
/**
 * @brief Navigation class constructor.
 * 
 * Initializes the Navigation node and sets up necessary parameters.
 */
Navigation::Navigation() : Node("navigation") {
    // Create the nodes and publisher for goal pose
    nav_odom_node = rclcpp::Node::make_shared("odom_node");
    nav_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "goal_pose", 10);
    // Set the position and flags required.
    check_odom = false;
    req_pos_y = 0.0;
}

/**
 * @brief Callback function for odom topic during the search.
 * 
 * @param msg Odometry message.
 */
void Navigation::odom_callback_search(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if the required position is reached.
    if ((std::abs(static_cast<int>(msg->pose.pose.position.y - req_pos_y))
        == 0) && (std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)) {
            check_odom = true;
    }
}

/**
 * @brief Callback function for odom topic during book placement in shelf.
 * 
 * @param msg Odometry message.
 */
void Navigation::odom_callback_place(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if the required disposal location is reached.
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x - 3)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y + 2.5)) == 0)) {
            check_odom = true;
    }
}

/**
 * @brief Callback function for odom topic during resuming the search.
 * 
 * @param msg Odometry message.
 */
void Navigation::odom_callback_resume(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Check if the resume search location is reached.
    if ((std::abs(static_cast<int>(msg->pose.pose.position.x)) == 0)
        && (std::abs(static_cast<int>(msg->pose.pose.position.y)) == 0)) {
            check_odom = true;
    }
}


/**
 * @brief Executes the search algorithm in the map.
 * 
 * @return true If the search algorithm is successful.
 * @return false If the search algorithm fails.
 */
bool Navigation::search_book() {
    // Wait for a second for the odom to receive data
    rclcpp::sleep_for(1000ms);
    // Grid search positions
    std::vector<float_t> search_pos = {6.0, 4.0, 2.0, 0.0};
    // Odom subscriber initialization
    auto odom_sub = nav_odom_node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                        std::bind(&Navigation::odom_callback_search, this, std::placeholders::_1));
    // Start the searching
    while (search_pos.size() > 0) {
        float_t pop_pos = search_pos.back();
        search_pos.pop_back();
        RCLCPP_INFO(this->get_logger(), "In Search Books %ld %f",
                    search_pos.size(), pop_pos);
        check_odom = false;
        req_pos_y = pop_pos;
        geometry_msgs::msg::PoseStamped rpyGoal;
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = this->get_clock()->now();
        // Set the search position
        rpyGoal.pose.position.x = 0;
        rpyGoal.pose.position.y = pop_pos;
        rpyGoal.pose.position.z = 0;
        rpyGoal.pose.orientation.x = 0;
        rpyGoal.pose.orientation.y = 0;
        rpyGoal.pose.orientation.z = 0;
        rpyGoal.pose.orientation.w = 1;
        // Check the position
        while (!check_odom) {
            rclcpp::spin_some(nav_odom_node);
            nav_pub_->publish(rpyGoal);
            rclcpp::sleep_for(500ms);
        }
        return false;
    }
    return true;  // or false based on the actual logic
}

/**
 * @brief Moves the robot and book to the shelf.
 * 
 * @return true If the robot successfully reaches the shelf.
 * @return false If the robot cannot reach the shelf.
 */
bool Navigation::go_to_shelf() {
    RCLCPP_INFO(this->get_logger(), "In Move to Drop Zone");
    // Initialize the flag
    check_odom = false;
    auto odom_sub = nav_odom_node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                    std::bind(&Navigation::odom_callback_place, this, std::placeholders::_1));
    geometry_msgs::msg::PoseStamped rpyGoal;
    // Set the location in the pose
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    rpyGoal.pose.position.x = 3;
    rpyGoal.pose.position.y = -2.5;
    rpyGoal.pose.position.z = 0;
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;
    // Check if the goal is reached.
    while (!check_odom) {
        rclcpp::spin_some(nav_odom_node);
        nav_pub_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
    }
    return true;  // or false based on the actual logic
}

/**
 * @brief Resumes the search algorithm after placing book in the shelf.
 * 
 * @return true If the search can be resumed.
 * @return false If the search cannot be resumed.
 */
bool Navigation::resume_search() {
    RCLCPP_INFO(this->get_logger(), "In move to resume search");
    // Set the flag
    check_odom = false;
    auto odom_sub = nav_odom_node->create_subscription<nav_msgs::msg::Odometry>("odom", 10,
                    std::bind(&Navigation::odom_callback_search, this, std::placeholders::_1));
    geometry_msgs::msg::PoseStamped rpyGoal;
    // Set the resume search location
    rpyGoal.header.frame_id = "map";
    rpyGoal.header.stamp = this->get_clock()->now();
    rpyGoal.pose.position.x = 0;
    rpyGoal.pose.position.y = 0;
    rpyGoal.pose.position.z = 0;
    rpyGoal.pose.orientation.x = 0;
    rpyGoal.pose.orientation.y = 0;
    rpyGoal.pose.orientation.z = 0;
    rpyGoal.pose.orientation.w = 1;

    while (!check_odom) {
        rclcpp::spin_some(nav_odom_node);
        nav_pub_->publish(rpyGoal);
        rclcpp::sleep_for(500ms);
    }
    return true;  // or false based on the actual logic
}
