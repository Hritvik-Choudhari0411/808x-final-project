/**
 * @file navigation.hpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Declaration file for the Navigation class, responsible for generating the search path and moving the robot.
 * @version 0.1
 * @date 2023-12-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "./Perception.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <memory>
#include <iostream>
#include <vector>
#include <chrono>

using POSE = geometry_msgs::msg::PoseStamped;
using PUBLISHER = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
using TIMER = rclcpp::TimerBase::SharedPtr;
using ODOM = nav_msgs::msg::Odometry;
using std::placeholders::_1;
using std::chrono::duration;
using namespace std::chrono_literals;

/**
 * @brief Navigation class responsible for generating the search path and moving the robot.
 * 
 * The Navigation class is designed to execute a search algorithm in the map, move the robot and book to the respective shelf,
 * and resume the search algorithm after that. The Perception class is a friend class of Navigation.
 */
class Navigation : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Navigation object.
     * 
     * Initializes the Navigation node and sets up necessary parameters.
     */
    Navigation();

    /**
     * @brief Executes the search algorithm in the map.
     * 
     * @return true If the search algorithm is successful.
     * @return false If the search algorithm fails.
     */
    bool search_book();

    /**
     * @brief Moves the robot and book to the shelf.
     * 
     * @return true If the robot successfully reaches the shelf.
     * @return false If the robot cannot reach the shelf.
     */
    bool go_to_shelf();

    /**
     * @brief Resumes the search algorithm after placing book in the shelf.
     * 
     * @return true If the search can be resumed.
     * @return false If the search cannot be resumed.
     */
    bool resume_search();
    /**
     * @brief Callback for odom topic during the search.
     * 
     * @param msg Odometry message.
     */
    void odom_callback_search(const ODOM::SharedPtr msg);

    /**
     * @brief Callback for odom topic during book placement in shelf.
     * 
     * @param msg Odometry message.
     */
    void odom_callback_place(const ODOM::SharedPtr msg);

    /**
     * @brief Callback for odom topic during resuming the search.
     * 
     * @param msg Odometry message.
     */
    void odom_callback_resume(const ODOM::SharedPtr msg);

 private:
    PUBLISHER nav_pub_; ///< PoseStamped publisher for navigation.
    TIMER timer_; ///< Timer for controlling periodic tasks.
    std::shared_ptr<rclcpp::Node> nav_odom_node; ///< Node for handling odometry data during navigation.
    bool check_odom; ///< Flag for checking odometry status.
    float_t req_pos_y; ///< Required position along the y-axis.
};