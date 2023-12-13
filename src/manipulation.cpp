/**
 * @file Manipulation.cpp
 * @brief Manipulation class implementation (code stubs)
 * @author Hritvik Choudhari (hac@umd.edu)
 * @version 0.1
 * @date 2023-12-10
 * 
 * @copyright Copyright (c) 2023
 */

#include "../include/manipulation.hpp"
#include <fstream>
#include <ios>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>

/**
 * @brief Manipulation class constructor.
 * 
 * Initializes the Manipulation node and sets up necessary parameters.
 */
Manipulation::Manipulation() : Node("manipulation") {
    // Set the book place position
    place_pose.position.x = 0;
    place_pose.position.y = 0;
    place_pose.position.z = 0;
    place_pose.orientation.x = 0;
    place_pose.orientation.y = 0;
    place_pose.orientation.z = 0;
    place_pose.orientation.w = 0;
}

/**
 * @brief Picks the book using the gripper.
 * 
 * @return Nothing.
 */
bool Manipulation::pick_book() {
    // Check if the service is available and wait
    return true;
}

/**
 * @brief Places the book at the shelf.
 * 
 * @return Nothing.
 */
bool Manipulation::place_book() {
    // Place the book at the shelf using system command
    return true;
}
