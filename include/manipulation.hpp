/**
 * @file manipulation.hpp
 * @brief Declaration file for the Manipulation class, responsible for commanding the gripper to pick and place the book.
 * @author Hritvik Choudhari (hac@umd.edu)
 * @version 0.1
 * @date 2023-12-09
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include <gazebo_msgs/srv/detail/delete_entity__struct.hpp>

#include <fstream>
#include <iostream>

/**
 * @brief Manipulation class responsible for commanding the gripper to pick and place the book.
 * 
 * The Manipulation class provides member functions to command the gripper to pick and place the book
 * using Gazebo service calls. It utilizes Gazebo service clients for spawning and deleting entities in the simulation.
 */
class Manipulation : public rclcpp::Node {
 public:
    /**
     * @brief Construct a new Manipulation object.
     * 
     * Initializes the Manipulation node and sets up necessary parameters.
     */
    Manipulation();

    /**
     * @brief Picks the disposal book using the gripper.
     * 
     * @return true If the book is successfully picked.
     * @return false If the book cannot be picked.
     */
    bool pick_book();

    /**
     * @brief Places the disposal book using the gripper.
     * 
     * @return true If the book is successfully placed.
     * @return false If the book cannot be placed.
     */
    bool place_book();


 private:
    geometry_msgs::msg::Pose pick_pose; ///< Pose for picking the book.
    geometry_msgs::msg::Pose place_pose; ///< Pose for placing the book.
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr pick_client; ///< Service client for deleting the book after picking.
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr place_client; ///< Service client for spawning the book for placement.
    rclcpp::Node::SharedPtr manpltn_node; ///< Node for handling manipulation-related functionalities.
};
