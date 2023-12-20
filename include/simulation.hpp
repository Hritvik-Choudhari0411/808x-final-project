/**
 * @file simulation.hpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @author Kshitij Karnawat (@KshitijKarnawat)
 * @brief RobotSim class interface
 * @version 0.1
 * @date 2023-12-10
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include <rclcpp/rclcpp.hpp>

#include "./manipulation.hpp"
#include "./navigation.hpp"
#include "./perception.hpp"

/**
 * @brief Simulation class to simulate the book collection and arranging task of
 * the robot.
 *
 */
class Simulation {
 public:
  /**
   * @brief Construct a new simulation object
   *
   */
  Simulation();

  Navigation navigation;
  Perception perception;
  Manipulation manipulation;
};
