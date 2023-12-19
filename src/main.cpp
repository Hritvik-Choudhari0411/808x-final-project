/**
 * @file main.cpp
 * @author Hritvik Choudhari (hac@umd.edu)
 * @brief Main file for simulation and demo.
 * @version 0.1
 * @date 2023-12-10
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "../include/simulation.hpp"
#include <rclcpp/utilities.hpp>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char * argv[]) {
    // Initialize the ROS
    rclcpp::init(argc, argv);
    // Create Robot Simulator Object
    Simulation sim;
    // Start the operation
    RCLCPP_INFO(rclcpp::get_logger("log"), "Starting Trash Collection");
    while (!sim.navigation.search_book()) {
        if (sim.perception.detect_book()) {
            break;
        }
    }
    rclcpp::sleep_for(2s);
    sim.manipulation.pick_book();
    sim.navigation.go_to_shelf();
    sim.manipulation.place_book();
    sim.navigation.resume_search();
    // Shutdown ROS
    rclcpp::shutdown();
    // cv::destroyWindow("view");
    return 0;
}
