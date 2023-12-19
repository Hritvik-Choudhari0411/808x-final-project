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

    pick_client = create_client<gazebo_msgs::srv::DeleteEntity>(
        "delete_entity");
    place_client = create_client<gazebo_msgs::srv::SpawnEntity>(
        "spawn_entity");
    manpltn_node = rclcpp::Node::
                    make_shared("book_manipulation_node");
}

/**
 * @brief Picks the book using the gripper.
 * 
 * @return bool.
 */
bool Manipulation::pick_book() {
while (!pick_client->wait_for_service(std::chrono_literals::1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(),
              "Interruped while waiting for the server.");

          return false;
        }
        RCLCPP_INFO(this->get_logger(),
              "Server not available, waiting again...");
    }
    // Create a request to delete the entity
    auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
    request->name = "book1";

    auto result = pick_client->async_send_request(request);
    // Wait till the entity is deleted and the status is success
    auto ret = rclcpp::spin_until_future_complete(manpltn_node,
                                            result, 10s);
    if (ret == rclcpp::FutureReturnCode::SUCCESS) {
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Places the book at the shelf.
 * 
 * @return bool.
 */
bool Manipulation::place_book() {
    
    auto res = system("ros2 run gazebo_ros spawn_entity.py -entity book -x 3.5 -y -2.5 -z 0 -file `ros2 pkg prefix 808x-final-project`/share/808x-final-project/models/book/model.sdf");
    if (res > 0) {
        return true;
    } else {
        return false;
    }
}
