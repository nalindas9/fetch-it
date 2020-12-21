/**
 * @file turtlebot.cpp
 * @brief Source file to implement turtlebot class
 * @date 12/07/2020
 * @author Nidhi Bhojak
 * @author Nalin Das
 * 
 * @section LICENSE
 *
 * @copyright (c) 2020, Nalin Das, Sukoon Sarin, Nidhi Bhojak
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION 
 * 
 * Controls motion of the turtlebot 
 *		   
 */
// Add ROS headers 
#include <std_msgs/Int8.h>
#include "ros/ros.h"
#include "../include/TurtleBot.h"
#include "geometry_msgs/Twist.h"
#include "../include/ObstacleAvoidance.h"

// Implement Turtlebot Class

/**
 * @brief Laser Callback function
 * @param data from LaserScan node
 * @return void
 *  **/
TurtleBot::TurtleBot() {
    // Initialize to all zeros
    velocity.linear.x = 0.0;
    velocity.linear.y = 0.0;
    velocity.linear.z = 0.0;
    velocity.angular.x = 0.0;
    velocity.angular.y = 0.0;
    velocity.angular.z = 0.0;

    // Publsh the velocities to /cmd_vel topic
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // ROS Subscriber for ball_present topic
    detect_sub = nh.subscribe<std_msgs::Int8>("ball_present",
                     1000, &TurtleBot::detectCallback, this);
}

TurtleBot::~TurtleBot() {
    ROS_INFO_STREAM("Turtlebot will stop now.");
}

void TurtleBot::moveAhead(float linear_vel) {
    velocity.linear.x = linear_vel;
    velocity.angular.z = 0.0;
}

void TurtleBot::turn(float angular_vel) {
    velocity.linear.x = 0.0;
    velocity.angular.z = angular_vel;
}

// void TurtleBot::collect() {
// }

void TurtleBot::moveTurtle() {
    // Looprate of 4 Hz
    ros::Rate rate(4);

    while (ros::ok()) {
        // Define twist msg
        geometry_msgs::Twist twist;
        // True if obstacle present
        obstacle_present = obstacle_avoidance.checkObstacle();
        ROS_WARN_STREAM("obstacle_present: " << obstacle_present
                         << "ball_present:" << getBallPresent());

        // Start moving the robot if no obstacle detected
        if (!obstacle_present && getBallPresent()) {
            ROS_WARN_STREAM("Moving forward ...");
            moveAhead(-0.12);

        // Start turning the robot to avoid obstacle 
        } else {
            ROS_WARN_STREAM("Rotating ...");
            turn(0.8);
        }
        
        // Publish the velocties 
        velocity_pub.publish(velocity);
        ros::spinOnce();
        // Pause to maintain loop rate
        rate.sleep();
    }
}

// void TurtleBot::reset() {
// }

void TurtleBot::setBallPresent(bool ball_present_) {
    ball_present = ball_present_;
}

bool TurtleBot::getBallPresent() {
    return ball_present;
}

geometry_msgs::Twist TurtleBot::getVelocity() {
    return velocity;
}

void TurtleBot::setObstaclePresent(bool present) {
    obstacle_present = present;
}

bool TurtleBot::getObstaclePresent() {
    return obstacle_present;
}

void TurtleBot::detectCallback(const std_msgs::Int8::ConstPtr& msg) {
    // ROS_WARN_STREAM("I heard: [%s]" << msg->data);
    (msg->data == 1) ? setBallPresent(true) : setBallPresent(false);
    // ROS_WARN_STREAM("ball_present inside callback:" << getBallPresent());
}
