/**
 * Copyright 2020 Nalin Das, Nidhi Bhojak, Sukoon Sarin
 * BSD 3-Clause License
 * 
 * @file ObstacleAvoidance.cpp
 * @brief Source file to implement obstacle avoidance class
 * @date 12/07/2020
 * @author Nidhi Bhojak
 * @author Nalin Das
 * 
 * @section LICENSE
 * BSD 3-Clause License
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
 * Source file to implement obstacle avoidance  
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "../include/ObstacleAvoidance.h"

ObstacleAvoidance::ObstacleAvoidance() {
    // Initialize the obstacle present to false
    obstacle_present = false;
    // ROS Subscriber to get data from the laser sensor 
    LaserScan = nh.subscribe<sensor_msgs::LaserScan>("/scan",
                 1000, &ObstacleAvoidance::laserScanCallback,
                 this);
}

ObstacleAvoidance::~ObstacleAvoidance() {
}

bool ObstacleAvoidance::getObstacleDetected() {
    return obstacle_present;
}

void ObstacleAvoidance::setObstacleDetected(bool present) {
    obstacle_present = present;
}

// Read Sensor data to get obstacle distances with respect to the robot 
void ObstacleAvoidance::laserScanCallback(const
            sensor_msgs::LaserScan
            ::ConstPtr& data) {
    double distance = data->ranges[180];
    // ROS_WARN_STREAM("Distance:" << distance);
    if (distance < 0.6) {
        setObstacleDetected(true);
        return;
    }
    setObstacleDetected(false);
}

bool ObstacleAvoidance::checkObstacle() {
    // Check if there is any obstacle 
    if (getObstacleDetected()) {
        ROS_WARN_STREAM("Obstacle Ahead!");
        return true;
    }
    return false;
}

