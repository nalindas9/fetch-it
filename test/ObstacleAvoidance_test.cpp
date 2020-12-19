/**
 * @file ObstacleAvoidance_test.cpp
 * @brief Test file for ObstacleAvoidance class
 * @date 12/15/2020
 * @author Sukoon Sarin
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
 * File containing unit tests for Class ObstacleAvoidance
 *		   
 */
#include "../include/ObstacleAvoidance.h"
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

/**
 * @brief	Test to check obstacle detected or not
 */
TEST(ObstacleAvoidanceTest, obstacleNotDetected) {
    ObstacleAvoidance obstacleavoidance_dummy;
    EXPECT_FALSE(obstacleavoidance_dummy.checkObstacle());
}

/**
 * @brief	Test to check getters and setters
 */
TEST(ObstacleAvoidanceTest, obstacleDetected) {
    ObstacleAvoidance obstacleavoidance_dummy;
    obstacleavoidance_dummy.setObstacleDetected(true);
    EXPECT_TRUE(obstacleavoidance_dummy.getObstacleDetected());
}

/**
 * @brief Check obstacle
 */
TEST(ObstacleAvoidanceTest, checkObstacle) {
    ObstacleAvoidance obstacleavoidance_dummy;
    /// Define distance threshold to detect obstacles
    double distance = 0.3;
    // how to call lasersensorcallback function()
    EXPECT_TRUE(obstacleavoidance_dummy.checkObstacle());
}
