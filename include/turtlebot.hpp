/**
 * @file turtlebot.hpp
 * @brief Header file to control the turtlebot motion
 * @date 12/07/2020
 * @author Nidhi Bhojak
 * 
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
 */

#pragma once 

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "../ObstacleAvoidance.h"
#include "../DetectBall.h"

class Turtlebot {
    private:
        // Define ROS Nodehandle
        ros::NodeHandle nh;

        // Publisher to publish velocities
        ros::Publisher pubVelocities;

        // Twist Object for Velocities
        geometry_msgs::Twist Velocity;

        // Linear Velocity in X-axis
        float linearVel;

        // Angular Velocity about Z-axis
        float angularVel;

        // Publishing Rate
        int PublishRate;

    public: 
        /**
         * @brief Constructor for Turtlebot class
         * @param none
         * @return none
         *  **/
        Turtlebot();

        /**
         * @brief Destructor for Turtlebot class
         * @param none
         * @return none
         *  **/
        ~Turtlebot();

        /**
         * @brief Function to move robot ahead
         * @param none
         * @return void
         *  **/
        void moveAhead(float linearVal);

        /**
         * @brief Function to rotate the robot
         * @param none
         * @return void
         *  **/
        void turn(float angularVal);

        /**
         * @brief Function to collect the Object 
         * @param none
         * @return void
         *  **/
        void collect();

        /**
         * @brief Implementing algorithm for robot motion
         * @param none
         * @return none
         *  **/
        void moveTurtle(obstacleAvoidance& ObstacleAvoidance);

        /**
         * @brief Reset Velocities
         * @param none
         * @return void
         *  **/
       bool reset();

}