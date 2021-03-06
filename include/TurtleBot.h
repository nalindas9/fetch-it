/**
 * @file turtlebot.h
 * @brief Header file to control the turtlebot motion
 * @date 12/07/2020
 * @author Nidhi Bhojak
 * @author Nalin Das
 * 
 * @section LICENSE
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
 * @section DESCRIPTION 
 * 
 * Header file for turtlebot motion control
 */

#pragma once 

#include "../include/ObstacleAvoidance.h"
#include "../include/DetectBall.h"
#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include "geometry_msgs/Twist.h"


class TurtleBot {
    private:
        // Obstacle avoidance object
        ObstacleAvoidance obstacle_avoidance;
        // DetectBall class object
        DetectBall detect_ball;
        
        // Store ball detected status
        bool ball_present;
        // Define ROS Nodehandle
        ros::NodeHandle nh;
        // Store obstacle present status
        bool obstacle_present;
        // Publisher to publish velocities
        ros::Publisher velocity_pub;

        // Twist Object for Velocities
        geometry_msgs::Twist velocity;

        // ROS subscriber to ball present topic
        ros::Subscriber detect_sub;

        // Publishing Rate
        int publish_rate;

    public:
        /**
         * @brief Constructor for Turtlebot class
         * @param none
         * @return none
         *  **/
        TurtleBot();

        /**
         * @brief Destructor for Turtlebot class
         * @param none
         * @return none
         *  **/
        ~TurtleBot();

        /**
         * @brief Function to move turtlebot ahead
         * @param linear_vel Linear Velocity
         * @return None
         *  **/
        void moveAhead(float linear_vel);

        /**
         * @brief Function to rotate the robot
         * @param angular_vel Angular velocity
         * @return void
         *  **/
        void turn(float angular_vel);

        /**
         * @brief Function to pick up the tennis ball 
         * @param none
         * @return None
         *  **/
        void collect();

        /**
         * @brief Move the turtlebot while avoiding obstacles
         * @param None None
         * @return None
         *  **/
        void moveTurtle();

        /**
         * @brief Reset Velocities
         * @param none
         * @return None
         *  **/
       void reset();

       /**
         * @brief Setter function for ball present check
         * @param ball_present_ bool value
         * @return void
         *  **/

       void setBallPresent(bool ball_present_);

       /**
         * @brief Getter function to get ball present value
         * @param none
         * @return bool value
         *  **/ 
       bool getBallPresent();

       /**
         * @brief Function to get velocities
         * @param none
         * @return velocity values for x, y, z
         *  **/

       geometry_msgs::Twist getVelocity();

       /**
         * @brief Function to set obstacle present 
         * @param present bool value
         * @return void 
         *  **/

       void setObstaclePresent(bool present);

       /**
         * @brief Function to get obstacle present 
         * @param none 
         * @return bool value true or false for obstacle present 
         *  **/

       bool getObstaclePresent();

       /**
         * @brief Detect Callback fucntion
         * @param msg 
         * @return void
         *  **/

       void detectCallback(const std_msgs::Int8::ConstPtr& msg);
};
