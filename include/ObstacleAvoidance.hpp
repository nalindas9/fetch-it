/**
 * @file ObstacleAvoidance.hpp
 * @brief Header file to implement Obstacle Avoidance 
 * @date 12/07/2020
 * @author Nidhi Bhojak
 * 
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
 * Class file to implement obstacle avoidance  
 *		   
 */

#pragma once

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class obstacleAvoidance {
    private:
        // Create the ROS Nodehandle
        ros::NodeHandle nh;

        // ROS Subscriber to get sensor data
        ros::Subscriber LaserScan;

        // Boolean variable to check the obstacle
        bool isObstacle;

        // BOt Velocities
        float linearVel;
        float angularVel;

    public: 
        /**
         * @brief Constructor for obstacle avoidance class
         * @param none
         * @return none
         *  **/
        obstacleAvoidance();

        /**
         * @brief Destructor for obstacle avoidance class
         * @param none
         * @return none
         *  **/
        ~obstacleAvoidance();

         /**
         * @brief Check obstacle function to check if there is obstacle
         * @param none 
         * @return bool value true or false for obstacle found
         *  **/
        bool checkObstacle();

        /**
         * @brief Laser Callback function
         * @param data from LaserScan node
         * @return void
         *  **/

        void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& data);
 
        bool getObstacleDetected(){
            return isObstacle;
        };

        void setObstacleDetected(bool obstacle) {
            isObstacle = obstacle;
        }
}
