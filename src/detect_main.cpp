/**
 * @file detect_main.cpp
 * @brief Source main file for detection
 * @date 12/12/2020
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
 * Main source file for the fetch-it object detection algorithm
 *		   
 */
#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "../include/DetectBall.h"

int main(int argc, char** argv) {
    // Initialize main node
    ros::init(argc, argv, "detect_main");
    // DetectBall class object
    DetectBall detect_ball;
    ros::NodeHandle n;
    // ROS Publisher
    ros::Publisher detect_pub = n.advertise<std_msgs::Int8>
                                    ("ball_present", 1000);
    std_msgs::Int8 msg;
    while (ros::ok()) {
        cv::Mat image = detect_ball.getCvImage();
        if (!image.empty()) {
            bool ball_detected = detect_ball.templateMatching();
            // ROS_WARN_STREAM("ball_detected: " << ball_detected);
            msg.data = ball_detected;
            // ROS Publisher 
            detect_pub.publish(msg);
        }
        ros::spinOnce();
    }

    return 0;
}
