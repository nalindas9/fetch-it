/**
 * @file turtlebot.cpp
 * @brief Source file to implement turtlebot class
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
 * Controls motion of the turtlebot 
 *		   
 */

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlebot.h"
#include "detectBall.h"
#include "obstacleAvoidance.h"

TurtleBot:TurtleBot() {
}

TurtleBot::~TurtleBot() {
}

void TurtleBot::moveAhead(float linear_vel) {
    velocity.linear.x = 0.3;
    velocity.angular.z = 0.0;
}

void TurtleBot::turn(float angular_vel) {
    Velocity.linear.x = 0.0;
    Velocity.angular.z = 0.4;
}

void TurtleBot::collect() {
}

void TurtleBot::moveTurtle() { 
    moveAhead(-0.12);
    velocity_pub.publish(velocity);
}

void TurtleBot::reset() {
}

void TurtleBot::setBallPresent(bool ball_present_) {
    ball_present = ball_present_;
}

bool TurtleBot::getBallPresent() {
    return ball_present;
}