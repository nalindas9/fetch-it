/**
 * @file TurtleBot_test.cpp
 * @brief Test file for turtlebot class
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
 * File containing unit tests for turtlebot class
 *		   
 */
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "../include/TurtleBot.h"

/**
 * @brief Class initialization test
 */
TEST(TurtleBotTest, classInitialization) {
    TurtleBot turtle_dummy;
    geometry_msgs::Twist velocity = turtle_dummy.getVelocity();
    EXPECT_EQ(0, velocity.linear.x);
    EXPECT_EQ(0, velocity.linear.y);
    EXPECT_EQ(0, velocity.linear.z);
    EXPECT_EQ(0, velocity.angular.x);
    EXPECT_EQ(0, velocity.angular.y);
    EXPECT_EQ(0, velocity.angular.z);
}

/**
 * @brief Test Move Ahead method
 */
TEST(TurtleBotTest, moveAheadTest) {
    TurtleBot turtle_dummy;
    turtle_dummy.moveAhead(0.4);
    geometry_msgs::Twist velocity = turtle_dummy.getVelocity();
    EXPECT_FLOAT_EQ(0.4, velocity.linear.x);
    EXPECT_FLOAT_EQ(0.0, velocity.angular.z);
}

/**
 * @brief Test turn method
 */
TEST(TurtleBotTest, turnTest) {
    TurtleBot turtle_dummy;
    turtle_dummy.turn(0.2);
    geometry_msgs::Twist velocity = turtle_dummy.getVelocity();
    EXPECT_FLOAT_EQ(0, velocity.linear.x);
    EXPECT_FLOAT_EQ(0.2, velocity.angular.z);
}

/**
 * @brief Test for setters
 */
TEST(TurtleBotTest, setterTest) {
    TurtleBot turtle_dummy;
    turtle_dummy.setBallPresent(true);
    turtle_dummy.setObstaclePresent(true);
    EXPECT_TRUE(turtle_dummy.getBallPresent());
    EXPECT_TRUE(turtle_dummy.getObstaclePresent());
}

/**
 * @brief Test for getters
 */
TEST(TurtleBotTest, getterTest) {
    TurtleBot turtle_dummy;
    turtle_dummy.setBallPresent(true);
    turtle_dummy.setObstaclePresent(true);
    EXPECT_TRUE(turtle_dummy.getBallPresent());
    EXPECT_TRUE(turtle_dummy.getObstaclePresent());
}
