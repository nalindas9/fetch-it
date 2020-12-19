/**
 * @file TurtleBot_test.cpp
 * @brief Test file for DetectBall class
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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include "../include/TurtleBot.h"




/**
 * @brief Check getters and setters
 */

TEST(DetectBallTest, objectNotDetected) {
    DetectBall detectball_dummy;
    detectball_dummy.setBallDetected(false);
    EXPECT_FALSE(detectball_dummy.getBallDetected());
}

/**
 * @brief Check getters and setters
 */

TEST(DetectBallTest, objectDetected) {
    DetectBall detectball_dummy;
    detectball_dummy.setBallDetected(true);
    EXPECT_TRUE(detectball_dummy.getBallDetected());
}

/**
 * @brief Test case for TemplateMatching method of DetectBall class
 */
TEST(DetectionTest, templateMatched) {
    DetectBall detectball_dummy;
    cv::Mat cv_image = cv::imread("../data/Tennis_Ball.jpg");
    detectball_dummy.templateMatching();
    EXPECT_FALSE(detectball_dummy.templateMatching());
}



