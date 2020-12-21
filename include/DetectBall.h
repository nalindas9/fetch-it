/**
 * BSD 3-Clause License
 *
 * @copyright (c) 2020, Sukoon Sarin, Nalin Das, Nidhi Bhojak
 * 
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
 */

/**
 * @file 	  DetectBall.h
 * @author 	Sukoon Sarin 	- Driver
 * @author 	Nalin Das    	- Navigator
 * @author 	Nidhi Bhojak  - design Keeper 
 * @brief 	Library header file to implement object detection
 *         	Class for template matching to detect tennis balls in the robot's world
 */

#ifndef INCLUDE_DETECTBALL_H_
#define INCLUDE_DETECTBALL_H_

#include <vector>
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"

/** 
 * Define class for ball detection 
 * **/

class DetectBall {
 private:
  /// Node Handle for ROS system
  ros::NodeHandle nh;

  // ROS Subscriber for kinect camera
  ros::Subscriber kinect_subscriber;

  // Array to store contours for each image
  std::vector<std::vector<cv::Point>> contours_array;

  /// Container to store converted image from cv_bridge
  cv::Mat cv_image;

  // For storing the HSV image and mask image
  cv::Mat mask_image, hsv_image;
  
  bool ObjectDetected;

 public:
/**
 * @brief  Constructor for Detect ball class
 * @param  none
 * @return none
 */
  DetectBall();

/**
 * @brief  Destructor for Detect ball class
 * @param  none
 * @return none
 */
  ~DetectBall();

  /**
   * @brief Kinect sensor callback
   * @param msg Sensor msg with RGB Image data
   * @return None
   */
  void kinectCallback(const sensor_msgs::Image::ConstPtr& msg);
  
  /**
   * @brief   Method to implement template matching
   * @param   Image filtered image of type cv::Mat
   * @return  Match found of type bool
   */
  bool templateMatching();

  /**
   * @brief   Get detected Object
   * @param   none
   * @return  Object detected or not of type bool
   */
  bool getBallDetected() {
    return ObjectDetected;
  }

  /**
   * @brief   Set detected Object
   * @param   object detected status
   * @return  none
   */
  void setBallDetected(bool object) {
    ObjectDetected = object;
  }

  /**
   * @brief Set OpenCV converted image
   * @param None None
   * @return Converted OpenCV Image
   */
  void setCvImage(cv::Mat cv_image);

  /**
   * @brief Get OpenCV converted image
   * @param None None
   * @return Converted OpenCV Image
   */
  cv::Mat getCvImage();

};

#endif  // INCLUDE_DETECTBALL_H_
