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
 * @file 	DetectBall.cpp
 * @author 	Sukoon Sarin 	- Driver
 * @author 	Nalin Das    	- Navigator
 * @author 	Nidhi Bhojak    - design Keeper 
 * @brief   Implements object detection using HSV method which detects color of 
 *          the ball in a certain range and creates a bounding box over it.
*/
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/Image.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "../include/DetectBall.h"

// ROS Subscriber to get feed from the camera
DetectBall::DetectBall() {
  kinect_subscriber = nh.subscribe("/camera/color/image_raw",
                                   1,
                                   &DetectBall::kinectCallback,
                                   this);
}

DetectBall::~DetectBall() {
}

bool DetectBall::templateMatching() {
  // Convert image from BGR to HSV
  cv::Mat cv_image = getCvImage();
  // Apply gaussian blur
  cv::GaussianBlur(cv_image, cv_image, cv::Size(3, 3), 0.1, 0.1);

  cv::cvtColor(cv_image, hsv_image, cv::COLOR_BGR2HSV);
  // Perform HSV Thresholding
  cv::inRange(hsv_image,
              cv::Scalar(0, 15, 0),
              cv::Scalar(0, 255, 255),
              mask_image);
  // Apply erosion on masked images
  cv::erode(mask_image, mask_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
  // Apply dilation on masked image to remove any small blobs
  cv::dilate(mask_image, mask_image, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);

  if (!mask_image.empty()) {
    //cv::imshow("HSV Mask Image", mask_image);
    //cv::waitKey(30);
  }
  // Find contours in the Mask Image
  cv::findContours(mask_image,
                   contours_array,
                   CV_RETR_LIST,
                   CV_CHAIN_APPROX_NONE);

  if (cv::countNonZero(mask_image) == 0) {
    setBallDetected(false);
  } else {
     setBallDetected(true);
  }
  return getBallDetected();
}

void DetectBall::kinectCallback(const sensor_msgs::Image::ConstPtr& msg) {
  // CV Bridge Image ptr object to store converted ROS to OpenCV Image
  cv_bridge::CvImagePtr cv_img_ptr;
  ROS_INFO_STREAM("Recieved msg from kinect sensor");
  try {
    cv_img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv_image = cv_img_ptr->image;
    // cv::imshow("Converted CV Image", cv_image);
    // cv::waitKey(30);
  }
  catch (cv_bridge::Exception& excep) {
    ROS_ERROR_STREAM("CV_bridge Conversion Exception Raised!: "
                    << excep.what());
  }
}

void DetectBall::setCvImage(cv::Mat cv_image_) {
    cv_image = cv_image_;
}

cv::Mat DetectBall::getCvImage() {
    return cv_image;
}
