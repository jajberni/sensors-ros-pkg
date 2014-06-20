/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, LABUST, UNIZG-FER
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the LABUST nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author : Ivor Rendulic
 *  Created: 18.06.2014.
 *********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <labust/sensors/image/ObjectDetectorNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/ObjectDetector.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace labust::sensors::image;

/**
 * Creates a ROS node for object detection from camera image.
 * TODO(irendulic): Allow external camera (currently ObjectDetectorNode cannot
 * be used with an external camera, and is not even instantiated when using an
 * IP camera).
 */
ObjectDetectorNode::ObjectDetectorNode() :
    it_(nh_),
    objectDetector() {
  // Subscribe to input video feed and publish output video feed
  image_transport::TransportHints hints("compressed", ros::TransportHints());
  image_sub_ = it_.subscribe("/camera/image_raw", 1, &ObjectDetectorNode::processFrame, this, hints);
  image_pub_ = it_.advertise("/object_tracking", 1);
}

// Process a sensor_image frame by converting it to openCV Mat format
void ObjectDetectorNode::processFrame(const sensor_msgs::ImageConstPtr &sensor_image) {
  image_pub_.publish(sensor_image);
  cv_bridge::CvImagePtr cv_image_bgr;
  cv::Point2f center;
  double area;

  try {
    cv_image_bgr = sensorImage2CvImage(sensor_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  objectDetector.detectObjectByColor(cv_image_bgr->image, center, area);
}

ObjectDetectorNode::~ObjectDetectorNode() {}

// TODO(irendulic): move the IP camera things to ObjectDetectorNode and allow
// different behavior for it by passing topic name/ip address of the camera.
int main(int argc, char** argv) {
  ros::init(argc, argv, "object_detector_node");
  //ObjectDetectorNode od;
  cv::Mat image;
  ObjectDetector object_detector;
  cv::namedWindow("Test");
  const std::string bosch_camera_address = "http://192.168.1.5/snap.jpg";
  while (true) {
    cv::VideoCapture vcap;
    vcap.open(bosch_camera_address);
    cv::Point2f center;
    double area;

    if(!vcap.read(image)) {
        ROS_INFO("No frame.");
        cv::waitKey(1000);
    } else {
      object_detector.detectObjectByColor(image, center, area);
      cv::circle(image, center, sqrt(area)/M_PI/2, cv::Scalar(0, 0, 255), -1);
      cv::imshow("Test", image);
    }
    if(cv::waitKey(200) >= 0) break;
  }   
  //vcap.release();
  //ros::spin();
  return 0;
}
