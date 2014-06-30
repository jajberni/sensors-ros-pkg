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
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/image_encodings.h>

#include <labust/sensors/image/ObjectDetectorNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/ColorObjectDetector.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace labust::sensors::image;

/**
 * Creates a ROS node for object detection from camera image.
 */
ObjectDetectorNode::ObjectDetectorNode() :
    it_(nh_),
    camera_topic_("/camera/image_raw"),
    is_compressed_(false),
    enable_video_display_(false),
    opencv_window_("detected_object"),
    detected_object_(new std_msgs::Float64MultiArray()) {

  ros::NodeHandle ph("~");
  ph.getParam("camera_topic", camera_topic_);
  ph.getParam("is_compressed", is_compressed_);

  // To access compressed video TransportHints must be used.
  if (is_compressed_) {
    image_transport::TransportHints hints("compressed", ros::TransportHints());
    image_sub_ = it_.subscribe(camera_topic_, 1, &ObjectDetectorNode::processFrame, this, hints);
  } else {
    image_sub_ = it_.subscribe(camera_topic_, 1, &ObjectDetectorNode::processFrame, this);
  }
  image_pub_ = it_.advertise("/object_tracking", 1);

  detected_object_publisher_ = nh_.advertise<std_msgs::Float64MultiArray>("detected_object", 1);
  detected_object_->data.resize(3);
}

ObjectDetectorNode::~ObjectDetectorNode() {}

void ObjectDetectorNode::setObjectDetector(ObjectDetector *object_detector) {
  object_detector_ = object_detector;
}

/**
 * Process image frame.
 * sensor_msgs::Image is converted to OpenCV format via cv_bridge and sent to ObjectDetector object.
 */
void ObjectDetectorNode::processFrame(const sensor_msgs::ImageConstPtr &sensor_image) {
  cv_bridge::CvImagePtr cv_image_bgr;
  cv::Point2f center;
  double area;
  try {
    cv_image_bgr = sensorImage2CvImage(sensor_image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  object_detector_->detect(cv_image_bgr->image, center, area);
  if (enable_video_display_ && area > 0) {
    cv::circle(cv_image_bgr->image, center, sqrt(area)/M_PI/2, cv::Scalar(0, 0, 255), -1);
    cv::imshow(opencv_window_, cv_image_bgr->image);
    cv::waitKey(1);
  }
  detected_object_->data[0] = center.x;
  detected_object_->data[1] = center.y;
  detected_object_->data[2] = area;
  detected_object_publisher_.publish(detected_object_);
}

void ObjectDetectorNode::setEnableVideoDisplay(bool enable_video_display) {
  enable_video_display_ = enable_video_display;
  object_detector_->setEnableVideoDisplay(enable_video_display);
  cv::namedWindow(opencv_window_); 
  cv::waitKey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "object_detector_node");
  ObjectDetectorNode od;
  od.setObjectDetector(new ColorObjectDetector());
  od.setEnableVideoDisplay(true);
  ros::spin();

  return 0;
}
