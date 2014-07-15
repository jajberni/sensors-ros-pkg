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

#include <labust/sensors/image/ExternalCameraPublisherNode.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <labust/sensors/image/ObjectDetector.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


using namespace labust::sensors::image;

/**
 * ROS node for publishing videos from external cameras.
 */
ExternalCameraPublisherNode::ExternalCameraPublisherNode() :
    it_(nh_),
    ros_rate_(24),
    is_video_(true),
    camera_address_("device"),
    device_id_(0),
    publish_topic_("/camera/image_raw") {
  ros::NodeHandle ph("~");
  ph.getParam("ros_rate", ros_rate_);
  ph.getParam("is_video", is_video_);
  ph.getParam("camera_address", camera_address_);
  ph.getParam("device_id", device_id_);
  ph.getParam("publish_topic", publish_topic_);
  image_pub_ = it_.advertise(publish_topic_, 1);
}

ExternalCameraPublisherNode::~ExternalCameraPublisherNode() {
  if (is_video_) {
    video_capture_.release();
  }
}

void ExternalCameraPublisherNode::start() {
  ros::Rate r(ros_rate_);
  // If target is a valid video stream, open it now.
  if (is_video_) {
    if (camera_address_ == "device") {
      video_capture_.open(device_id_);
    } else {
      video_capture_.open(camera_address_);
    }
  }
  while (ros::ok()) {
    r.sleep();
    cv::Mat frame;
    // If target is a valid video stream, read one frame.
    // If target is not a valid video stream (e.g. a stream of .jpeg images), open image and read it.
    bool successful_read;
    if (is_video_) {
      successful_read = video_capture_.read(frame);
    } else {
      //frame = cv::imread(camera_address_, 1);
      //successful_read = static_cast<bool>(frame.data);
      video_capture_.open(camera_address_);
      successful_read = video_capture_.read(frame);
      //std::cout << cv::format(frame, "csv") << std::endl << std::endl;
      //return;
    }
    if (!successful_read) {
      ROS_INFO("No frame");
      continue;
    }
    // Convert cv::Mat image to sensor_msgs::Image and publish it.
    // TODO(irendulic): find encoding from cv::Mat, separate source type (video or images) in a better way.
    image_pub_.publish(cvImage2SensorImage(frame, "bgr8"));
    if (!is_video_) video_capture_.release();
    ros::spinOnce();
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "external_camera_publisher_node");
  ExternalCameraPublisherNode ex_cam;
  ex_cam.start();
  ros::spin();

  return 0;
}
