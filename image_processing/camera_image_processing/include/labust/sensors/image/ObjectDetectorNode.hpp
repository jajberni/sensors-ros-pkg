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
*********************************************************************/
#ifndef OBJECTDETECTORNODE_HPP_
#define OBJECTDETECTORNODE_HPP_
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float64MultiArray.h>
#include <labust/sensors/image/ObjectDetector.hpp>

namespace labust {
  namespace sensors {
    namespace image {

      /**
       * ROS node for object detection in camera image.
       * TODO(irendulic): Add support for camera not publishing a ROS
       * topic (e.g. IP camera).
       */
      class ObjectDetectorNode {

      public:
        ObjectDetectorNode();
        ~ObjectDetectorNode();
        void setObjectDetector(ObjectDetector *object_detector);
        void setEnableVideoDisplay(bool enable_video_display);

      private:
        void processFrame(const sensor_msgs::ImageConstPtr &sensor_image);
        ros::NodeHandle nh_;
        ros::Publisher detected_object_publisher_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_;
        image_transport::Publisher image_pub_;
        std_msgs::Float64MultiArrayPtr detected_object_;
        ObjectDetector *object_detector_;
        std::string camera_topic_, opencv_window_;
        bool is_compressed_, enable_video_display_;
      };

    }
  }
}

/* OBJECTDETECTORNODE_HPP_ */
#endif
