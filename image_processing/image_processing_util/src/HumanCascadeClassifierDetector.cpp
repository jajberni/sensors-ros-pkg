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
#include <algorithm>
#include <labust/sensors/image/HumanCascadeClassifierDetector.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace labust::sensors::image;

HumanCascadeClassifierDetector::HumanCascadeClassifierDetector() {
  WINDOW_ = "Human face detector";
  enable_video_display_ = false;

  // TODO(irendulic): Change this, upload .xml files.
  haarcascade_files_.push_back("haarcascade_eye.xml");
  haarcascade_files_.push_back("haarcascade_frontalface_default.xml");
  haarcascade_files_.push_back("haarcascade_fist.xml");
  haarcascade_files_.push_back("haarcascade_mcs_mouth.xml");
  haarcascade_files_.push_back("haarcascade_palm.xml");

}

void HumanCascadeClassifierDetector::addCascade(CascadeType cascade_type) {
  cascades_.push_back(cv::CascadeClassifier("/home/ivor/haarcascades/" + haarcascade_files_.at(cascade_type)));
}

void HumanCascadeClassifierDetector::setEnableVideoDisplay(bool enable_video_display) {
  enable_video_display_ = enable_video_display;
  this->createOpenCvWindow();
}

void HumanCascadeClassifierDetector::createOpenCvWindow() {
  cv::namedWindow(WINDOW_);
  cv::waitKey(1);
}

HumanCascadeClassifierDetector::~HumanCascadeClassifierDetector() {
  cv::destroyWindow(WINDOW_);
}

void HumanCascadeClassifierDetector::detect(cv::Mat &image, cv::Point2f &center, double &area) {
  cv::Mat image_gray;
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  cv::equalizeHist(image_gray, image_gray);

  for (std::vector<cv::CascadeClassifier>::iterator cascade_it = cascades_.begin(); cascade_it != cascades_.end(); ++cascade_it) {
    std::vector<cv::Rect> found;

    cascade_it->detectMultiScale(image_gray, found, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    for (std::vector<cv::Rect>::iterator r = found.begin(); r != found.end(); r++)
      cv::rectangle(image, *r, CV_RGB(255,0,0), 3);

    if (enable_video_display_) {
      cv::imshow(WINDOW_, image);
      cv::waitKey(1);
    }
  }
}
