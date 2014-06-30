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
#include <algorithm>
#include <labust/sensors/image/ColorObjectDetector.hpp>
#include <labust/sensors/image/ImageProcessingUtil.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace labust::sensors::image;

ColorObjectDetector::ColorObjectDetector() {
  WINDOW_ = "Color object detector";
  enable_video_display_ = false;
  iLowH = 0;
  iHighH = 179;
  iLowS = 0; 
  iHighS = 255;
  iLowV = 0;
  iHighV = 255;
}

void ColorObjectDetector::setEnableVideoDisplay(bool enable_video_display) {
  enable_video_display_ = enable_video_display;
  this->createOpenCvWindow();
}

void ColorObjectDetector::createOpenCvWindow() {
  cv::namedWindow(WINDOW_);

  // Create trackbars in "Control" window
  // Hue (0-179) 
  cvCreateTrackbar("LowH", WINDOW_, &iLowH, 179); 
  cvCreateTrackbar("HighH", WINDOW_, &iHighH, 179);
  // Saturation (0-255)
  cvCreateTrackbar("LowS", WINDOW_, &iLowS, 255); 
  cvCreateTrackbar("HighS", WINDOW_, &iHighS, 255);
  // Value (0-255)
  cvCreateTrackbar("LowV", WINDOW_, &iLowV, 255); 
  cvCreateTrackbar("HighV", WINDOW_, &iHighV, 255);
  cv::waitKey(1);
}

ColorObjectDetector::~ColorObjectDetector() {
  cv::destroyWindow(WINDOW_);
}

bool compare(cv::vector<cv::Point> a, cv::vector<cv::Point> b) {
  return (a.size() > b.size());
}

void ColorObjectDetector::detect(const cv::Mat image_bgr, cv::Point2f &center, double &area) {
  cv::Mat image_hsv, image_thresholded;
  cv::cvtColor(image_bgr, image_hsv, CV_BGR2HSV);
  cv::inRange(image_hsv, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), image_thresholded); 
  if (enable_video_display_) {
    cv::imshow(WINDOW_, image_thresholded);
    cv::waitKey(1);
  }

  cv::Mat image_contours = cv::Mat::zeros(image_thresholded.rows, image_thresholded.cols, CV_8UC3); 
  cv::vector< cv::vector<cv::Point> > contours;
  cv::findContours(image_thresholded, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

  if (contours.size() == 0) return;

  cv::Moments mu;
  std::sort(contours.begin(), contours.end(), compare);
  //cv::vector< cv::vector<cv::Point> >::iterator max_iterator = std::max_element(contours.begin(), contours.end(), compare);
  // Consider 10 contours with largest circumference
  int max_area_index = 0;
  double max_area = 0, curr_area;
  for (int i=0; i<std::min(static_cast<int>(contours.size()), 5); ++i) {
    curr_area = cv::contourArea(contours[i], false);
    if (curr_area > max_area) {
      max_area = curr_area;
      max_area_index = i;
    }
  }
  mu = cv::moments(contours[max_area_index], false);
  center = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);
  area = max_area;

  // cv::drawContours(image_contours, contours, max_area_index, cv::Scalar(152,5,85), CV_FILLED);
}
