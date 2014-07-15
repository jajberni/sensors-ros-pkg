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
#include <labust/sensors/image/HumanHogDetector.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace labust::sensors::image;

HumanHogDetector::HumanHogDetector() {
  WINDOW_ = "Human HOG detector";
  enable_video_display_ = false;

  hog_descriptor_.setSVMDetector(cv::HOGDescriptor::getDefaultPeopleDetector());
}

void HumanHogDetector::setEnableVideoDisplay(bool enable_video_display) {
  enable_video_display_ = enable_video_display;
  this->createOpenCvWindow();
}

void HumanHogDetector::createOpenCvWindow() {
  cv::namedWindow(WINDOW_);
  cv::waitKey(1);
}

HumanHogDetector::~HumanHogDetector() {
  cv::destroyWindow(WINDOW_);
}

void HumanHogDetector::detect(cv::Mat &image, cv::Point2f &center, double &area) {
  cv::vector<cv::Rect> found, found_filtered;

  hog_descriptor_.detectMultiScale(image, found, 0, cv::Size(8,8), cv::Size(32,32), 1.05, 2);
  for (int i=0; i<found.size(); ++i) {
    cv::Rect r = found[i];
    int j;
    for (j=0; j<found.size(); ++j) {
      if (j!=i && (r & found[j])==r) break;
    }
    if (j==found.size()) found_filtered.push_back(r);
  }

  for (int i=0; i<found.size(); ++i) {
    cv::Rect r = found_filtered[i];
    r.x += cvRound(r.width*0.1);
    r.width = cvRound(r.width*0.8);
    r.y += cvRound(r.height*0.06);
    r.height = cvRound(r.height*0.9);
    cv::rectangle(image, r.tl(), r.br(), cv::Scalar(0,255,0), 2);
  }

  if (enable_video_display_) {
    cv::imshow(WINDOW_, image);
    cv::waitKey(1);
  }
}
