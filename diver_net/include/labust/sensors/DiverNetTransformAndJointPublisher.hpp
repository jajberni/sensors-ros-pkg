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
#ifndef DIVERNETTRANSFORMANDJOINTPUBLISHER_HPP_
#define DIVERNETTRANSFORMANDJOINTPUBLISHER_HPP_
#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>

#include <Eigen/Dense>

#include <string>

namespace labust
{
	namespace sensors
	{
		/**
		 * The class implements the Diver net driver.
		 * \todo Implement additional messages
		 * \todo Split acquisition and processing into two nodes.
		 * \todo Add a Diver Net imu class that will contain name, id, calibration and init options.
		 */
		class DiverNetTransformAndJointPublisher
		{
			enum {headerSize=4, dataPerNode = 18, crc = 2, adc=2};

		public:
			/**
			 * Main constructor
			 */
			DiverNetTransformAndJointPublisher();
			/**
			 * Generic destructor.
			 */
			~DiverNetTransformAndJointPublisher();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			/**
			 * Configure the diver net device
			 */
			void configureNet();
			/**
			 * Publish joint states.
			 */
			void publishTransformAndJoints(const std_msgs::Float64MultiArrayPtr &rpy);
			/**
			 * Handle net init.
			 */
			void onNetInit(const std_msgs::Bool::ConstPtr& init);

			/**
			 * The ROS publisher.
			 */
			ros::Publisher rawData, jointsPub, rpyData, breathingBelt;
                        ros::Subscriber rpy;
			/**
			 * The transform broadcaster.
			 */
			tf2_ros::TransformBroadcaster broadcast;
			/**
			 * Transform buffer.
			 */
			tf2::BufferCore tfbuffer;
			/**
			 * The transform listener.
			 */
			tf2_ros::TransformListener listener;
			/**
			 * The number of nodes.
			 */
			int nodeCount;
			/**
			 * The calibration and zero state values.
			 */
			Eigen::MatrixXd offset, zeroState, currentMeas;
			/**
			 * Joint names connected to IMU numbers.
			 * \todo Consider if a map is better option ?
			 */
			std::vector<std::string> names;
			/**
			 * Data mutex.
			 */
			boost::mutex dataMux;
                        std::string _rpy_topic;
		};
	}
}

/* DIVERNETTRANSFORMANDJOINTPUBLISHER_HPP_ */
#endif
