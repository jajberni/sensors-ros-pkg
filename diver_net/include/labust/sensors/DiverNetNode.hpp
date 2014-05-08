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
#ifndef DIVERNETNODE_HPP_
#define DIVERNETNODE_HPP_
#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/buffer_core.h>

#include <Eigen/Dense>

#include <string>

namespace labust
{
	namespace sensors
	{
		/**
		 * The class implements the Spatial INS driver.
		 * \todo Implement additional messages
		 */
		class DiverNetNode
		{
			enum {headerSize=4, dataPerNode = 18, crc = 2, adc=2};

		public:
			/**
			 * Main constructor
			 */
			DiverNetNode();
			/**
			 * Generic destructor.
			 */
			~DiverNetNode();
			/**
			 * Initialize and setup controller.
			 */
			void onInit();

		private:
			/**
			 * Handle the header detection.
			 */
			void onHeader(const boost::system::error_code& e, std::size_t size);
			/**
			 * Handle the incoming data stream.
			 */
			void onData(const boost::system::error_code& e, std::size_t size);

			/**
			 * Configure the diver net device
			 */
			void configureNet();

			/**
			 * The serial port setup helper method.
			 */
			bool setup_port();
			/**
			 * The start receive helper function.
			 */
			void start_receive();
			/**
			 * The start receive helper function.
			 */
			bool test_sync();

			/**
			 * Publish joint states.
			 */
			void publishJoints(Eigen::MatrixXd& raw);

			/**
			 * The ROS publisher.
			 */
			ros::Publisher rawData, jointsPub, rpyData;
			/**
			 * Hardware i/o service.
			 */
			boost::asio::io_service io;
			/**
			 * The serial input port.
			 */
			boost::asio::serial_port port;
			/**
			 * The main operation thread.
			 */
			boost::thread runner;
			/**
			 * The input buffer.
			 */
			boost::asio::streambuf buffer;
			/**
			 * The header ring buffer.
			 */
			std::vector<uint8_t> ringBuffer;
			/**
			 * The raw data
			 */
			std::vector<uint8_t> rawBuffer;
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
			 * The raw data
			 */
			/**
			 * The number of nodes.
			 */
			int nodeCount;
		};
	}
}

/* DIVERNETNODE_HPP_ */
#endif
