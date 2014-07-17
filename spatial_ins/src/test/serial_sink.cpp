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
 *  Author : Dula Nad
 *  Created: 23.01.2013.
 *********************************************************************/
#include <std_msgs/UInt8MultiArray.h>
#include <ros/ros.h>
#include <boost/asio.hpp>

struct SharedData
{
	SharedData():port(io)
	{};

	~SharedData()
	{
		io.stop();
	}

	void init()
	{
		this->setup_port();
		ros::NodeHandle nh;
		serialIn = nh.subscribe<std_msgs::UInt8MultiArray>("serial_out",1,
				boost::bind(&SharedData::onSerialIn, this,_1));
	}

	bool setup_port()
	{
		ros::NodeHandle ph("~");
		std::string portName("/dev/ttyUSB0");
		int baud(115200);

		ph.param("PortName",portName,portName);
		ph.param("Baud",baud,baud);

		using namespace boost::asio;
		port.open(portName);
		port.set_option(serial_port::baud_rate(baud));
		port.set_option(serial_port::flow_control(
				serial_port::flow_control::none));

		return port.is_open();
	}

	void onSerialIn(const std_msgs::UInt8MultiArray::ConstPtr data)
	{
		boost::asio::write(port, boost::asio::buffer(data->data));
	}

	ros::Subscriber serialIn;
	boost::asio::io_service io;
	boost::asio::serial_port port;
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "serial_sink");
	SharedData data;
	data.init();
	ros::spin();
	return 0;
}
