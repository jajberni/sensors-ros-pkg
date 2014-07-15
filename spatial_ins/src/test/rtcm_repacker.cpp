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
#include <labust/navigation/BitPacker.hpp>

#include <ros/ros.h>

#include <boost/asio.hpp>
#include <boost/crc.hpp>
#include <boost/thread.hpp>
#include <bitset>

#include <boost/regex.hpp>

#include <iosfwd>

struct SharedData
{
	enum{RTCM3PREAMB=0xD3, headerSize = 3};

	SharedData():port(io), port2(io), ringBuffer(headerSize, 0){};

	~SharedData()
	{
		io.stop();
		runner.join();
	}

	void init()
	{
		this->setup_port();

		ros::NodeHandle nh;
		rtcmOut = nh.advertise<std_msgs::UInt8MultiArray>("rtcm_out",1);

		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
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

		std::string portName2("/dev/ttyUSB0");
		int baud2(115200);

		ph.param("PortName2",portName2,portName2);
		ph.param("Baud2",baud2,baud2);

		using namespace boost::asio;
		port2.open(portName);
		port2.set_option(serial_port::baud_rate(baud));
		port2.set_option(serial_port::flow_control(
				serial_port::flow_control::none));

		return port.is_open();
	}

	void start_receive()
	{
		using namespace boost::asio;
		async_read(port, buffer.prepare(headerSize),
					boost::bind(&SharedData::onHeader, this, _1,_2));
	}

	void onHeader(const boost::system::error_code& e,
			std::size_t size)
	{
		if (!e)
		{
			//std::cout<<"Read: "<<size<<std::endl;
			buffer.commit(size);
			if (size == 1)
			{
				//Put the new byte on the end of the ring buffer
				ringBuffer.push_back(buffer.sbumpc());
			}
			else
			{
				//Copy all data into the buffer
				buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);
			}

			//Check LRC
			if ((ringBuffer[0] == RTCM3PREAMB))
			{
				std::cout<<"Received RTCM3PREAMB."<<std::endl;
				int size = 256*(ringBuffer[1] & 0x03) + ringBuffer[2] + 3;
				std::cout<<"Size:"<<size<<", len1:"<<int(ringBuffer[1])<<", len2:"<<int(ringBuffer[2])<<std::endl;

				array_out.data.resize(size+3);
				array_out.data[0] = ringBuffer[0];
				array_out.data[1] = ringBuffer[1];
				array_out.data[2] = ringBuffer[2];

				boost::asio::async_read(port, buffer.prepare(size),
							boost::bind(&SharedData::onData,this,_1,_2));
				return;
			}
			else
			{
				std::cout<<"No preamble."<<std::endl;
				ringBuffer.erase(ringBuffer.begin());
				boost::asio::async_read(port,
						buffer.prepare(1),
						boost::bind(&SharedData::onHeader,this,_1,_2));
				return;
			}
		}
		else
		{
			ROS_ERROR("RTCM repacker: %s",e.message().c_str());
		}
		this->start_receive();
	}

	void onData(const boost::system::error_code& e,
			std::size_t size)
	{
		//std::cout<<"Received data size:"<<size<<std::endl;
		if (!e)
		{
			buffer.commit(size);
			std::istream is(&buffer);
			is.read(reinterpret_cast<char*>(&array_out.data[3]),size);
			uint32_t type = (uint32_t(array_out.data[3])<<4) + (uint32_t(uint8_t(array_out.data[4]))>>4);
			std::cout<<"Decoded type "<<type<<", b1:"<<int(array_out.data[3])<<", b2:"<<int(array_out.data[4])<<std::endl;

			//Write corrections back to the port
			boost::asio::write(port2, boost::asio::buffer(array_out.data));

			rtcmOut.publish(array_out);


		}
		this->start_receive();
	}

	boost::asio::streambuf buffer;
	std::vector<uint8_t> ringBuffer;
	std_msgs::UInt8MultiArray array_out;
	ros::Publisher rtcmOut;
	boost::thread runner;
	boost::asio::io_service io;
	boost::asio::serial_port port;
	boost::asio::serial_port port2;
};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rtcm_repacker");
	SharedData data;
	data.init();
	ros::spin();
	return 0;
}
