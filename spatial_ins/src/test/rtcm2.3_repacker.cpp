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
#include <rtklib.h>

#include <iosfwd>

struct BitPtr
{
	BitPtr():pointer(std::make_pair(0,0)),ptr(0),bytes(0){};
	template <class T>
	BitPtr(T* ptr, size_t bytes):
		pointer(std::make_pair(0, 0)), ptr(ptr), bytes(bytes){};

	bool operator[](uint32_t idx)
	{
		uint8_t byte = *(ptr + pointer.first + (idx + pointer.second)/8);
		uint8_t bit = 7-(idx + pointer.second)%8;
		//std::cout<<"Ptr:"<<pointer.first<<", bit:"<<uint32_t(bit)<<std::endl;
		//std::cout<<"Values:"<<idx<<", "<<idx+pointer.second<<", "<<(idx + pointer.second)/8<<", "<<(idx + pointer.second)%8;
		//std::cout<<"result bit:"<<(byte & (uint8_t(1) << bit))<<std::endl<<std::endl;
		return byte & (uint8_t(1) << bit);
	}

	template <class T>
	T get(size_t idx, size_t offset = 0)
	{
		T retVal(0);

		for (int i=0; i<idx;++i)
		{
			retVal = retVal | (*this)[i+offset];
			retVal = retVal << 1;
		}

		retVal = retVal >> 1;

		return retVal;
	}

	void operator+=(size_t offset)
	{
		pointer.first += (offset+pointer.second)/8;
		pointer.second = (offset+pointer.second)%8;
	}

	void setBase(uint8_t* ptr, size_t bytes)
	{
		this->ptr = ptr;
		this->bytes = bytes;
		pointer.first = 0;
	}

	uint32_t byteOffset(){return pointer.first;};
	uint32_t bitOffset(){return pointer.first + pointer.second;};

	uint32_t bitsRemaining(){return (bytes - pointer.first)*8 - pointer.second;}

private:
	std::pair<uint32_t, uint8_t> pointer;
	uint8_t* ptr;
	size_t bytes;
};

/*
struct BitStream
{
	BitStream();

	template <class T>
	T get(size_t idx, size_t offset = 0)
	{
		T retVal(0);

		for (int i=0; i<idx;++i)
		{
			retVal = retVal | (*this)[i+offset];
			retVal = retVal << 1;
		}

		retVal = retVal >> 1;

		return retVal;
	}

private:
	BitPtr pt;
	std::vector<uint8_t> data;
};
*/

struct SharedData
{
	enum{RTCM2PREAMB=0x66, headerSize = 10};

	SharedData():port(io), port2(io), missed(0), word1(0), word2(0), newMsg(true)
	{
		//Get some capacity to avoid moving of the memory segment
		ringBuffer.reserve(1024);
		ringBuffer.resize(headerSize);
	};

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

		init_rtcm(&rtcm);

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

		readTime = ros::Time::now();

		std::string portName2("/dev/ttyUSB0");
		int baud2(115200);

		ph.param("PortName2",portName2,portName2);
		ph.param("Baud2",baud2,baud2);

		using namespace boost::asio;
		port2.open(portName2);
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
			buffer.commit(size);
			buffer.sgetn(reinterpret_cast<char*>(ringBuffer.data()),size);

			while (!ringBuffer.empty())
			{
				array_out.data.push_back(ringBuffer[0]);
				if ((input_rtcm2(&rtcm, ringBuffer[0])) > 0)
				{
					std::cout<<"sending data: "<<(rtcm.buff[1] >> 2)<<std::endl;
					//boost::asio::write(port2, boost::asio::buffer(array_out.data));
					rtcmOut.publish(array_out);
					array_out.data.clear();
					newMsg = true;
				}

				ringBuffer.erase(ringBuffer.begin());
			}

			ringBuffer.resize(headerSize);
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
		std::cout<<"Received data size:"<<size<<std::endl;
		if (!e)
		{
			buffer.commit(size);
			array_out.data.resize(headerSize + size);
			std::copy(ringBuffer.begin(), ringBuffer.end(), array_out.data.begin());
			word1 = (ringBuffer[ringBuffer.size() -1]) & 0x3;
			buffer.sgetn(reinterpret_cast<char*>(&array_out.data[headerSize]),size);
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
	BitPtr pt;
	ros::Time readTime;
	boost::asio::serial_port port2;
	uint32_t word1,word2;
	int missed;
	bool newMsg;
	rtcm_t rtcm;

};


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "rtcm_repacker");
	SharedData data;
	data.init();
	ros::spin();
	return 0;
}
