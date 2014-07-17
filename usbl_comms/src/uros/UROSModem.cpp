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
 *  Author: Dula Nad
 *  Created: 02.04.2013.
 *********************************************************************/
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <auv_msgs/NavSts.h>
#include <std_msgs/Bool.h>

#include <labust/tritech/DiverMsg.hpp>

#include <ros/ros.h>

#include <boost/bind.hpp>

using namespace labust::tritech;

struct SharedData
{
	enum {ABORT = 1};

	SharedData():initCnt(0), waitForInit(true), useExtLatLon(false), inited(false), gain(0), sendPeriod(1.0)
	{
		dispatch[DiverMsg::PositionInit] = boost::bind(&SharedData::onPositionInit, this);
		dispatch[DiverMsg::Position_18] = boost::bind(&SharedData::onNewPosition, this);
		dispatch[DiverMsg::Position_14Def] = boost::bind(&SharedData::onPositionDef, this);

		ros::NodeHandle ph("~");
		ph.param("useExtLatLon",useExtLatLon, useExtLatLon);
		ph.param("waitForInit",waitForInit, waitForInit);
		inited = !waitForInit;
	};

	void onPositionInit()
	{
		//if (!useExtLatLon)
		{
			ROS_INFO("Init message.");
			//Send back init
			std::cout<<"Sending back init:"<<incoming_package.data<<std::endl;
			msgOut.publish(incoming_package);
			bits2latlon.convert(incoming_msg.data[DiverMsg::lat],
				incoming_msg.data[DiverMsg::lon], 22);
			++initCnt;
			//inited = true;
		}
	}

	void onNewPosition()
	{
		ROS_INFO("New position-18 message.");
		decodePosition(18);
	}

	void onPositionDef()
	{
		
		ROS_INFO("New position-def message.");
		decodePosition(14);
		std::cout<<"Got position def:"<<incoming_msg.data[DiverMsg::def]<<std::endl;

		std_msgs::Bool flag;
		switch (incoming_msg.data[DiverMsg::def])
		{
		case ABORT:
			ROS_INFO("Aborting mission.");
			flag.data = true;
			abortOut.publish(flag);
			break;
		default:
			ROS_WARN("Unknown default message");
			break;
		}
	}

	void decodePosition(int bits)
	{
		if (inited)
		{
			bits2latlon.convert(incoming_msg.data[DiverMsg::lat],
					incoming_msg.data[DiverMsg::lon],bits);
			if (!useExtLatLon)
			{
				outgoing_msg.latitude = bits2latlon.latitude;
				outgoing_msg.longitude = bits2latlon.longitude;
			}
		}
	}

	DiverMsg outgoing_msg, incoming_msg;
	std_msgs::String incoming_package;
	ros::Publisher msgOut, abortOut;
	std::map<int, boost::function<void(void)> > dispatch;
	bool waitForInit, useExtLatLon, inited;
	Bits2LatLon bits2latlon;
	ros::Time lastSent;
	double sendPeriod;
	int gain, initCnt;
	bool gainSync;
};

void onModem(SharedData& data, const std_msgs::String::ConstPtr msg)
{
	ROS_INFO("Received modem message with type: %d",DiverMsg::testType(msg->data));
	std::cout<<"Received modem message with type: "<<int(DiverMsg::testType(msg->data))<<std::endl;

	try
	{
		data.incoming_package.data = msg->data;
		data.incoming_msg.fromString<DiverMsg::AutoTopside>(msg->data);
		int msg_type = data.incoming_msg.data[DiverMsg::type];
		if (data.dispatch.find(msg_type) != data.dispatch.end())
		{
			if (msg_type == DiverMsg::PositionInit)
			{
				//Restart the init sequence
				if (data.inited)
				{ 
					data.inited = false;
					data.initCnt = 0;
				}
			}
			else if (!data.inited)
			{
				++data.initCnt;
				data.inited = (data.initCnt > 2);
			}
			data.dispatch[msg_type]();
		}
		else
		{
			ROS_ERROR("No handler for message type %d",msg_type);
		}

	}
	catch (std::exception& e)
	{
		ROS_ERROR("Exception caught on incoming msg: %s",e.what());
	}
	
	if (data.inited)
	{
		std::cout<<"Send data"<<std::endl;
		//Send back newest data
		std_msgs::String out;
		out.data = data.outgoing_msg.toString<DiverMsg::AutoDiver>();
		std::cout<<"Latitude:"<<data.outgoing_msg.latitude<<", longitude:"<<data.outgoing_msg.longitude<<std::endl;
		std::cout<<"Lat:"<<data.outgoing_msg.data[DiverMsg::lat]<<", lon:"<<data.outgoing_msg.data[DiverMsg::lon]<<std::endl;
		data.msgOut.publish(out);
	}
}

void onNavSts(SharedData& data, const auv_msgs::NavSts::ConstPtr nav)
{
	if (data.useExtLatLon)
	{
		data.outgoing_msg.latitude = nav->global_position.latitude;
		data.outgoing_msg.longitude = nav->global_position.longitude;
	};
}

void onGainIn(SharedData& data, const std_msgs::Int32::ConstPtr gain)
{
	data.gain = log10(gain->data);
}

void onAdcIn(SharedData& data, const std_msgs::Float32::ConstPtr adc)
{
  if (!data.inited && !data.useExtLatLon) return;
	//std::cout<<"New adc ok"<<std::endl;
	data.lastSent = ros::Time::now();
	data.outgoing_msg.data[DiverMsg::msg] = adc->data * pow(10,data.gain);
	//std::cout<<"Packed adc:"<<data.outgoing_msg.data[DiverMsg::msg]<<std::endl;
	data.outgoing_msg.data[DiverMsg::msg] |= ((data.gain & 0x3) << 10);
	//std::cout<<"Packed adc + gain:"<<data.outgoing_msg.data[DiverMsg::msg]<<std::endl;
	//std::cout<<"Gain:"<<data.gain<<std::endl;
	data.outgoing_msg.data[DiverMsg::type] = DiverMsg::Rhodamine;
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"uros_modem");
	
	SharedData data;
	data.outgoing_msg.latitude = 42.12345678;
	data.outgoing_msg.longitude = 15.12345678;
	ros::NodeHandle nh;
	ros::Subscriber adcin = nh.subscribe<std_msgs::Float32>("adc_in",1,boost::bind(&onAdcIn, boost::ref(data), _1));
	ros::Subscriber gainin = nh.subscribe<std_msgs::Int32>("gain_in",1,boost::bind(&onGainIn, boost::ref(data), _1));
	ros::Subscriber navin = nh.subscribe<auv_msgs::NavSts>("nav_in",1,boost::bind(&onNavSts, boost::ref(data), _1));
	ros::Subscriber modemin = nh.subscribe<std_msgs::String>("modem_in",1,boost::bind(&onModem, boost::ref(data), _1));
	data.msgOut = nh.advertise<std_msgs::String>("modem_out", 1);
	data.abortOut = nh.advertise<std_msgs::Bool>("abort",1);

	ros::spin();
	
	return 0;
}
