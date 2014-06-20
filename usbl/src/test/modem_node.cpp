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
 *  Created: 14.02.2013.
 *********************************************************************/
#include <labust/tritech/MTDevice.hpp>
#include <labust/tritech/mtMessages.hpp>
#include <labust/tritech/USBLMessages.hpp>
#include <labust/tritech/DiverMsg_adv.hpp>

#include <std_msgs/String.h>
#include <ros/ros.h>

#include <boost/bind.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>

void onMsg(labust::tritech::MTDevice& modem, const std_msgs::String::ConstPtr msg)
{
	using namespace labust::tritech;

	MTMsgPtr tmsg(new MTMsg());
	tmsg->txNode = 255;
	tmsg->rxNode = labust::tritech::Nodes::SlaveModem;
	tmsg->node = labust::tritech::Nodes::SlaveModem;
	tmsg->msgType = MTMsg::mtMiniModemCmd;

	MMCMsg mmsg;
	mmsg.msgType = labust::tritech::mmcSetRepBits;

	///\todo Consider if network byte order is needed ?
	//for (int i=0;i<6;++i) 	mmsg.data[i+1] = msg->data[i];
	std::copy(msg->data.begin(), msg->data.end(), mmsg.data.begin());

	boost::archive::binary_oarchive ar(*tmsg->data, boost::archive::no_header);
	ar<<mmsg;
	modem.send(tmsg);

	ROS_INFO("Send message.");
};

void onData(ros::Publisher& modemOut, labust::tritech::MTMsgPtr tmsg)
{
	using namespace labust::tritech;

	boost::archive::binary_iarchive dataSer(*tmsg->data, boost::archive::no_header);
	MMCMsg data;
	dataSer>>data;

	ROS_INFO("Received USBL message type %d.", data.msgType);

	/*
	std::cout<<"Received datammcRangedRepByte package:";
	for (int i=0; i<data.data.size(); ++i)
	{
		std::cout<<int(data.data[i])<<",";
	}
	std::cout<<std::endl;*/

	std_msgs::String modem;
	modem.data.assign(data.data.begin(), data.data.end());
	modemOut.publish(modem);
}

int main(int argc, char* argv[])
{
	using namespace labust::tritech;

	ros::init(argc,argv,"modem_node");

	ros::NodeHandle nh,ph("~");

	std::string port("/dev/ttyUSB9");
	int baud(57600);

	ph.param("PortName",port,port);
	ph.param("Baud",baud,baud);

	MTDevice modem(port,baud);

	ros::Subscriber inMsg = nh.subscribe<std_msgs::String>("modem_out",	1,boost::bind(&onMsg,boost::ref(modem),_1));
	ros::Publisher pub = nh.advertise<std_msgs::String>("modem_in",	1);

	MTDevice::HandlerMap map;
	map[MTMsg::mtMiniModemData] = boost::bind(&onData,boost::ref(pub), _1);
	modem.registerHandlers(map);

	ros::spin();

	return 0;
}



