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
#include <labust/sensors/DiverNetNode.hpp>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <iosfwd>

using namespace labust::sensors;

DiverNetNode::DiverNetNode():
							io(),
							port(io),
							ringBuffer(headerSize,0),
							nodeCount(20),
							listener(tfbuffer)
{
	this->onInit();
}

DiverNetNode::~DiverNetNode()
{
	io.stop();
	runner.join();
}

void DiverNetNode::onInit()
{
	ros::NodeHandle nh, ph("~");
	ros::Rate r(1);
	bool setupOk(false);
	while (!(setupOk = this->setup_port()) && ros::ok())
	{
		ROS_ERROR("DiverNetNode::Failed to open port.");
		r.sleep();
	}

	//Setup publisher
	rawData = nh.advertise<std_msgs::Int16MultiArray>("net_data",1);
	rpyData = nh.advertise<std_msgs::Float64MultiArray>("rpy_data",1);
	jointsPub = nh.advertise<sensor_msgs::JointState>("joint_states",1);

	if (setupOk)
	{
		ROS_INFO("DiverNet is connected.");
		//Configure the device
		this->configureNet();
		//Start the receive cycle
		this->start_receive();
		runner = boost::thread(boost::bind(&boost::asio::io_service::run,&io));
	}
}

void DiverNetNode::configureNet()
{
	//Read number of nodes
	ros::NodeHandle ph("~");
	ph.param("node_count", nodeCount, nodeCount);
	rawBuffer.resize(nodeCount * dataPerNode + crc + adc);
	//Connect imu number with joint
	//Setup acc,gyro,mag ranges
}

void DiverNetNode::start_receive()
{
	using namespace boost::asio;
	async_read(port, buffer.prepare(headerSize),
			boost::bind(&DiverNetNode::onHeader, this, _1,_2));
}

bool DiverNetNode::setup_port()
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

void DiverNetNode::onHeader(const boost::system::error_code& e,
		std::size_t size)
{
	if (!e)
	{
		//std::cout<<"Header read size:"<<size<<std::endl;
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
		if (test_sync())
		{
			std::cout<<"Sync ok."<<std::endl;
			boost::asio::async_read(port, boost::asio::buffer(rawBuffer),
					boost::bind(&DiverNetNode::onData,this,_1,_2));
			return;
		}
		else
		{
			std::cout<<"Sync failed."<<std::endl;
			ringBuffer.erase(ringBuffer.begin());
			boost::asio::async_read(port,
					buffer.prepare(1),
					boost::bind(&DiverNetNode::onHeader,this,_1,_2));
			return;
		}
	}
	else
	{
		ROS_ERROR("DiverNetNode: %s",e.message().c_str());
	}
	this->start_receive();
}

bool DiverNetNode::test_sync()
{
	std::string header(reinterpret_cast<char*>(ringBuffer.data()),headerSize);

	std::cout<<"SyncData:"<<header<<std::endl;

	//Sometimes the first byte fails.
	return (header == "D1Ve") || (header.substr(1) == "1Ve");
}

void DiverNetNode::onData(const boost::system::error_code& e,
		std::size_t size)
{
	//std::cout<<"Received data size:"<<size<<std::endl;
	if (!e)
	{
		//buffer.commit(size);
		//std::istream is(&buffer);

		//std::string data(size+headerSize,'\0');
		//is.read(&data[headerSize],size);

		//std::string data(size,'\0');
		//is.read(&data[0],size);

		/*data[0] = 'D';
		data[1] = '1';
		data[2] = 'V';
		data[3] = 'e';*/

		boost::crc_optimal<16, 0xA001> result;
		uint16_t crc_test = 256*rawBuffer[rawBuffer.size()-1]+rawBuffer[rawBuffer.size()-2];

		/*		std::cout<<"data:";
		for (int i=0; i<rawBuffer.size(); ++i)
		{
			std::cout<<int(rawBuffer[i])<<", ";
			if (((i+1)%nodeCount == 0) && (i!=0)) std::cout<<std::endl;
		}
		std::cout<<std::endl;*/

		result.process_bytes(rawBuffer.data(),rawBuffer.size()-2);
		std::cout<<"CRC: "<<crc_test<<",";
		std::cout<<256*uint8_t(rawBuffer[rawBuffer.size()-2])+uint8_t(rawBuffer[rawBuffer.size()-1])<<","<<result.checksum()<<std::endl;

		if (true || (result.checksum() == crc_test))
		{
			//Process tested data
			//This can be done better
			std::cout<<"CRC ok."<<std::endl;
			std_msgs::Int16MultiArrayPtr out(new std_msgs::Int16MultiArray());
			out->data.resize((nodeCount * dataPerNode)/2);
			Eigen::MatrixXd raw(nodeCount,9);

			int elemCount = 9;
			for (int i=0; i<nodeCount; ++i)
			{
				for (int e=0; e<elemCount; ++e)
				{
					raw(i,e) = out->data[i*elemCount + e] = rawBuffer[2*e*nodeCount + i] +
							256*rawBuffer[(2*e+1)*nodeCount + i];
					raw(i,e) /= (1 << 15);
				}
			}
			rawData.publish(out);

			publishJoints(raw);
		}
		else
		{
			std::cout<<"Data CRC failed."<<std::endl;
		}
	}
	this->start_receive();
}

void DiverNetNode::publishJoints(Eigen::MatrixXd& raw)
{
	enum {ax,ay,az,mx,my,mz,gx,gy,gz};

	std_msgs::Float64MultiArrayPtr rpy(new std_msgs::Float64MultiArray());
	rpy->data.resize(nodeCount * 3);

	sensor_msgs::JointStatePtr joints(new sensor_msgs::JointState());
	joints->name.resize(nodeCount * 3);
	joints->position.resize(nodeCount * 3);
	joints->velocity.resize(nodeCount * 3);
	joints->effort.resize(nodeCount * 3);


	std::vector<std::string> names;
	names.push_back("right_blade");
	names.push_back("right_upper_arm");
	names.push_back("right_forearm");
	names.push_back("right_hand");
	names.push_back("head");

	names.push_back("left_blade");
	names.push_back("left_upper_arm");
	names.push_back("left_forearm");
	names.push_back("left_hand");
	names.push_back("upper_body");

	names.push_back("placehodler_1");

	names.push_back("left_thigh");
	names.push_back("left_calf");
	names.push_back("left_leg");
	names.push_back("lower_back");

	names.push_back("right_thigh");
	names.push_back("right_calf");
	names.push_back("right_leg");

	names.push_back("placeholder_2");
	names.push_back("placeholder_3");

	for (int i=0; i<nodeCount; ++i)
	{
		Eigen::VectorXd curraw = raw.row(i);
		//Unnormalized
		double gmax = raw.row(i).head<3>().norm();
		/*std::cout<<"ax: "<<curraw(ax);
		std::cout<<", ay: "<<curraw(ay);
		std::cout<<", az: "<<curraw(az);
		std::cout<<", gmax: "<<gmax<<std::endl;*/

		double roll = atan2(curraw(ay), curraw(az));
		double pitch = atan2(curraw(ax), sqrt(curraw(ay)*curraw(ay) + curraw(az)*curraw(az)));
		double hx = curraw(mx)*cos(pitch)+curraw(my)*sin(pitch)*sin(roll)+curraw(mz)*cos(roll)*sin(pitch);
		double hy = curraw(my)*cos(roll)-curraw(mz)*sin(roll);
		double yaw = atan2(-hy, hx);
		rpy->data[3*i] = roll;
		rpy->data[3*i+1] = pitch;
		rpy->data[3*i+2] = yaw;

		if (i<3)
		{
			/*joints->name[3*i] = names[i] + "_x";
			joints->name[3*i+1] = names[i] + "_y";
			joints->name[3*i+2] = names[i] + "_z";
			joints->position[3*i] = roll;
			joints->position[3*i+1] = pitch;
			joints->position[3*i+2] = yaw;*/


			geometry_msgs::TransformStamped transform;
			transform.transform.translation.x = 0;
			transform.transform.translation.y = 0;
			transform.transform.translation.z = 0;
			labust::tools::quaternionFromEulerZYX(roll, pitch, yaw,
					transform.transform.rotation);
			transform.child_frame_id = "abs_" + names[i];
			transform.header.frame_id = "local";
			transform.header.stamp = ros::Time::now();
			broadcast.sendTransform(transform);
		}
	}

	for (int i=0; i<nodeCount; ++i)
	{
		if (i<3)
		{
			std::string parent;
			double roll(0),pitch(0),yaw(0);
			if (tfbuffer._getParent(names[i]+"_2", ros::Time(0), parent))
			{
				std::cout<<"found parent:"<<parent<<std::endl;
				geometry_msgs::TransformStamped trans;
				if (std::find(names.begin(), names.end(), parent) != names.end())
				{
					 std::cout<<"cross transform"<<parent<<std::endl;
					 //trans = tfbuffer.lookupTransform("abs_"+names[i], "abs_"+parent, ros::Time(0));
					 trans = tfbuffer.lookupTransform("abs_"+parent, "abs_"+names[i], ros::Time(0));
				}
				else
				{
					//trans = tfbuffer.lookupTransform("abs_"+names[i], "local", ros::Time(0));
					trans = tfbuffer.lookupTransform("local", "abs_"+names[i], ros::Time(0));
				}
				labust::tools::eulerZYXFromQuaternion(trans.transform.rotation,
						roll, pitch, yaw);
			}

			joints->name[3*i] = names[i] + "_x";
			joints->name[3*i+1] = names[i] + "_y";
			joints->name[3*i+2] = names[i] + "_z";
			joints->position[3*i] = roll;
			joints->position[3*i+1] = pitch;
			joints->position[3*i+2] = yaw;
		}
	}

	//Process joints
	//Stamp data
	joints->header.stamp = ros::Time::now();
	//Publish data
	rpyData.publish(rpy);
	jointsPub.publish(joints);
}

int main(int argc, char* argv[])
{
	ros::init(argc,argv,"diver_net_node");
	DiverNetNode node;
	ros::spin();

	return 0;
}


