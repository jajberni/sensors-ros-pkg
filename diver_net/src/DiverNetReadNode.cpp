#include <labust/sensors/DiverNetReadNode.hpp>
#include <labust/tools/conversions.hpp>

#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>

#include <Eigen/Dense>

#include <boost/bind.hpp>
#include <boost/crc.hpp>

#include <iosfwd>

using namespace labust::sensors;

DiverNetReadNode::DiverNetReadNode():
							io(),
							port(io),
							ringBuffer(headerSize,0),
							nodeCount(20)
{
	this->onInit();
}

DiverNetReadNode::~DiverNetReadNode()
{
	io.stop();
	runner.join();
}

void DiverNetReadNode::onInit()
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
	breathingBelt = nh.advertise<std_msgs::Int16>("breathing_belt",1);

	netInit = nh.subscribe<std_msgs::Bool>("net_init",1,&DiverNetReadNode::onNetInit, this);

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

void DiverNetReadNode::configureNet()
{
	//Read number of nodes
	ros::NodeHandle ph("~");
	ph.param("node_count", nodeCount, nodeCount);
	rawBuffer.resize(nodeCount * dataPerNode + crc + adc);
}

void DiverNetReadNode::start_receive()
{
	using namespace boost::asio;
	async_read(port, buffer.prepare(headerSize),
			boost::bind(&DiverNetReadNode::onHeader, this, _1,_2));
}

bool DiverNetReadNode::setup_port()
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

void DiverNetReadNode::onHeader(const boost::system::error_code& e,
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
					boost::bind(&DiverNetReadNode::onData,this,_1,_2));
			return;
		}
		else
		{
			std::cout<<"Sync failed."<<std::endl;
			ringBuffer.erase(ringBuffer.begin());
			boost::asio::async_read(port,
					buffer.prepare(1),
					boost::bind(&DiverNetReadNode::onHeader,this,_1,_2));
			return;
		}
	}
	else
	{
		ROS_ERROR("DiverNetNode: %s",e.message().c_str());
	}
	this->start_receive();
}

bool DiverNetReadNode::test_sync()
{
	std::string header(reinterpret_cast<char*>(ringBuffer.data()),headerSize);

	std::cout<<"SyncData:"<<header<<std::endl;

	//Sometimes the first byte fails.
	return (header == "D1Ve") || (header.substr(1) == "1Ve");
}

void DiverNetReadNode::onData(const boost::system::error_code& e,
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
                        std_msgs::Int16Ptr bbelt(new std_msgs::Int16());
                        bbelt->data = (rawBuffer[nodeCount * dataPerNode] + 256 * rawBuffer[nodeCount * dataPerNode + 1]);
                        out->data.push_back(bbelt->data);
                        breathingBelt.publish(bbelt);
			rawData.publish(out);
		}
		else
		{
			std::cout<<"Data CRC failed."<<std::endl;
		}
	}
	this->start_receive();
}

void DiverNetReadNode::onNetInit(const std_msgs::Bool::ConstPtr& init) {
    if (init->data) {
      boost::mutex::scoped_lock l(dataMux);
    }
}


int main(int argc, char* argv[])
{
	ros::init(argc,argv,"diver_net_read_node");
	DiverNetReadNode node;
	ros::spin();

	return 0;
}


