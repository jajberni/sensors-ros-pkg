#include <labust/sensors/DiverNetFilterNode.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

using namespace labust::sensors;

DiverNetFilterNode::DiverNetFilterNode():
    fs(50.0),
    dT(1.0/50),
    node_count(20),
    data_per_node(18),
    is_initialized(false),
    unwrap_count(Eigen::MatrixXd::Zero(node_count, 3)),
    rpy_raw(new std_msgs::Float64MultiArray()),
    rpy_filtered(new std_msgs::Float64MultiArray()) {
  this->onInit();
}

void DiverNetFilterNode::onInit() {
  ros::NodeHandle nh;
  ros::Rate rate(1);

  raw_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_raw", 1);
  filtered_angles_publisher = nh.advertise<std_msgs::Float64MultiArray>("rpy_filtered", 1);
  
  rpy_raw->data.resize(node_count * 3);
  rpy_filtered->data.resize(node_count * 3);

  raw_data = nh.subscribe("net_data", 1, &DiverNetFilterNode::processData, this);
}

DiverNetFilterNode::~DiverNetFilterNode() {}

void DiverNetFilterNode::processData(const std_msgs::Int16MultiArrayPtr &raw_data) {
  const int elem_count(9);

  // Rearange raw data for convenience
  Eigen::MatrixXd raw(node_count,elem_count);
  for (int i=0; i<node_count; ++i) {
    for (int e=0; e<elem_count; ++e) {
      raw(i,e) = raw_data->data[i*elem_count + e];
      raw(i,e) /= (1<<15);
    }
  }  

  calculateRawAngles(raw);
  complementaryFilter(raw);
  
  raw_angles_publisher.publish(rpy_raw);
  filtered_angles_publisher.publish(rpy_filtered);
}

void DiverNetFilterNode::calculateRawAngles(const Eigen::MatrixXd raw) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  for (int i=0; i<node_count; ++i) {
    double roll = atan2(raw(i,ay), raw(i,az));
    double pitch = -atan2(raw(i,ax), sqrt(raw(i,ay) * raw(i,ay) + raw(i,az) * raw(i,az)));
    double hx = raw(i,mx) * cos(pitch) + raw(i,my) * sin(pitch) * sin(roll) + raw(i,mz) * cos(roll) * sin(pitch);
    double hy = raw(i,my) * cos(roll) - raw(i,mz) * sin(roll);
    double yaw = atan2(-hy, hx);

    // Unwrap angles
    //double delta_roll = roll + unwrap_count(i,0) * 2 * M_PI - rpy_filtered->data[3*i];
    //double delta_pitch = pitch + unwrap_count(i,1) * 2 * M_PI - rpy_filtered->data[3*i+1];
    //double delta_yaw = yaw + unwrap_count(i,2) * 2 * M_PI - rpy_filtered->data[3*i+2];

    rpy_raw->data[3*i] = roll;// - (abs(delta_roll > M_PI) ? round(delta_roll/(2*M_PI)) * 2 * M_PI : 0);
    rpy_raw->data[3*i+1] = pitch;// - (abs(delta_pitch > M_PI) ? round(delta_pitch/(2*M_PI)) * 2 * M_PI : 0);
    rpy_raw->data[3*i+2] = yaw;// - (abs(delta_yaw > M_PI) ? round(delta_yaw/(2*M_PI)) * 2 * M_PI : 0);
    //if (abs(delta_roll) > 1.5 * M_PI) unwrap_count(i,0) -= round(delta_roll/(2*M_PI));
    //if (abs(delta_pitch) > 1.5 * M_PI) unwrap_count(i,1) -= round(delta_pitch/(2*M_PI));
    //if (abs(delta_yaw) > 1.5 * M_PI) unwrap_count(i,2) -= round(delta_yaw/(2*M_PI));
    //rpy_raw->data[3*i] = roll + unwrap_count(i,0) * 2 * M_PI;
    //rpy_raw->data[3*i+1] = pitch + unwrap_count(i,1) * 2 * M_PI;
    //rpy_raw->data[3*i+2] = yaw + unwrap_count(i,2) * 2 * M_PI;
  }
}

void DiverNetFilterNode::complementaryFilter(const Eigen::MatrixXd raw) {
  enum {ax,ay,az,mx,my,mz,gx,gy,gz};
  enum {roll,pitch,yaw};

  if (!is_initialized) {
    for (int i=0; i<node_count; ++i) {
      rpy_filtered->data[3*i] = rpy_raw->data[3*i];
      rpy_filtered->data[3*i+1] = rpy_raw->data[3*i+1];
      rpy_filtered->data[3*i+1] = rpy_raw->data[3*i+2];
      is_initialized = true;
      continue;
    }
  }

  // Complementary filter
  const double a = 0.01;
  const double gyro_gain = 2000.0 * M_PI / 180;
  for (int i=0; i<node_count; ++i) {
    double delta_roll = rpy_raw->data[3*i] - rpy_filtered->data[3*i];
    double delta_pitch = rpy_raw->data[3*i+1] - rpy_filtered->data[3*i+1];
    double delta_yaw = rpy_raw->data[3*i+2] - rpy_filtered->data[3*i+2];
    if (abs(delta_roll) > M_PI) rpy_raw->data[3*i] -= round(delta_roll/(2*M_PI)) * 2 * M_PI;
    if (abs(delta_pitch) > M_PI) rpy_raw->data[3*i+1] -= round(delta_pitch/(2*M_PI)) * 2 * M_PI;
    if (abs(delta_yaw) > M_PI) rpy_raw->data[3*i+2] -= round(delta_yaw/(2*M_PI)) * 2 * M_PI;

    double a1 = 0.02 - 0.02 * abs(rpy_raw->data[3*i] - rpy_filtered->data[3*i]) / (M_PI);
    double a2 = 0.02 - 0.02 * abs(rpy_raw->data[3*i+1] - rpy_filtered->data[3*i+1]) / (M_PI);
    double a3 = 0.02 - 0.02 * abs(rpy_raw->data[3*i+2] - rpy_filtered->data[3*i+2]) / (M_PI);
    rpy_filtered->data[3*i] = (1-a1)*(rpy_filtered->data[3*i] + raw(i,gx)*dT*gyro_gain) + a1*rpy_raw->data[3*i];
    rpy_filtered->data[3*i+1] = (1-a2)*(rpy_filtered->data[3*i+1] + raw(i,gy)*dT*gyro_gain) + a2*rpy_raw->data[3*i+1];
    rpy_filtered->data[3*i+2] = (1-a3)*(rpy_filtered->data[3*i+2] + raw(i,gz)*dT*gyro_gain) + a3*rpy_raw->data[3*i+2];
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "diver_net_filter_node");
  DiverNetFilterNode node;
  ros::spin();
  return 0;
}
