#ifndef DIVERNETFILTERNODE_HPP_
#define DIVERNETFILTERNODE_HPP_
#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>

namespace labust 
{
  namespace sensors 
  {
    /**
    * This class implements the complementary IMU filter for Diver net.
    * \todo Implement additional messages
    * \todo Split acquisition and processing into two nodes.
    * \todo Add a Diver Net imu class that will contain name, id, calibration and init options.
    */
    class DiverNetFilterNode {

      public:
        /**
         * Main constructor
         */
        DiverNetFilterNode();
        /**
         * Generic destructor.
         */
        ~DiverNetFilterNode();
        /**
         * Initialize filter.
         */

      private:
        void onInit();
        void processData(const std_msgs::Int16MultiArrayPtr &raw_data);
        void calculateRawAngles(const Eigen::MatrixXd raw);
        void complementaryFilter(const Eigen::MatrixXd raw);
        ros::Subscriber raw_data;
        ros::Publisher raw_angles_publisher, filtered_angles_publisher;
        Eigen::MatrixXd unwrap_count;
        std_msgs::Float64MultiArrayPtr rpy_raw, rpy_filtered;
        const double dT, fs;
        int node_count, data_per_node;
        bool is_initialized;
    };
  }
}

/* DIVERNETFILTERNODE_HPP_ */
#endif
