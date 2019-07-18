#ifndef WIIMOTE_STATE_ESTIMATION_KF_NODE_H
#define WIIMOTE_STATE_ESTIMATION_KF_NODE_H

#include <ros/ros.h>
#include <string>
#include <sensor_msgs/Imu.h>
#include "wiimote_state_estimation/filter_method.h"

template <unsigned int NX, unsigned int NU, unsigned int NZ>
class StateEstimation_node
{
public:
  StateEstimation_node(FilterMethod<NX, NU, NZ> &fm, ros::NodeHandle &nh);
  StateEstimation_node(const std::string &node, FilterMethod<NX, NU, NZ> &fm, ros::NodeHandle &nh);
  FilterMethod<NX, NU, NZ> &fm;
  void run();

private:
  ros::Subscriber imuSub;
  ros::Publisher imuFilteredPub;
  void imuCb(const sensor_msgs::Imu &imuMsg);
};

template <unsigned int NX, unsigned int NU, unsigned int NZ>
StateEstimation_node<NX, NU, NZ>::StateEstimation_node(FilterMethod<NX, NU, NZ> &fm, ros::NodeHandle &nh)
    : fm(fm)
{
  imuSub = nh.subscribe("/android/imu", 1000, &StateEstimation_node<NX, NU, NZ>::imuCb, this);
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
StateEstimation_node<NX, NU, NZ>::StateEstimation_node(const std::string &node, FilterMethod<NX, NU, NZ> &fm, ros::NodeHandle &nh)
    : fm(fm)
{
  imuSub = nh.subscribe(node, 1000, &StateEstimation_node<NX, NU, NZ>::imuCb, this);
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void StateEstimation_node<NX, NU, NZ>::imuCb(const sensor_msgs::Imu &imuMsg)
{
  ROS_INFO("Received data!");
}

template <unsigned int NX, unsigned int NU, unsigned int NZ>
void StateEstimation_node<NX, NU, NZ>::run()
{
  ros::Rate lr{2};

  while (ros::ok())
  {
    ros::spinOnce();
  }
}

#endif // WIIMOTE_STATE_ESTIMATION_KF_NODE_H