#ifndef WIIMOTE_STATE_ESTIMATION_KF_NODE_H
#define WIIMOTE_STATE_ESTIMATION_KF_NODE_H

#include <ros/ros.h>

class KalmanFilter_node
{
public:
  KalmanFilter_node();

private:
  ros::Subscriber imuSub;
  ros::Publisher imuFilteredPub;
};

#endif // WIIMOTE_STATE_ESTIMATION_KF_NODE_H