#include <ros/ros.h>
#include "wiimote_state_estimation/state_estimation_node.h"
#include "wiimote_state_estimation/kf.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimation");
  ros::NodeHandle nh;
  KalmanFilter<3, 1, 3> kf;
  StateEstimation_node se_node("/android/imu", kf, nh);
  se_node.run();
  return 0;
}