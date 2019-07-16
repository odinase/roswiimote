#include <ros/ros.h>
#include "wiimote_state_estimation/kf_node.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_estimation");
  ros::NodeHandle nh;
  return 0;
}