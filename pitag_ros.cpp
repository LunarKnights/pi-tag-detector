#include "ros/ros.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pitag_ros");

  auto nh = ros::NodeHandle();
  auto nhPrivate = ros::NodeHandle("~");

  // TODO: spawn nodelet

  ros::spin();
  return 0;
}
