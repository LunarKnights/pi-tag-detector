#include "ros/ros.h"

#include <nodelet/loader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pitag_ros");

  // spawn nodelet
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  nodelet.load(ros::this_node::getName(),
          "pitag_ros/PitagNodelet",
          remap, nargv);

  ros::spin();
  return 0;
}
