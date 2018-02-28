#include "ros/ros.h"

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "pitag_ros_nodelet.h"

PLUGINLIB_EXPORT_CLASS(pitag_ros::PitagNodelet, nodelet::Nodelet)

namespace pitag_ros {

PitagNodelet::PitagNodelet()
{
}

void PitagNodelet::onInit()
{
}

}


