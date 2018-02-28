#ifndef PITAG_ROS_NODELET_H_
#define PITAG_ROS_NODELET_H_

#include "ros/ros.h"

#include <nodelet/nodelet.h>

namespace pitag_ros {

class PitagNodelet: public nodelet::Nodelet
{
public:
  PitagNodelet();
  void onInit() override;
protected:
};

}

#endif
