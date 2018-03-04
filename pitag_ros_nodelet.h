#ifndef PITAG_ROS_NODELET_H_
#define PITAG_ROS_NODELET_H_

#include "ros/ros.h"

#include <memory>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "FiducialDefines.h"
#include "FiducialModelPi.h"

namespace pitag_ros {

class PitagNodelet: public nodelet::Nodelet
{
public:
  PitagNodelet();
  void onInit() override;
protected:
  void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void imageCb(const sensor_msgs::ImageConstPtr &msg);

  std::shared_ptr<ipa_Fiducials::AbstractFiducialModel> tagDetector;
  std::shared_ptr<image_transport::ImageTransport> it;
  ros::Subscriber camInfoSub;
  image_transport::Subscriber imageSub;
  image_transport::Publisher debugPub;
  ros::Publisher posePub;

  std::string cameraFrame;

  bool cameraInfoSet;
  bool publishDebugFrame;
};

}

#endif
