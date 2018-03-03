#include "ros/ros.h"

#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>

#include "pitag_ros_nodelet.h"

PLUGINLIB_EXPORT_CLASS(pitag_ros::PitagNodelet, nodelet::Nodelet)

namespace pitag_ros {

PitagNodelet::PitagNodelet():
  tagDetector(new ipa_Fiducials::FiducialModelPi()),
  cameraInfoSet(false)
{
  // do nothing
}

void PitagNodelet::onInit()
{
  auto nhPrivate = getPrivateNodeHandle();
  it = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nhPrivate));
  imageSub = it->subscribe("camera", 1, &PitagNodelet::imageCb, this);
  camInfoSub = nhPrivate.subscribe("camera_info", 1, &PitagNodelet::cameraInfoCb, this);

  // store an empty matrix for now
  cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
  std::string modelFilename;
  if (!nhPrivate.getParam("model_path", modelFilename))
  {
    NODELET_FATAL("model_path must be provided");
    return;
  }
  
  if (tagDetector->Init(cameraMatrix, modelFilename, false) & RET_FAILED)
  {
    NODELET_FATAL("initializing fiducial detector with camera matrix [FAILED]");
    return;
  }
}

void PitagNodelet::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (cameraInfoSet)
  {
    return;
  }

  auto &toCopy = msg->K;
  std::array<float, toCopy.static_size> copiedMatrix;
  std::copy(toCopy.begin(), toCopy.end(), copiedMatrix.begin());

  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32F, copiedMatrix.data());
  tagDetector->SetCameraMatrix(cameraMatrix);
  cameraInfoSet = true;
}

void PitagNodelet::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  // wait for camera info
  if (!cameraInfoSet)
  {
    return;
  }
  try
  {
    auto cvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    std::vector<ipa_Fiducials::t_pose> poses;
    if (tagDetector->GetPose(cvPtr->image, poses) & RET_FAILED)
    {
      // silently fail; GetPose fails if no fiducials are visible, which will happen a lot
      return;
    }
    // TODO: publish poses
  }
  catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  catch(cv::Exception & e) {
    ROS_ERROR("cv exception: %s", e.what());
  }
}


}
