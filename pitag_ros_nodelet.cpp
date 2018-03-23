#include "ros/ros.h"

#include <algorithm>

#include <opencv2/core.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <fiducial_msgs/FiducialTransform.h>
#include <fiducial_msgs/FiducialTransformArray.h>

#include "pitag_ros_nodelet.h"

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

  std::string cameraTopicName, cameraInfoTopicName;
  nhPrivate.param<std::string>("camera_topic", cameraTopicName, "camera");
  nhPrivate.param<std::string>("camera_info_topic", cameraInfoTopicName, "camera_info");

  it = std::shared_ptr<image_transport::ImageTransport>(
      new image_transport::ImageTransport(nhPrivate));
  imageSub = it->subscribe(cameraTopicName, 1, &PitagNodelet::imageCb, this);
  camInfoSub = nhPrivate.subscribe(cameraInfoTopicName, 1, &PitagNodelet::cameraInfoCb, this);
  posePub = nhPrivate.advertise<fiducial_msgs::FiducialTransformArray>("fiducials", 1);

  // store an empty matrix for now
  cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
  std::string modelFilename;
  if (!nhPrivate.getParam("model_path", modelFilename))
  {
    NODELET_FATAL("model_path must be provided");
    return;
  }
  nhPrivate.param<bool>("publish_debug", publishDebugFrame, false);
  
  if (tagDetector->Init(cameraMatrix, modelFilename, false) & RET_FAILED)
  {
    NODELET_FATAL("initializing fiducial detector with camera matrix [FAILED]");
    return;
  }

  if (publishDebugFrame)
  {
    debugPub = it->advertise("debug_image", 1);
  }
}

void PitagNodelet::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  if (cameraInfoSet)
  {
    return;
  }

  auto &toCopy = msg->K;
  std::array<double, toCopy.static_size> copiedMatrix;
  std::copy(toCopy.begin(), toCopy.end(), copiedMatrix.begin());

  cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, copiedMatrix.data());
  tagDetector->SetCameraMatrix(cameraMatrix);

  cameraFrame = msg->header.frame_id;
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
    auto cvPtr = cv_bridge::toCvCopy(msg);
    std::vector<ipa_Fiducials::t_pose> poses;
    if (tagDetector->GetPose(cvPtr->image, poses) & RET_FAILED)
    {
      // silently fail; GetPose fails if no fiducials are visible, which will happen a lot
      return;
    }
    if (poses.size() > 0)
    {
      // publish poses
      fiducial_msgs::FiducialTransformArray fta;
      fta.header.stamp = msg->header.stamp;
      fta.header.frame_id = cameraFrame;
      fta.image_seq = msg->header.seq;
      for (const auto &pose: poses)
      {
        fiducial_msgs::FiducialTransform ft;
        ft.fiducial_id = pose.id;
        tf2::Matrix3x3 rotation;

        // assumptions about pose structure; pose.rot is a 3x3 matrix,
        // pose.trans is a 3x1 vector
        if (pose.rot.rows != 3 || pose.rot.cols != 3)
        {
          ROS_FATAL("pitag pose rotation is invalid size!");
          return;
        }
        if (pose.trans.rows != 3 || pose.trans.cols != 1)
        {
          ROS_FATAL("pitag pose translation is invalid size!");
          return;
        }

        for (int i = 0; i < 3; ++i)
        {
          for (int j = 0; j < 3; ++j)
          {
            rotation[i][j] = pose.rot.at<double>(i, j);
          }
        }

        const tf2::Vector3 translation(
            pose.trans.at<double>(0, 0),
            pose.trans.at<double>(0, 1),
            pose.trans.at<double>(0, 2));
        tf2::Quaternion quatRotation;
        rotation.getRotation(quatRotation);

        const tf2::Transform transform = tf2::Transform(quatRotation, translation);
        ft.transform = tf2::toMsg(transform);

        // NOTE: these aren't provided by the library, maybe it'll be worth
        // trying to add them eventually
        ft.image_error = 0.0;
        ft.object_error = 0.0;
        ft.fiducial_area = 0.0;
        fta.transforms.push_back(ft);
      }
      posePub.publish(fta);
    }

    if (publishDebugFrame && poses.size() > 0 & posePub.getNumSubscribers() > 0)
    {
      auto cvPtr = cv_bridge::toCvCopy(msg, "mono8");
      
      std::vector<ipa_Fiducials::t_points> points;
      if (tagDetector->GetPoints(cvPtr->image, points) & RET_FAILED)
      {
        return;
      }
      // copy the image in CV, draw in some circles, and publish it
      for (const auto &fiducial: points)
      {
        // TODO: maybe add text to distinguish fiducials here
        for (const auto &point: fiducial.image_points)
        {
          auto &image = cvPtr->image;
          cv::circle(image, cv::Point(point.x, point.y), 5, cv::Scalar(127));
        }
      }

      debugPub.publish(cvPtr->toImageMsg());
    }
  }
  catch(cv_bridge::Exception & e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
  catch(cv::Exception & e) {
    ROS_ERROR("cv exception: %s", e.what());
  }
}

}

PLUGINLIB_EXPORT_CLASS(pitag_ros::PitagNodelet, nodelet::Nodelet);
