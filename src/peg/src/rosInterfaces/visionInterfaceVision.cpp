#include "header/visionInterfaceVision.h"

VisionInterfaceVision::VisionInterfaceVision(ros::NodeHandle nh, std::string robotName)
{
  this->robotName = robotName;
  this->topicRoot = "/uwsim/Vision/";
  std::cout << "[" << robotName <<"][VISION_INTERFACE] Start"<<std::endl;

  pubHoleTransform =
      nh.advertise<geometry_msgs::TransformStamped>((topicRoot + "holePose"), 1);


}


int VisionInterfaceVision::publishHoleTransform(Eigen::Matrix4d wThole){

  geometry_msgs::TransformStamped wThole_ros;
  wThole_ros.child_frame_id = "hole";
  wThole_ros.transform = CONV::transfMatrix_eigen2geomMsgs(wThole);

  pubHoleTransform.publish(wThole_ros);

  return 0;

}
