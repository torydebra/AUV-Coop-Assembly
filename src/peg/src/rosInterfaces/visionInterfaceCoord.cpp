#include "header/visionInterfaceCoord.h"

VisionInterfaceCoord::VisionInterfaceCoord(
        ros::NodeHandle nh, std::string nodeName)
{
    this->nodeName = nodeName;
    this->topicRoot = "/uwsim/Vision/";
    std::cout << "[" << nodeName <<"][VISION_INTERFACE] Start"<<std::endl;

    std::string topicHole = topicRoot + "holePose";
    subHoleTransform = nh.subscribe(
          topicHole, 1, &VisionInterfaceCoord::subHoleTransformCallback, this);
    arrived = false;

}


void VisionInterfaceCoord::subHoleTransformCallback(
        const geometry_msgs::TransformStamped::ConstPtr& msg){

    this->wThole_priv = msg->transform;
    arrived = true;


}

int VisionInterfaceCoord::getHoleTransform(Eigen::Matrix4d *wThole){

  if (arrived){
    *wThole = CONV::transfMatrix_geomMsgs2Eigen(wThole_priv);
    arrived = false;
    return 0;
  } else {
    return 1;
  }

}

