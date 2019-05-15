#include "header/visionInterfaceMissMan.h"

VisionInterfaceMissMan::VisionInterfaceMissMan(
        ros::NodeHandle nh, std::string robotName)
{
    this->robotName = robotName;
    this->topicRoot = "/uwsim/Vision/";
    std::cout << "[" << robotName <<"][VISION_INTERFACE] Start"<<std::endl;

    std::string topicHole = topicRoot + "holePose";
    subHoleTransform = nh.subscribe(
          topicHole, 1, &VisionInterfaceMissMan::subHoleTransformCallback, this);

}


void VisionInterfaceMissMan::subHoleTransformCallback(
        const geometry_msgs::TransformStamped::ConstPtr& msg){

    this->wThole_priv = msg->transform;


}

int VisionInterfaceMissMan::getHoleTransform(Eigen::Matrix4d *wThole){

  *wThole = CONV::transfMatrix_geomMsgs2Eigen(wThole_priv);

}

