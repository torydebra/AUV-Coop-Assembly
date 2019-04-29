#include "header/worldInterface.h"

WorldInterface::WorldInterface(std::string callerName, std::string toolName,
                               std::string toolName2){

  this->callerName = callerName;
  this->toolName = toolName;
  this->toolName2 = toolName2;

  std::cout << "[" << callerName <<"][WORLD_INTERFACE] Start"<< std::endl;

}

int WorldInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //wait to transform wTtool to be ready
  std::string topicTool = "/" + toolName;
  tfListener.waitForTransform("world", topicTool, ros::Time(0), ros::Duration(3.0));

  if (toolName2.size() != 0){
    std::string topicTool2 = "/" + toolName2;
    tfListener.waitForTransform("world", topicTool2, ros::Time(0), ros::Duration(3.0));

  }

  //wait to transform wTv of the other robot to be ready
  //std::string topic3 = "/" + otherRobotName;
  //tfListener.waitForTransform("world", topic3, ros::Time(0), ros::Duration(3.0));

  return 0;
}


int WorldInterface::getwTt(Eigen::Matrix4d* wTt_eigen, bool tool2){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTt_tf;

  std::string topic;
  if (tool2 == true){
    if (toolName2.size() == 0){
      std::cout << "[" << callerName <<"][WORLD_INTERFACE] ERROR"
                << "tool Name 2 not inserted during a previous calling to my costructor"
                << std::endl;
      return -1;
    }
    topic = "/" + toolName2;

  } else {
    topic = "/" + toolName;
  }

  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTt_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *wTt_eigen = CONV::transfMatrix_tf2eigen(wTt_tf);

  return 0;
}

int WorldInterface::getRobPos(Eigen::Vector3d* pos, std::string robotName){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTv_tf;

  std::string topic = "/" + robotName;
  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTv_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Matrix4d wTvother_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);
  *pos = wTvother_eigen.topRightCorner<3,1>();

  return 0;
}
