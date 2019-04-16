#include "header/worldInterface.h"

WorldInterface::WorldInterface(std::string callerName, std::string toolName){

  this->callerName = callerName;
  this-> toolName = toolName;

  std::cout << "[" << callerName <<"][WORLD_INTERFACE] Start"<< std::endl;


}

int WorldInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //wait to transform wTtool to be ready
  std::string topicTool = "/" + toolName;
  tfListener.waitForTransform("world", topicTool, ros::Time(0), ros::Duration(3.0));
  return 0;
}


int WorldInterface::getwTt(Eigen::Matrix4d* wTt_eigen){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTt_tf;

  std::string topic = "/" + toolName;
  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTt_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *wTt_eigen = CONV::transfMatrix_tf2eigen(wTt_tf);

  return 0;
}
