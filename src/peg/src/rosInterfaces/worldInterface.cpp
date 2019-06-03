#include "header/worldInterface.h"

WorldInterface::WorldInterface(std::string callerName){

  this->callerName = callerName;
  std::cout << "[" << callerName <<"][WORLD_INTERFACE] Start"<< std::endl;

}



int WorldInterface::waitReady(std::string objName){

  if(!ros::ok()){
    return -1;
  }

  //wait to transform wTx to be ready
  std::string objTopic = "/" + objName;
  tfListener.waitForTransform("world", objTopic, ros::Time(0), ros::Duration(3.0));


  return 0;
}

int WorldInterface::waitReady(std::string firstFrame, std::string secondFrame){

  if(!ros::ok()){
    return -1;
  }

  //wait to transform xTx to be ready
  std::string firstFrameTopic = "/" + firstFrame;
  std::string secondFrameTopic = "/" + secondFrame;

  std::cout << "[" << callerName <<"][WORLD_INTERFACE]: waiting for trasform from " << firstFrameTopic
            << " to " << secondFrame << "\n";
  tfListener.waitForTransform(firstFrameTopic, secondFrameTopic, ros::Time(0), ros::Duration(60.0));


  return 0;
}

int WorldInterface::getT(Eigen::Matrix4d* xTx_eigen, std::string firstFrame, std::string secondFrame){
  std::string firstFrameTopic = "/" + firstFrame;
  std::string secondFrameTopic = "/" + secondFrame;

  tf::StampedTransform wTt_tf;


  try {
    tfListener.lookupTransform(firstFrameTopic, secondFrameTopic, ros::Time(0), wTt_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *xTx_eigen = CONV::transfMatrix_tf2eigen(wTt_tf);



}

int WorldInterface::getwT(Eigen::Matrix4d* wTt_eigen, std::string objName){

  std::string topic = "/" + objName;
  tf::StampedTransform wTt_tf;

  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTt_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *wTt_eigen = CONV::transfMatrix_tf2eigen(wTt_tf);


}


int WorldInterface::getwPos(Eigen::Vector3d* pos, std::string robotName){

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
