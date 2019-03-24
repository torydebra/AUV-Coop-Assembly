#include "header/rosInterface.h"

/** @brief RosInterface Constructor

    @param robotName the name of the robot (found in .xml file of the scene)
    @param topicTwist the name of the topic where twist command must be published
    @param arcg, argv the standard argument of c++ main, they are needed for ros::init
*/
RosInterface::RosInterface(std::string robotName, std::string topicTwist, int argc, char **argv)
{

  ROS_INFO("[ROS_INTERFACE] Start");
  ros::init(argc, argv, "rosInterface");
  ros::NodeHandle nh;

  this->robotName = robotName;
  this->topicTwist = topicTwist;
  pubTwist = nh.advertise<geometry_msgs::TwistStamped>(this->topicTwist,1);

}

int RosInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //Wait to transform wTv to be ready (or fail if wait more than 3 sec)
  std::string topic = "/" + robotName;
  tfListener_wTv.waitForTransform("world", topic, ros::Time(0), ros::Duration(3.0));

  return 0;
}

int RosInterface::getwTv(Eigen::Matrix4d* wTv_eigen){

  if(!ros::ok()){
    return -1;
  }


  std::string topic = "/" + robotName;
  try {
    tfListener_wTv.lookupTransform("world", topic, ros::Time(0), wTv_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *wTv_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);

  return 0;
}

int RosInterface::sendQDot(std::vector<double> qDot){

  if(!ros::ok()){
    return -1;
  }
  geometry_msgs::TwistStamped twist;
  twist.twist.angular.x=qDot.at(4);
  twist.twist.angular.y=qDot.at(5);
  twist.twist.angular.z=qDot.at(6);
  twist.twist.linear.x=qDot.at(7);
  twist.twist.linear.y=qDot.at(8);
  twist.twist.linear.z=qDot.at(9);

  pubTwist.publish(twist);
  ros::spinOnce();
  return 0;

}
