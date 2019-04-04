#include "header/rosInterface.h"

/** @brief RosInterface Constructor

    @param robotName the name of the robot (found in .xml file of the scene)
    @param topicTwist the name of the topic where twist command must be published
    @param arcg, argv the standard argument of c++ main, they are needed for ros::init
*/
RosInterface::RosInterface(std::string topicRoot, std::string  robotName,
                           std::string otherRobotName, std::string toolName,
                           int argc, char **argv)
{

  std::cout << "[" << robotName <<"][ROS_INTERFACE] Start"<<std::endl;
  ros::init(argc, argv, "rosInterface_" + robotName);
  ros::NodeHandle nh;

  this->robotName = robotName;
  this->otherRobotName = otherRobotName;
  this->topicRoot = topicRoot;
  this->toolName = toolName;

  pubTwist = nh.advertise<geometry_msgs::TwistStamped>((topicRoot + "twist_command_A"),1);
  pubJoint = nh.advertise<sensor_msgs::JointState>((topicRoot + "joint_command_A"),1);

  subJointState = nh.subscribe(topicRoot+"joint_state_A", 1, &RosInterface::subJointStateCallback, this);
}


int RosInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //Wait to transform wTv to be ready (or fail if wait more than 3 sec)
  std::string topic = "/" + robotName;
  tfListener.waitForTransform("world", topic, ros::Time(0), ros::Duration(3.0));

  //wait to transform wTtool to be ready
  std::string topic2 = "/" + toolName;
  tfListener.waitForTransform("world", topic2, ros::Time(0), ros::Duration(1.0));

  //wait to transform wTv of the other robot to be ready
  std::string topic3 = "/" + otherRobotName;
  tfListener.waitForTransform("world", topic3, ros::Time(0), ros::Duration(3.0));

  //Wait to joint state to be ready (ie : the callback is called at least once)
  ros::Rate rate(100);
  while (jState_priv.size()==0){
    ros::spinOnce();
    rate.sleep();
  }


  std::cout << "[" << robotName <<"][ROS_INTERFACE] Init done" << std::endl;

  return 0;
}

int RosInterface::getwTv(Eigen::Matrix4d* wTv_eigen){

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

  *wTv_eigen = CONV::transfMatrix_tf2eigen(wTv_tf);

  return 0;
}

int RosInterface::getwTt(Eigen::Matrix4d* wTt_eigen){

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

//TODO: maybe get position of other with another method
int RosInterface::getOtherRobPos(Eigen::Vector3d* pos){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTvother_tf;

  std::string topic = "/" + otherRobotName;
  try {
    tfListener.lookupTransform("world", topic, ros::Time(0), wTvother_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  Eigen::Matrix4d wTvother_eigen = CONV::transfMatrix_tf2eigen(wTvother_tf);
  *pos = wTvother_eigen.topRightCorner<3,1>();

  return 0;
}


void RosInterface::subJointStateCallback(const sensor_msgs::JointState& js)
{
   jState_priv = js.position;
}

/**
 * @brief RosInterface::getJointState
 * @param jState
 * @return
 * @note Doing so, only when main call this function it gets the joint state position
 */
int RosInterface::getJointState(std::vector<double> *jState){

  if(!ros::ok()){
    return -1;
  }

  *jState = jState_priv;
  return 0;
}


int RosInterface::sendQDot(std::vector<double> qDot){

  if(!ros::ok()){
    return -1;
  }
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.velocity.push_back(qDot.at(0));
  js.name.push_back(std::string("Shoulder"));
  js.velocity.push_back(qDot.at(1));
  js.name.push_back(std::string("Elbow"));
  js.velocity.push_back(qDot.at(2));
  js.name.push_back(std::string("JawRotate"));
  js.velocity.push_back(qDot.at(3));
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x=qDot.at(4);
  twist.twist.linear.y=qDot.at(5);
  twist.twist.linear.z=qDot.at(6);
  twist.twist.angular.x=qDot.at(7);
  twist.twist.angular.y=qDot.at(8);
  twist.twist.angular.z=qDot.at(9);

  pubJoint.publish(js);
  pubTwist.publish(twist);
  return 0;

}

void RosInterface::spinOnce(){
  ros::spinOnce();
}
