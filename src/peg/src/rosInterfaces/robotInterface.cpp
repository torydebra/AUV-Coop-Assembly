#include "header/robotInterface.h"


/**
 * @brief robotInterface::robotInterface Constructor
 * @param argc the standard argument of c++ main needed for ros::init
 * @param argv the standard argument of c++ main needed for ros::init and for know robot names:
 * argv[1]=robotName, argv[2] = otherRobotName
 * @param toolName name of the peg in the xml file of the scene
 */
RobotInterface::RobotInterface(ros::NodeHandle nh, std::string robotName, std::string otherRobotName, std::string toolName)
{

  this->robotName = robotName;
  this->otherRobotName = otherRobotName;
  this->topicRoot = "/uwsim/" + robotName + "/";
  this->toolName = toolName;

  std::cout << "[" << robotName <<"][ROBOT_INTERFACE] Start"<<std::endl;

  pubTwist = nh.advertise<geometry_msgs::TwistStamped>((topicRoot + "twist_command"),1);
  pubJoint = nh.advertise<sensor_msgs::JointState>((topicRoot + "joint_command"),1);

  subJointState = nh.subscribe(topicRoot+"joint_state", 1, &RobotInterface::subJointStateCallback, this);
}


int RobotInterface::init(){

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
    std::cout << "[" << robotName <<"][ROBOT_INTERFACE] Waiting for Jstate..."
              <<std::endl;
    ros::spinOnce();
    rate.sleep();
  }


  std::cout << "[" << robotName <<"][ROBOT_INTERFACE] Init done" << std::endl;

  return 0;
}

int RobotInterface::getwTv(Eigen::Matrix4d* wTv_eigen){

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

int RobotInterface::getwTt(Eigen::Matrix4d* wTt_eigen){

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
int RobotInterface::getOtherRobPos(Eigen::Vector3d* pos){

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


void RobotInterface::subJointStateCallback(const sensor_msgs::JointState& js)
{
   jState_priv = js.position;
}

/**
 * @brief robotInterface::getJointState
 * @param jState
 * @return
 * @note Doing so, only when main call this function it gets the joint state position
 */
int RobotInterface::getJointState(std::vector<double> *jState){

  if(!ros::ok()){
    return -1;
  }

  *jState = jState_priv;
  return 0;
}


int RobotInterface::sendyDot(std::vector<double> yDot){

  if(!ros::ok()){
    return -1;
  }
  sensor_msgs::JointState js;
  js.name.push_back(std::string("Slew"));
  js.velocity.push_back(yDot.at(0));
  js.name.push_back(std::string("Shoulder"));
  js.velocity.push_back(yDot.at(1));
  js.name.push_back(std::string("Elbow"));
  js.velocity.push_back(yDot.at(2));
  js.name.push_back(std::string("JawRotate"));
  js.velocity.push_back(yDot.at(3));
  geometry_msgs::TwistStamped twist;
  twist.twist.linear.x=yDot.at(4);
  twist.twist.linear.y=yDot.at(5);
  twist.twist.linear.z=yDot.at(6);
  twist.twist.angular.x=yDot.at(7);
  twist.twist.angular.y=yDot.at(8);
  twist.twist.angular.z=yDot.at(9);

  pubJoint.publish(js);
  pubTwist.publish(twist);
  return 0;

}

//void robotInterface::spinOnce(){
//  ros::spinOnce();
//}
