#include "header/robotInterface.h"


/**
 * @brief robotInterface::robotInterface Constructor
 * @param argc the standard argument of c++ main needed for ros::init
 * @param argv the standard argument of c++ main needed for ros::init and for know robot names:
 * argv[1]=robotName, argv[2] = otherRobotName
 * @param toolName name of the peg in the xml file of the scene
 */
RobotInterface::RobotInterface(ros::NodeHandle nh, std::string robotName, std::string otherRobotName)
{

  this->robotName = robotName;
  this->otherRobotName = otherRobotName;
  this->topicRoot = "/uwsim/" + robotName + "/";

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



int RobotInterface::getwTjoints(std::vector<Eigen::Matrix4d> *wTjoints) {
  if(!ros::ok()){
    return -1;
  }

  std::vector<tf::StampedTransform> wTJoints_tf(ARM_DOF);

  try {
    for(int i=0; i<ARM_DOF; i++){
      std::ostringstream s;
      s << (i+1);
      const std::string si(s.str());
      std::string topic = "/" + robotName +"/part" + si;
      if (i==3){
        topic += "_base";
      }
      tfListener.lookupTransform("world", topic, ros::Time(0), wTJoints_tf[i]);
    }


  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  for(int i=0; i<ARM_DOF; i++){
    wTjoints->push_back(CONV::transfMatrix_tf2eigen(wTJoints_tf[i]));
  }


  return 0;
}
