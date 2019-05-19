#include "header/robotInterface.h"

/** @brief robotInterface Constructor

    @param robotName the name of the robot (found in .xml file of the scene)
    @param topicTwist the name of the topic where twist command must be published
    @param arcg, argv the standard argument of c++ main, they are needed for ros::init
*/
robotInterface::robotInterface(std::string robotName, std::string topicRoot, int argc, char **argv)
{

  ROS_INFO("[ROS_INTERFACE] Start");
  ros::init(argc, argv, "robotInterface");
  ros::NodeHandle nh;

  this->robotName = robotName;
  this->topicRoot = topicRoot;

  pubTwist = nh.advertise<geometry_msgs::TwistStamped>((topicRoot + "twist_command_A"),1);
  pubJoint = nh.advertise<sensor_msgs::JointState>((topicRoot + "joint_command_A"),1);

  subJointState = nh.subscribe(topicRoot+"joint_state_A", 1, &robotInterface::subJointStateCallback, this);
}


int robotInterface::init(){

  if(!ros::ok()){
    return -1;
  }

  //Wait to transform wTv to be ready (or fail if wait more than 3 sec)
  std::string topic = "/" + robotName;
  tfListener_wTv.waitForTransform("world", topic, ros::Time(0), ros::Duration(3.0));

  //Wait to joint state to be ready (ie : the callback is called at least once)
  ros::Rate rate(100);
  while (jState_priv.size()==0){
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

int robotInterface::getwTv(Eigen::Matrix4d* wTv_eigen){

  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform wTv_tf;

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

int robotInterface::getvTee(Eigen::Matrix4d* vTee_eigen){
  if(!ros::ok()){
    return -1;
  }

  tf::StampedTransform vTee_tf;

  std::string topic1 = "/" + robotName;
  std::string topic2 = "/" + robotName + "/part4_base";//ASSUMING this name for ee (as in xml files of scenes)
  try {
    tfListener_wTv.lookupTransform(topic1, topic2, ros::Time(0), vTee_tf);

  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  *vTee_eigen = CONV::transfMatrix_tf2eigen(vTee_tf);

  return 0;

}

int robotInterface::getvTjoints(std::vector<Eigen::Matrix4d> *vTjoints) {
  if(!ros::ok()){
    return -1;
  }

  std::vector<tf::StampedTransform> vTJoints_tf(ARM_DOF);

  std::string topic1 = "/" + robotName;
  try {
    for(int i=0; i<ARM_DOF; i++){
      std::ostringstream s;
      s << (i+1);
      const std::string si(s.str());
      std::string topic2 = topic1+"/part" + si;
      if (i==3){
        topic2 += "_base";
      }
      tfListener_wTv.lookupTransform(topic1, topic2, ros::Time(0), vTJoints_tf[i]);
    }


  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }

  for(int i=0; i<ARM_DOF; i++){
    vTjoints->push_back(CONV::transfMatrix_tf2eigen(vTJoints_tf[i]));
  }


  return 0;
}

void robotInterface::subJointStateCallback(const sensor_msgs::JointState& js)
{
   jState_priv = js.position;
}

/**
 * @brief robotInterface::getJointState
 * @param jState
 * @return
 * @note Doing so, only when main call this function it gets the joint state position
 */
int robotInterface::getJointState(std::vector<double> *jState){

  if(!ros::ok()){
    return -1;
  }

  *jState = jState_priv;
  return 0;
}


int robotInterface::sendyDot(std::vector<double> yDot){

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

void robotInterface::spinOnce(){
  ros::spinOnce();
}
