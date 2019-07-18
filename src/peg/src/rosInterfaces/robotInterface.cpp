#include "header/robotInterface.h"


/**
 * @brief robotInterface::robotInterface Constructor
 * @param nh the nodehandle to deal with ros sub and pub
 * @param name of the robot itself (for sub to topics and print purposes)
 */
RobotInterface::RobotInterface(ros::NodeHandle nh, std::string robotName)
{

  this->robotName = robotName;
  this->topicRoot = "/uwsim/" + robotName + "/";

  std::cout << "[" << robotName <<"][ROBOT_INTERFACE] Start"<<std::endl;

  pubTwist = nh.advertise<geometry_msgs::TwistStamped>((topicRoot + "twist_command"),1);
  pubJoint = nh.advertise<sensor_msgs::JointState>((topicRoot + "joint_command"),1);

  subJointState = nh.subscribe(topicRoot+"joint_state", 1, &RobotInterface::subJointStateCallback, this);

  //force_priv.resize(3, 0.0); //0 for default value
  //torque_priv.resize(3, 0.0);
  vectorForceQueue.resize(3);
  vectorTorqueQueue.resize(3);
  for (int i =0; i<3; i++){ //3 becuase x y z directions
    vectorForceQueue.at(i) = boost::circular_buffer<double>(NELEMENTQUEUEFOR);
    vectorTorqueQueue.at(i) = boost::circular_buffer<double>(NELEMENTQUEUETOR);
  }

  //subForceTorque = nh.subscribe(topicRoot+"forceSensorPeg", 1, &RobotInterface::subForceTorqueCallback, this);

  //hard-coded name for topic, beacuse both robot read from a single force sensor
  subForceTorque = nh.subscribe("/uwsim/g500_A/forceSensorPeg", 1, &RobotInterface::subForceTorqueCallback, this);
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

//  std::cout << "rosinterface :\n "
//            << wTv_tf.getBasis().getRow(0).getX() << " " << wTv_tf.getBasis().getRow(0).getY() << " " << wTv_tf.getBasis().getRow(0).getZ() << "\n"
//            << wTv_tf.getBasis().getRow(1).getX() << " " << wTv_tf.getBasis().getRow(1).getY() << " " << wTv_tf.getBasis().getRow(1).getZ() << "\n"
//            << wTv_tf.getBasis().getRow(2).getX() <<" " << wTv_tf.getBasis().getRow(2).getY() << " " << wTv_tf.getBasis().getRow(2).getZ() << "\n"
//            << "\n\n";

//  std::cout << "rosinterface :\n "
 //           << wTv_tf.getRotation().getX() << " " << wTv_tf.getRotation().getY() << " " << wTv_tf.getRotation().getZ() << " " << wTv_tf.getRotation().getW() << "\n";

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

/**
 * @brief RobotInterface::subForceTorqueCallback
 * @param msg
 */
void RobotInterface::subForceTorqueCallback(const geometry_msgs::WrenchStamped& msg){

  //std::vector<double> force_priv(3); //with queues they dont need to be member of class
  //force_priv.at(0) = msg.wrench.force.x;
  //force_priv.at(1) = msg.wrench.force.y;
  //force_priv.at(2) = msg.wrench.force.z;
  vectorForceQueue.at(0).push_back(msg.wrench.force.x);
  vectorForceQueue.at(1).push_back(msg.wrench.force.y);
  vectorForceQueue.at(2).push_back(msg.wrench.force.z);


  //std::vector<double> torque_priv(3);
  //torque_priv.at(0) = msg.wrench.torque.x;
  //torque_priv.at(1) = msg.wrench.torque.y;
  //torque_priv.at(2) = msg.wrench.torque.z;
  vectorTorqueQueue.at(0).push_back(msg.wrench.torque.x);
  vectorTorqueQueue.at(1).push_back(msg.wrench.torque.y);
  vectorTorqueQueue.at(2).push_back(msg.wrench.torque.z);
}


int RobotInterface::getForceTorque(Eigen::Vector3d *force, Eigen::Vector3d *torque){

  if (vectorForceQueue.at(0).size() == 0){ // no sensor data yet
    *force << 0, 0, 0;
    *torque << 0, 0, 0;
    return 1;
  }

  std::vector<double> sumForces (3);
  std::vector<double> sumTorques (3);

  for (int i=0; i<3; i++){
    sumForces.at(i) = std::accumulate(vectorForceQueue.at(i).begin(), vectorForceQueue.at(i).end(), 0.0); //0 init the sum to 0
    sumTorques.at(i) = std::accumulate(vectorTorqueQueue.at(i).begin(), vectorTorqueQueue.at(i).end(), 0.0); //0 init the sum to 0

    sumForces.at(i) /= vectorForceQueue.at(i).size();
    sumTorques.at(i) /= vectorTorqueQueue.at(i).size();
  }


  *force = CONV::vector_std2Eigen(sumForces);
  *torque = CONV::vector_std2Eigen(sumTorques);
  // need to saturate otherwise simulation crashes
  //*force = FRM::saturateVectorEigen(*force, 1);
  //*torque = FRM::saturateVectorEigen(*torque, 1);
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



//int RobotInterface::getwTjoints(std::vector<Eigen::Matrix4d> *wTjoints) {
//  if(!ros::ok()){
//    return -1;
//  }

//  std::vector<tf::StampedTransform> wTJoints_tf(ARM_DOF);

//  try {
//    for(int i=0; i<ARM_DOF; i++){
//      std::ostringstream s;
//      s << (i+1);
//      const std::string si(s.str());
//      std::string topic = "/" + robotName +"/part" + si;
//      if (i==3){
//        topic += "_base";
//      }
//      tfListener.lookupTransform("world", topic, ros::Time(0), wTJoints_tf[i]);
//    }


//  } catch (tf::TransformException &ex) {
//    ROS_ERROR("%s",ex.what());
//    ros::Duration(1.0).sleep();
//  }

//  for(int i=0; i<ARM_DOF; i++){
//    wTjoints->push_back(CONV::transfMatrix_tf2eigen(wTJoints_tf[i]));
//  }


//  return 0;
//}

