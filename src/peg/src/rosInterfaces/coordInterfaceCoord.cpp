#include "header/coordInterfaceCoord.h"

CoordInterfaceCoord::CoordInterfaceCoord(ros::NodeHandle nh, std::string robotNameA,
                                         std::string robotNameB){

  this->robotNameA = robotNameA;
  this->robotNameB = robotNameB;

  std::cout << "[COORDINATOR][COORD_INTERFACE] Start" << std::endl;

  readyRobA = false;
  readyRobB = false;
  std::string topicReadyA = "/uwsim/" + robotNameA+"_MissionManager"  + "/ready";
  readyRobASub = nh.subscribe(topicReadyA, 1, &CoordInterfaceCoord::readyRobASubCallback, this);
  std::string topicReadyB = "/uwsim/" + robotNameB+"_MissionManager"  + "/ready";
  readyRobBSub = nh.subscribe(topicReadyB, 1, &CoordInterfaceCoord::readyRobBSubCallback, this);


  std::string topicCoordFromMMA = "/uwsim/" + robotNameA +"_MissionManager/toCoord";
  subCoordFromMMA = nh.subscribe(topicCoordFromMMA, 1, &CoordInterfaceCoord::subCoordFromMMACallBack, this);
  std::string topicCoordFromMMB = "/uwsim/" + robotNameB +"_MissionManager/toCoord";
  subCoordFromMMB = nh.subscribe(topicCoordFromMMB, 1, &CoordInterfaceCoord::subCoordFromMMBCallBack, this);

  readxDotA = false;
  readxDotB = false;
  readJJsharpA = false;
  readJJsharpB = false;

  std::string topicStart = "/uwsim/Coord/StartStopBoth";
  startBothPub = nh.advertise<std_msgs::Bool>(topicStart, 1);

  std::string topicCoopVel = "/uwsim/Coord/CoopVel";
  coopVelPub = nh.advertise<geometry_msgs::TwistStamped>(topicCoopVel, 1);


}

bool CoordInterfaceCoord::getReadyBothRob(){
  return (CoordInterfaceCoord::getReadyRobA()
          && CoordInterfaceCoord::getReadyRobB());
}

bool CoordInterfaceCoord::getReadyRobA(){
  return this->readyRobA;
}
bool CoordInterfaceCoord::getReadyRobB(){
  return this->readyRobB;
}

/**
 * @brief CoordInterfaceCoord::getMatricesFromRobs
 *   get all matrices needed to algorithm
 *   The function call each get if this get isnt previously called with success (ie,
 *   arrived message)
 * @param *nonCoopCartVelA OUT the eigen vector in which we want to store the message
 *    passed by reference so we dont waste memory, already used for temp vars
 * @param *nonCoopCartVelB OUT
 * @param *admisVelToolA OUT the eigen matrix in which we want to store the message
 *   passed by reference so we dont waste memory, already used for temp vars
 * @param admisVelToolB OUT
 * @return 0 if everything arrived,
 *         i>0 if i matrices didn't arrived
 *         -1 Error about dimension of one or more the messages arrived
 */
int CoordInterfaceCoord::getMatricesFromRobs(
      Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVelA,
      Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVelB,
      Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelToolA,
      Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelToolB){

  int return1 = 0;
  int return2 = 0;
  int return3 = 0;
  int return4 = 0;

  if (!readxDotA){
    return1 = CoordInterfaceCoord::getNonCoopCartVel(nonCoopCartVelA, robotNameA);
  }
  if (!readxDotB){
    return2 = CoordInterfaceCoord::getNonCoopCartVel(nonCoopCartVelB, robotNameB);
  }
  if (!readJJsharpA){
    return3 = CoordInterfaceCoord::getJJsharp(admisVelToolA, robotNameA);
  }
  if (!readJJsharpB){
    return4 = CoordInterfaceCoord::getJJsharp(admisVelToolB, robotNameB);
  }

  if (return1 == -1 || return2 ==-1 || return3 ==-1 || return4 ==-1){
    return -1;
  }
  if (return1 == -2 || return2 ==-2 || return3 ==-2 || return4 ==-2){
    return -2;
  }

  int counter = return1+return2+return3+return4;

  //if so, all get returned 0 (message arrived) or someone arent called because called
  //previously and have the flag read to true.
  if (counter == 0){ //reset read flags
    readxDotA = false;
    readxDotB = false;
    readJJsharpA = false;
    readJJsharpB = false;
  }

  return counter;

}

void CoordInterfaceCoord::publishStartBoth(bool start){

  std_msgs::Bool start_msg;
  start_msg.data = start;
  startBothPub.publish(start_msg);

}

void CoordInterfaceCoord::publishCoopVel(Eigen::Matrix<double, VEHICLE_DOF, 1> coopVel){

  geometry_msgs::TwistStamped msg;
  msg.twist.linear.x = coopVel(0);
  msg.twist.linear.y = coopVel(1);
  msg.twist.linear.z = coopVel(2);
  msg.twist.angular.x = coopVel(3);
  msg.twist.angular.y = coopVel(4);
  msg.twist.angular.z = coopVel(5);

  coopVelPub.publish(msg);

}


/**
 * @brief CoordInterfaceCoord::getxDot
 * @param *nonCoopCartVel_eigen the eigen vector in which we want to store the message
 * passed by reference so we dont waste memory, already used for temp vars
 * @param robotName the robot from which we want the matrix (A or B)
 * @return 0 correct executions,
 *         1 Warning mess do not arrived,
 *         -1 Error about dimension of message arrived
 */
int CoordInterfaceCoord::getNonCoopCartVel(
    Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen,
    std::string robotName){

  int vectSize;
  std::vector<double> *tempXdot;
  bool* read;
  if (robotName.compare(robotNameA)){
    vectSize = tempXdotA.size();
    tempXdot = &tempXdotA;
    read = &readxDotA;
  } else if (robotName.compare(robotNameB)){
    vectSize = tempXdotB.size();
    tempXdot = &tempXdotB;
    read = &readxDotB;
  } else {
    return -2;
  }

  if (vectSize == 0){
//    std::cout << "[COORDINATOR][COORD_INTERFACE] WARNING:"
//              << "No xDot_tool to read now "
//              << std::endl;
    return 1;
  }

  if (vectSize != VEHICLE_DOF){
    std::cout << "[COORDINATOR][COORD_INTERFACE] ERROR:"
              << "something wrong with dimension of xDot_tool received from "
              << robotName << std::endl;
    return -1;
  }

  for (int i=0; i<vectSize; i++){
    (*nonCoopCartVel_eigen)(i,0) = tempXdot->at(i);
  }

  tempXdot->clear();
  *read = true;

  return 0;
}

/**
 * @brief CoordInterfaceCoord::getJJsharp
 * @param *admisVelTool_eigen the eigen matrix in which we want to store the message
 * passed by reference so we dont waste memory, already used for temp vars
 * @param robotName the robot from which we want the matrix (A or B)
 * @return 0 correct executions,
 *         1 Warning mess do not arrived,
 *         -1 Error about dimension of message arrived
 */
int CoordInterfaceCoord::getJJsharp(
    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelTool_eigen,
    std::string robotName){


  int nSize;
  std::vector<double> *tempJJsharp;
  bool* read;
  if (robotName.compare(robotNameA)){
    nSize = tempJJsharpA.size();
    tempJJsharp = &tempJJsharpA;
    read = &readJJsharpA;
  } else if (robotName.compare(robotNameB)){
    nSize = tempJJsharpB.size();
    tempJJsharp = &tempJJsharpB;
    read = &readJJsharpB;
  } else {
    return -2;
  }


  if (nSize == 0){
//    std::cout << "[COORDINATOR][COORD_INTERFACE] WARNING:"
//              << "No JJ# to read now "
//              << std::endl;
    return 1;
  }

  if (nSize != VEHICLE_DOF*VEHICLE_DOF){
//    std::cout << "[COORDINATOR][COORD_INTERFACE] ERROR:"
//              << "something wrong with dimension of JJ# received from "
//              << robotName << std::endl;
    return -1;
  }

  //Rembering that eigen is column major and the msg is store row major
  int posRow = 0;
  for (int i=0; i<VEHICLE_DOF; i++){
    for (int j=0; j<VEHICLE_DOF; j++){
      posRow = i*VEHICLE_DOF;
      (*admisVelTool_eigen)(i,j) = tempJJsharp->at(posRow + j);
    }
  }

  tempJJsharp->clear();
  *read = true;
  return 0;
}


void CoordInterfaceCoord::readyRobASubCallback(const std_msgs::Bool::ConstPtr& start){
  readyRobA = start->data;
}
void CoordInterfaceCoord::readyRobBSubCallback(const std_msgs::Bool::ConstPtr& start){
  readyRobB = start->data;
}


/**
 * @brief CoordInterfaceCoord::subCoordFromMMACallBack
 * the callback for subscriber to A robot.
 * @param msg the message received
 * @note in the callback we simply copy the data of the msg. This is faster than
 * copy directly in the two eigen matrices; callbacks must be fast as possible.
 * The copy is performed in the gets functions
 */
void CoordInterfaceCoord::subCoordFromMMACallBack(const peg_msgs::toCoord::ConstPtr& msg){

  tempXdotA.resize(6); //message is a twist so 6 elements

  tempXdotA.at(0) = msg->xDot.twist.linear.x;
  tempXdotA.at(1) = msg->xDot.twist.linear.y;
  tempXdotA.at(2) = msg->xDot.twist.linear.z;
  tempXdotA.at(3) = msg->xDot.twist.angular.x;
  tempXdotA.at(4) = msg->xDot.twist.angular.y;
  tempXdotA.at(5) = msg->xDot.twist.angular.z;


  //dim[0].stride is the total number of element (row*col)
  tempJJsharpA.resize(msg->JJsharp.layout.dim[0].stride);
  tempJJsharpA = msg->JJsharp.data;

}

/**
 * @brief CoordInterfaceCoord::subCoordFromMMBCallBack
 * the callback for subscriber to B robot.
 * @param msg the message received
 * @note in the callback we simply copy the data of the msg. This is faster than
 * copy directly in the two eigen matrices; callbacks must be fast as possible.
 * The copy is performed in the gets functions
 */
void CoordInterfaceCoord::subCoordFromMMBCallBack(const peg_msgs::toCoord::ConstPtr& msg){

  tempXdotB.resize(6); //message is a twist so 6 elements

  tempXdotB.at(0) = msg->xDot.twist.linear.x;
  tempXdotB.at(1) = msg->xDot.twist.linear.y;
  tempXdotB.at(2) = msg->xDot.twist.linear.z;
  tempXdotB.at(3) = msg->xDot.twist.angular.x;
  tempXdotB.at(4) = msg->xDot.twist.angular.y;
  tempXdotB.at(5) = msg->xDot.twist.angular.z;


  //dim[0].stride is the total number of element (row*col)
  tempJJsharpB.resize(msg->JJsharp.layout.dim[0].stride);
  tempJJsharpB = msg->JJsharp.data;

}
