#include "header/coordInterfaceCoord.h"

CoordInterfaceCoord::CoordInterfaceCoord(ros::NodeHandle nh, std::string robotName){

  this->robotName = robotName;
  std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] Start"<<std::endl;

  readyRob = false;
  std::string topicReady = "/uwsim/" + robotName+"_MissionManager"  + "/ready";
  readyRobSub = nh.subscribe(topicReady, 1, &CoordInterfaceCoord::readyRobSubCallback, this);


  std::string topicCoordFromMM = "/uwsim/" + robotName +"_MissionManager/toCoord";
  subCoordFromMM = nh.subscribe(topicCoordFromMM, 1, &CoordInterfaceCoord::subCoordFromMMCallBack, this);



}

bool CoordInterfaceCoord::getReadyRob(){
  return this->readyRob;
}

/**
 * @brief CoordInterfaceCoord::getxDot
 * @param *nonCoopCartVel_eigen the eigen vector in which we want to store the message
 * passed by reference so we dont waste memory, already used for temp vars
 * @return 0 correct executions,
 *         1 Warning mess do not arrived,
 *         -1 Error about dimension of message arrived
 */
int CoordInterfaceCoord::getNonCoopCartVel(
    Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen){


  int vectSize = tempXdot.size();

  if (vectSize == 0){
//    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] WARNING:"
//              << "No xDot_tool to read now "
//              << std::endl;
    return 1;
  }

  if (vectSize != VEHICLE_DOF){
    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] ERROR:"
              << "something wrong with dimension of xDot_tool received from "
              << robotName << std::endl;
    return -1;
  }

  for (int i=0; i<vectSize; i++){
    (*nonCoopCartVel_eigen)(i,0) = tempXdot.at(i);
  }

  tempXdot.clear();

  return 0;
}

/**
 * @brief CoordInterfaceCoord::getJJsharp
 * @param *admisVelTool_eigen the eigen matrix in which we want to store the message
 * passed by reference so we dont waste memory, already used for temp vars
 * @return 0 correct executions,
 *         1 Warning mess do not arrived,
 *         -1 Error about dimension of message arrived
 */
int CoordInterfaceCoord::getJJsharp(
    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> *admisVelTool_eigen){


  int nSize = tempJJsharp.size();

  if (nSize == 0){
//    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] WARNING:"
//              << "No JJ# to read now "
//              << std::endl;
    return 1;
  }

  if (nSize != VEHICLE_DOF*VEHICLE_DOF){
    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] ERROR:"
              << "something wrong with dimension of JJ# received from "
              << robotName << std::endl;
    return -1;
  }

  //Rembering that eigen is column major and the msg is store row major
  int posRow = 0;
  for (int i=0; i<VEHICLE_DOF; i++){
    for (int j=0; j<VEHICLE_DOF; j++){
      posRow = i*VEHICLE_DOF;
      (*admisVelTool_eigen)(i,j) = tempJJsharp.at(posRow + j);
    }
  }

  tempJJsharp.clear();
  return 0;
}


void CoordInterfaceCoord::readyRobSubCallback(const std_msgs::Bool::ConstPtr& start){
  readyRob = start->data;
}


/**
 * @brief CoordInterfaceCoord::subCoordFromMM1CallBack
 * the callback for subscriber to one robot.
 * @param msg the message received
 * @note in the callback we simply copy the data of the msg. This is faster than
 * copy directly in the two eigen matrices; callbacks must be fast as possible.
 * The copy is performed in the gets functions
 */
void CoordInterfaceCoord::subCoordFromMMCallBack(const peg_msgs::toCoord::ConstPtr& msg){

  tempXdot.resize(6); //message is a twist so 6 elements

  tempXdot.at(0) = msg->xDot.twist.linear.x;
  tempXdot.at(1) = msg->xDot.twist.linear.y;
  tempXdot.at(2) = msg->xDot.twist.linear.z;
  tempXdot.at(3) = msg->xDot.twist.angular.x;
  tempXdot.at(4) = msg->xDot.twist.angular.y;
  tempXdot.at(5) = msg->xDot.twist.angular.z;


  //dim[0].stride is the total number of element (row*col)
  tempJJsharp.resize(msg->JJsharp.layout.dim[0].stride);
  tempJJsharp = msg->JJsharp.data;

}
