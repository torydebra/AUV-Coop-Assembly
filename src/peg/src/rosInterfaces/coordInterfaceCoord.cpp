#include "header/coordInterfaceCoord.h"

CoordInterfaceCoord::CoordInterfaceCoord(ros::NodeHandle nh, std::string robotName){

  this->robotName = robotName;
  std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] Start"<<std::endl;

  std::string topicCoordFromMM = "/uwsim/" + robotName +"_MissionManager/toCoord";
  subCoordFromMM = nh.subscribe(topicCoordFromMM, 1, &CoordInterfaceCoord::subCoordFromMMCallBack, this);


}

/**
 * @brief CoordInterfaceCoord::getxDot
 * @param *nonCoopCartVel_eigen the eigen vector in which we want to store the message
 * passed by reference so we dont waste memory, already used for temp vars
 * @return 0 correct executions,
 *         1 Warning mess do not arrived,
 *         -1 Error about dimension of message arrived
 */
int CoordInterfaceCoord::getxDot(
    Eigen::Matrix<double, VEHICLE_DOF, 1> *nonCoopCartVel_eigen){


  int vectSize = tempXdot.size();

  if (vectSize == 0){
    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] WARNING:"
              << "I don't receive anything from " << robotName <<  " yet"
              << std::endl;
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
    std::cout << "[COORDINATOR][COORD_INTERFACE_to_" << robotName <<"] WARNING:"
              << "Nothing to read now" << robotName <<  "yet"
              << std::endl;
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


/**
 * @brief CoordInterfaceCoord::subCoordFromMM1CallBack
 * the callback for subscriber to one robot.
 * @param msg the message received
 * @note in the callback we simply copy the data of the msg. This is faster than
 * copy directly in the two eigen matrices; callbacks must be fast as possible.
 * The copy is performed in the gets functions
 */
void CoordInterfaceCoord::subCoordFromMMCallBack(const peg::toCoord_msg::ConstPtr& msg){

  tempXdot.resize(msg->xDot.size());
  // a bit faster if we copy in msgData? callbacks must be fast
  for (int i=0; i<msg->xDot.size(); ++i){
    tempXdot.at(i) = msg->xDot.at(i).data;
  }

  //dim[0].stride is the total number of element (row*col)
  tempJJsharp.resize(msg->JJsharp.layout.dim[0].stride);
  tempJJsharp = msg->JJsharp.data;

}
