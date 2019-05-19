#include "header/coordInterfaceMissMan.h"

CoordInterfaceMissMan::CoordInterfaceMissMan(ros::NodeHandle nh, std::string robotName){

  this->robotName = robotName;
  this->start = false;
  std::string topicRoot = "/uwsim/" + robotName + "_MissionManager/";

  std::cout << "[" << robotName <<"][COORD_INTERFACE] Start"<<std::endl;

  pubToCoord = nh.advertise<peg_msgs::toCoord>((topicRoot + "toCoord"), 1);

  //pub to let coord that is ready (e.g. to see if both missManager A & B started)
  // ASK towrite in doc  _missionManager" topic are for mission manager topic...
  //withotu missman (only robname) are topic of robotInterface
  readyPub = nh.advertise<std_msgs::Bool>(topicRoot + "ready", 1);

  //sub to know if other robot is ready
  //even if we can read directly the topic of the other robot,
  // for consistency ALL infos received from extern (ie subscribed topic)
  // are given by coordinator.
  // for make each miss manager comunicate with its own robot, there is
  // the robot interface that is another thing
  std::string topicStart = "/uwsim/Coord/StartStopBoth";
  startSub = nh.subscribe(topicStart, 1, &CoordInterfaceMissMan::startSubCallback, this);

  //sub to receive coop vel from coordinator
  std::string topicCoopVel = "/uwsim/Coord/CoopVel";
  coopVelSub = nh.subscribe(topicCoopVel, 1, &CoordInterfaceMissMan::subMMFromCoordCallBack, this);


  /// initialize JJsharp to send to coordinator (dimensions, the actual size of data after this will remain 0)
  /// this dimension are fixed so it is useless to create a new message each time we have to publish
  //two dimension (it is a matrix)
  toCoordMsg.JJsharp.layout.dim.push_back(std_msgs::MultiArrayDimension());
  toCoordMsg.JJsharp.layout.dim.push_back(std_msgs::MultiArrayDimension());
  toCoordMsg.JJsharp.layout.dim[0].label = "row";
  toCoordMsg.JJsharp.layout.dim[1].label = "column";
  toCoordMsg.JJsharp.layout.dim[0].size = VEHICLE_DOF;
  toCoordMsg.JJsharp.layout.dim[1].size = VEHICLE_DOF;
  toCoordMsg.JJsharp.layout.dim[0].stride = VEHICLE_DOF*VEHICLE_DOF;
  toCoordMsg.JJsharp.layout.dim[1].stride = VEHICLE_DOF;
  toCoordMsg.JJsharp.layout.data_offset = 0;


}

void CoordInterfaceMissMan::pubIamReadyOrNot(bool ready){
  std_msgs::Bool ready_msg;
  ready_msg.data = ready;
  readyPub.publish(ready_msg);
}

void CoordInterfaceMissMan::startSubCallback(const std_msgs::Bool::ConstPtr& msg){
  start = msg->data;
}

bool CoordInterfaceMissMan::getStartFromCoord(){
  return this->start;
}

/**
 * @brief CoordInterface::publishToCoord publish the necessary thing to the coordinator node
 * @param nonCoopCartVel_eigen (barDotX = w_Jtool_robot * yDot) it is the non-cooperative, tool-frame Cartesian
 * velocities that would be resulted if the yDot computed (at the first TPIK, normal one
 * without cooperation things) were applied.
 * @param admisVelTool_eigen (w_Jtool_robot * w_Jtool_robot^#) computation to express
 * the space of achievable object velocities at the current,
 * overall-system constrained, configuration
 * @return 0 correct exec
 */
int CoordInterfaceMissMan::publishToCoord(Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen,
                                          Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen){


  toCoordMsg.xDot.twist.linear.x = nonCoopCartVel_eigen(0);
  toCoordMsg.xDot.twist.linear.y = nonCoopCartVel_eigen(1);
  toCoordMsg.xDot.twist.linear.z = nonCoopCartVel_eigen(2);
  toCoordMsg.xDot.twist.angular.x = nonCoopCartVel_eigen(3);
  toCoordMsg.xDot.twist.angular.y = nonCoopCartVel_eigen(4);
  toCoordMsg.xDot.twist.angular.z = nonCoopCartVel_eigen(5);


  toCoordMsg.JJsharp.data.clear();
  //eigen is column major, multiarray is row major, so we cant simply msg.data = eigen.data
  //and having clear above, we cant directly acces with toCoordMsg.JJsharp.data.at(posRow + j)
  //ASK in this way is it safer? or we can directly overwrite value of previous message (ie without clear?) it would be also faster?
  //faster because every time we dont have to delete memory with clear and reassign with the toCoordMsg.JJsharp.data = temp;
  std::vector<double> temp(VEHICLE_DOF*VEHICLE_DOF, 0);
  int posRow = 0;
  for (int i=0; i<VEHICLE_DOF; i++){
    for (int j=0; j<VEHICLE_DOF; j++){
      posRow = i*VEHICLE_DOF;
      temp[posRow + j] = admisVelTool_eigen(i,j);
    }
  }
  toCoordMsg.JJsharp.data = temp;


  pubToCoord.publish(toCoordMsg);

  return 0;
}


int CoordInterfaceMissMan::getCoopCartVel(
    Eigen::Matrix<double, VEHICLE_DOF, 1> *coopCartVel_eigen){

  int vectSize = tempCoopVel.size();

  if (vectSize == 0){
//    std::cout << "[" << robotName << "][COORD_INTERFACE] WARNING:"
//              << "No xDot_coopVel to read now "
//              << std::endl;
    return 1;
  }

  if (vectSize != VEHICLE_DOF){
    std::cout << "[" << robotName << "][COORD_INTERFACE] ERROR:"
              << "something wrong with dimension of xDot_coopVel received from "
              << "Coordinator" << std::endl;
    return -1;
  }

  for (int i=0; i<vectSize; i++){
    (*coopCartVel_eigen)(i,0) = tempCoopVel.at(i);
  }

  tempCoopVel.clear();

  return 0;
}


/**
 * @brief CoordInterfaceMissMan::subMMFromCoordCallBack store info of message in a std vector,
 * in the get function this temp std vector will be used to fill the eigen vector
 * @param msg the message arrived
 */
void CoordInterfaceMissMan::subMMFromCoordCallBack(const geometry_msgs::TwistStamped& msg){

  tempCoopVel.resize(6); //message is a twist so 6 elements

  tempCoopVel.at(0) = msg.twist.linear.x;
  tempCoopVel.at(1) = msg.twist.linear.y;
  tempCoopVel.at(2) = msg.twist.linear.z;
  tempCoopVel.at(3) = msg.twist.angular.x;
  tempCoopVel.at(4) = msg.twist.angular.y;
  tempCoopVel.at(5) = msg.twist.angular.z;

}
