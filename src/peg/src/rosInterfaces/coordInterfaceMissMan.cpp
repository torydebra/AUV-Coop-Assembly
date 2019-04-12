#include "header/coordInterfaceMissMan.h"

CoordInterfaceMissMan::CoordInterfaceMissMan(ros::NodeHandle nh, std::string robotName){

  this->robotName = robotName;
  std::string topicRoot = "/uwsim/" + robotName + "_MissionManager/";

  std::cout << "[" << robotName <<"][COORD_INTERFACE] Start"<<std::endl;

  pubToCoord = nh.advertise<peg::toCoord_msg>((topicRoot + "toCoord"), 1);

  /// initialize messages

  toCoordMsg.xDot.resize(VEHICLE_DOF);
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



/**
 * @brief CoordInterface::publishToCoord publish the necessary thing to the coordinator node
 * @param *robInfo Info struct to which prelevate necessary matrices. note passed by reference
 * to avoid a copy, not to modify it
 * @return  correct exec
 *
 * @note nonCoopCartVel_eigen (barDotX = w_Jtool_robot * yDot) it is the non-cooperative, tool-frame Cartesian
 * velocities that would be resulted if the yDot computed (at the first TPIK, normal one
 * without cooperation things) were applied.
 * @note admisVelTool_eigen (w_Jtool_robot * w_Jtool_robot^#) computation to express
 * the space of achievable object velocities at the current,
 * overall-system constrained, configuration

 */
int CoordInterfaceMissMan::publishToCoord(Infos* robInfo, std::vector<double> yDot){

  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen =
      robInfo->robotState.w_Jtool_robot * CONV::vector_std2Eigen(yDot);


  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen =
      robInfo->robotState.w_Jtool_robot * FRM::pseudoInverse(robInfo->robotState.w_Jtool_robot);


  toCoordMsg.JJsharp.data.clear();

  for (int i =0; i<VEHICLE_DOF; i++){
    toCoordMsg.xDot.at(i).data = nonCoopCartVel_eigen(i);
  }

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
