#include "header/main.h"



int main(int argc, char **argv)
{



  ROS_INFO("[MAIN] Start");
  ros::init(argc, argv, "main"); //necessary for real ros node rosinterface

  /// GOAL VEHICLE
  double goalLinearVect[] = {-0.287, -0.062, 7.424};
  Eigen::Matrix4d wTgoal_eigen = Eigen::Matrix4d::Identity();

  //rot part
  wTgoal_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
  //trasl part
  wTgoal_eigen(0, 3) = goalLinearVect[0];
  wTgoal_eigen(1, 3) = goalLinearVect[1];
  wTgoal_eigen(2, 3) = goalLinearVect[2];

  //struct transforms to pass them to Controller class
  struct Transforms transf;
  transf.wTgoal_eigen = wTgoal_eigen;

  ///Controller
  Controller controller;

  ///Ros interface
  RosInterface rosInterface("girona500_A", "/uwsim/g500_A/twist_command_A", argc, argv);

  rosInterface.init();
  Eigen::Matrix4d wTv_eigen;
  int ms = 10; //100 H
  while(ros::ok()){

    rosInterface.getwTv(&wTv_eigen);
    transf.wTv_eigen = wTv_eigen;

    controller.updateTransforms(&transf);

    std::vector<double> qDot = controller.execAlgorithm();

    rosInterface.sendQDot(qDot);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(ms));
  }

  return 0;
}

