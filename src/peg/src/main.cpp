#include "header/main.h"



int main(int argc, char **argv)
{

  ROS_INFO("[MAIN] Start");
  ros::init(argc, argv, "main"); //necessary for real ros node rosinterface

  /// GOAL VEHICLE
  double goalLinearVect[] = {-0.287, -0.062, 7.424};
  Eigen::Matrix4d wTgoalVeh_eigen = Eigen::Matrix4d::Identity();
  //rot part
  wTgoalVeh_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
  //trasl part
  wTgoalVeh_eigen(0, 3) = goalLinearVect[0];
  wTgoalVeh_eigen(1, 3) = goalLinearVect[1];
  wTgoalVeh_eigen(2, 3) = goalLinearVect[2];

  /// GOAL EE
  double goalLinearVectEE[] = {-0.27, -0.102, 2.124};
  Eigen::Matrix4d wTgoalEE_eigen = Eigen::Matrix4d::Identity();
  //rot part
  wTgoalEE_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
//  wTgoalEE_eigen.topLeftCorner(3,3) << 0.5147, 0 , -0.8574,
//                                          0    ,1,     0    ,
//                                        0.8573 , 0  , 0.5147;
  //trasl part
  wTgoalEE_eigen(0, 3) = goalLinearVectEE[0];
  wTgoalEE_eigen(1, 3) = goalLinearVectEE[1];
  wTgoalEE_eigen(2, 3) = goalLinearVectEE[2];


  //struct transforms to pass them to Controller class
  struct Transforms transf;
  transf.wTgoalVeh_eigen = wTgoalVeh_eigen;
  transf.wTgoalEE_eigen = wTgoalEE_eigen;
  ///Controller
  Controller controller;

  ///Ros interface
  RosInterface rosInterface("girona500_A", "/uwsim/g500_A/", argc, argv);

  rosInterface.init();
  int ms = 100;
  while(ros::ok()){

    rosInterface.getwTv(&(transf.wTv_eigen));
    rosInterface.getvTee(&(transf.vTee_eigen));

    std::vector <Eigen::Matrix4d> vTjoints;
    rosInterface.getvTjoints(&vTjoints);
    transf.vTjoints = vTjoints;

    rosInterface.getJointState(&(transf.jState));

    controller.updateTransforms(&transf);

    std::vector<double> qDot = controller.execAlgorithm();

    rosInterface.sendQDot(qDot);

    rosInterface.spinOnce(); // actually the spinonce is called here and not in sendQdot

    boost::this_thread::sleep_for(boost::chrono::milliseconds(ms));
  }

  return 0;
}

