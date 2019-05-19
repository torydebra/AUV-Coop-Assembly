#include "header/main_old.h"
#include <chrono>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "main"); //necessary for real ros node robotInterface
  std::cout << "[girona500_A][MISSION_MANAGER] Start" << std::endl;

  //// MISSION MANAGER
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
 //// MISSION MANAGER end

  /// struct container data to pass among functions
  Infos robInfo;

  /// KDL parser to after( in the control loop )get jacobian from joint position
  std::string filename = "/home/tori/UWsim/Peg/model/g500ARM5.urdf";
  KDLHelper kdlHelper(filename);
  std::string vehicle = "base_link";
  std::string link0 = "part0";
  std::string endEffector = "end_effector";
  kdlHelper.setSolvers(link0, endEffector);
  /// KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));


  //struct transforms to pass them to Controller class
  robInfo.transforms.wTgoalVeh_eigen = wTgoalVeh_eigen;
  robInfo.transforms.wTgoalEE_eigen = wTgoalEE_eigen;

  ///Controller
  //std::string pathLog = "logPeg/" + PRT::getCurrentDateFormatted();
  //Controller controller("girona500_A", &pathLog);
  Controller controller("girona500_A");


  ///Ros interface
  RobotInterface robotInterface("/uwsim/g500_A/", "girona500_A", "girona500_B", "pipe", argc, argv);
  robotInterface.init();

  robotInterface.getwTt(&(robInfo.transforms.wTt_eigen));

  int ms = 100;
  boost::asio::io_service io;
  while(1){
//    auto start = std::chrono::steady_clock::now();

    // this must be inside loop
    boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));

    robotInterface.getJointState(&(robInfo.robotState.jState));
    robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
    robotInterface.getOtherRobPos(&(robInfo.exchangedInfo.otherRobPos));
    //robotInterface.getvTee(&(transf.vTee_eigen));

//    std::vector <Eigen::Matrix4d> vTjoints;
//    robotInterface.getvTjoints(&vTjoints);
//    transf.vTjoints = vTjoints;

    //get ee pose RESPECT LINK 0
    kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));

    // useful have vTee: even if it is redunant, tasks use it a lot
    robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;

    // get jacobian respect link0
    kdlHelper.getJacobian(robInfo.robotState.jState, &(robInfo.robotState.link0_J_man));

    computeWholeJacobian(&robInfo);



    controller.updateTransforms(&robInfo);

    std::vector<double> yDot = controller.execAlgorithm();

    robotInterface.sendyDot(yDot);

    robotInterface.spinOnce(); // actually the spinonce is called here and not in sendyDot

    //    auto end = std::chrono::steady_clock::now();
    //    auto diff = end - start;
    //    std::cout << std::chrono::duration<double, std::milli> (diff).count()
    //        << " ms" << std::endl;


    loopRater.wait(); //wait for the remaning time until period setted (ms)
  }

  return 0;
}

