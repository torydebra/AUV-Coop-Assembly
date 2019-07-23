#include "header/coordinator.h"


int main(int argc, char **argv){

  if (argc < 3){
    std::cout << "[COORDINATOR] Please insert the two robots name"
              << std::endl;
    return -1;
  }
  std::string robotNameA = argv[1];
  std::string robotNameB = argv[2];
  std::string pathLog;
  if (LOG && (argc > 3)){   //if flag log setted to 1 and path log is given
    pathLog = argv[3];
  }

  bool readyBothRob = false;

  ros::init (argc, argv, "Coordinator");
  std::cout << "[COORDINATOR] Start" << std::endl;
  ros::NodeHandle nh;

  // world interface needed to find peg position to calculate barX_t (reference that brings tool in goal)
  std::string toolName = "g500_A/pegHead";
  std::string toolName2 = "g500_B/pegHead";
  WorldInterface worldInterface("COORDINATOR");
  worldInterface.waitReady(toolName);
  worldInterface.waitReady(toolName2);
  Eigen::Matrix4d wTt;
  worldInterface.getwT(&wTt, toolName);



  /// GOAL TOOL
  //double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
  double goalLinearVectToolTrue[] = {0.9999999999999999, -9.999999999999998, 8.378840300977453};
  double goalLinearVectTool[] =     {0.9999999999999999, -9.999999999999998, 8.378840300977453}; //with error

  std::vector<double> eulRadTrue = {0, 0, -1.443185307179587}; //true angular
  std::vector<double> eulRad = {0, 0, -1.443185307179587}; //with error on z: 0.1rad ~ 6deg

  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();

  /// rot part
//  wTgoalTool_eigen.topLeftCorner<3,3>() << 0,  1,  0,
//                                           -1,  0,  0,
//                                           0,  0,  1;
  //wTgoalTool_eigen.topLeftCorner<3,3>() = wTt.topLeftCorner<3,3>(); //actual goal = actual rotation
  wTgoalTool_eigen.topLeftCorner<3,3>() = FRM::eul2Rot(eulRad);

  //to get inside the hole of 0.2m:
  Eigen::Vector3d v_inside;
  //v_inside << 0.40, 0.18, 0; //rigth big hole + error
  v_inside << 0.20, 0, 0;
  Eigen::Vector3d w_inside = FRM::eul2Rot(eulRadTrue) * v_inside;

  /// trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0] + w_inside(0);
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1] + w_inside(1);
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2] + w_inside(2);

  Eigen::Matrix4d wTgoalTool_ideal; //the perfect goal for log purpose
  wTgoalTool_ideal.topLeftCorner<3,3>() = FRM::eul2Rot(eulRadTrue);
  wTgoalTool_ideal(0, 3) = goalLinearVectToolTrue[0] + w_inside(0);
  wTgoalTool_ideal(1, 3) = goalLinearVectToolTrue[1] + w_inside(1);
  wTgoalTool_ideal(2, 3) = goalLinearVectToolTrue[2] + w_inside(2);



  CoordInterfaceCoord coordInterface(nh, robotNameA, robotNameB);
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> refTool;
  Eigen::Matrix<double, VEHICLE_DOF, 1> coopVelToolFeasible;
  Eigen::Vector3d forcePegTip;
  Eigen::Vector3d torquePegTip;
  double changeMagnitude = 0.0005; //gain for change the goals according to force arrived
  //double changeMagnitudeAng = 0.0001;

  ///Log things
  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger("Coordinator", pathLog);
    logger.createDirectoryForNode();
  }

  int ms = MS_CONTROL_LOOP;
  boost::asio::io_service io;

  while(ros::ok()){

    while(!(readyBothRob)){ //wait until both ready, meanwhile publish false to make no robot start
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      coordInterface.publishStartBoth(false);

      readyBothRob = coordInterface.getReadyBothRob();
      ros::spinOnce();
      loopRater.wait();
    }

    std::cout << "[COORDINATOR] Now both robot are ready" << std::endl;

    while(readyBothRob){
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      coordInterface.publishStartBoth(true);

      worldInterface.getwT(&wTt, toolName);

      /// CHANGE GOAL
      if (CHANGE_GOAL){ //if active, calculate and modify joint command accordingly

        coordInterface.getForceTorque(&forcePegTip, &torquePegTip);
        std::cout << "force:\n" << forcePegTip << "\n torques:\n" << torquePegTip << "\n";

        if (forcePegTip.norm() > 0 || torquePegTip.norm() > 0) {
          wTgoalTool_eigen = changeGoalLin(changeMagnitude, forcePegTip, wTgoalTool_eigen);
          //wTgoalTool_eigen = changeGoalAng(changeMagnitudeAng, forcePegTip, wTt.topLeftCorner<3,3>(),  wTgoalTool_eigen);


          coordInterface.publishUpdatedGoal(wTgoalTool_eigen);

          ///DEBUGG
          //std::cout << "Goal modified:\n";
          //std::cout << wTgoalTool_eigen;
          //std::cout << "\n\n";
        }
      }
      /// END CHANGE GOAL

      //if at least one did not arrive, repeat loop, but call each get only once (this is done inside methods class interface)
      int returned = 1;
      while (returned > 0){
        returned = coordInterface.getMatricesFromRobs(&nonCoopCartVelA_eigen, &nonCoopCartVelB_eigen, &admisVelToolA_eigen, &admisVelToolB_eigen);
        if (returned == -1){
          std::cout << "[COORDINATOR] ERROR in the dimension of mesage arrived from robots" << std::endl;
          return -1;
        }
        if (returned == -2){
          std::cout << "[COORDINATOR] ERROR in the name of robots" << std::endl;
          return -1;
        }

        boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(1));
        ros::spinOnce();
        loopRater.wait();
      }


//      std::cout << nonCoopCartVelA_eigen << "\n\n";
//      std::cout << admisVelToolA_eigen << "\n\n";
//      std::cout << nonCoopCartVelB_eigen << "\n\n";
//      std::cout << admisVelToolB_eigen << "\n\n";

      worldInterface.getwT(&wTt, toolName);
      refTool = calculateRefTool(wTgoalTool_eigen, wTt);
      coopVelToolFeasible = execCoordAlgo(nonCoopCartVelA_eigen, admisVelToolA_eigen,
                                          nonCoopCartVelB_eigen, admisVelToolB_eigen,
                                          refTool, &logger);

      std::cout << "Calculated coop vel:\n" << coopVelToolFeasible << "\n\n";

      coordInterface.publishCoopVel(coopVelToolFeasible);


      ///LOGGING

      CMAT::TransfMatrix wTtool_cmat = CONV::matrix_eigen2cmat(wTt);
      CMAT::TransfMatrix wTgoalTool_ideal_cmat = CONV::matrix_eigen2cmat(wTgoalTool_ideal);
      CMAT::Vect6 errorSwapped = CMAT::CartError(wTgoalTool_ideal_cmat, wTtool_cmat);//ang;lin
      // ang and lin must be swapped because in yDot and jacob linear part is before

      Eigen::Matrix<double, VEHICLE_DOF, 1> error;
      error(0) = errorSwapped(4);
      error(1) = errorSwapped(5);
      error(2) = errorSwapped(6);
      error(3) = errorSwapped(1);
      error(4) = errorSwapped(2);
      error(5) = errorSwapped(3);
      if (pathLog.size() != 0){
        Eigen::Matrix4d wTt2;
        worldInterface.getwT(&wTt2, toolName2);
        //logger.logCartError(wTt, wTt2, "stressTool");
        logger.logNumbers(wTgoalTool_eigen, "wTgoal");
        logger.logNumbers(wTt, "wTt");
        logger.logNumbers(error, "realgoal_Tool_error");
      }

      readyBothRob = coordInterface.getReadyBothRob();
      ros::spinOnce();
      loopRater.wait();
    }

    std::cout << "[COORDINATOR] Now one robot is not ready anymore" << std::endl;

  }

}

Eigen::Matrix<double, VEHICLE_DOF, 1> execCoordAlgo(
    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelA_eigen,
    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolA_eigen,
    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelB_eigen,
    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolB_eigen,
    Eigen::Matrix<double, VEHICLE_DOF, 1> refTool,
    Logger* logger){

  double muZero = 1; //this is only to not divide by zero?

  // weights to understand the "difficulty" each robot has in following the ideal reference(refTool)
  double muA = muZero + ((refTool-nonCoopCartVelA_eigen).norm());
  double muB = muZero + ((refTool-nonCoopCartVelB_eigen).norm());

  //xHatDot_tool
  Eigen::Matrix<double, VEHICLE_DOF, 1> coopVelTool;
  coopVelTool = (1 / (muA + muB)) *
      ( (muA * nonCoopCartVelA_eigen) + (muB * nonCoopCartVelB_eigen) );

  // C
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> constraintMat;
  constraintMat = (admisVelToolA_eigen - admisVelToolB_eigen);

  //xTildeDot_tool
  Eigen::Matrix<double, VEHICLE_DOF, 1> coopVelToolFeasible;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> eye =
      Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF>::Identity();

  coopVelToolFeasible = ( eye - (FRM::pseudoInverse(constraintMat) * constraintMat) )
      * coopVelTool;

  if (logger != NULL){
    logger->logNumbers(muA, "weightA");
    logger->logNumbers(muB, "weightB");
    logger->logNumbers(coopVelTool, "notFeasibleCoopVel");
    logger->logNumbers(nonCoopCartVelA_eigen, "nonCoopVelg500_A");
    logger->logNumbers(nonCoopCartVelB_eigen, "nonCoopVelg500_B");
    logger->logNumbers(coopVelToolFeasible, "coopVel");
    logger->logNumbers(refTool, "idealTool");
  }

  return coopVelToolFeasible;
}


Eigen::Matrix<double, VEHICLE_DOF, 1> calculateRefTool(Eigen::Matrix4d wTgoaltool_eigen,
                                                       Eigen::Matrix4d wTtool_eigen){

  CMAT::TransfMatrix wTgoaltool_cmat = CONV::matrix_eigen2cmat(wTgoaltool_eigen);
  CMAT::TransfMatrix wTtool_cmat = CONV::matrix_eigen2cmat(wTtool_eigen);

  //double gain = 0.5; GAIN E SATURAZ??
  CMAT::Vect6 errorSwapped = CMAT::CartError(wTgoaltool_cmat, wTtool_cmat);//ang;lin
  // ang and lin must be swapped because in yDot and jacob linear part is before


  Eigen::Matrix<double, VEHICLE_DOF, 1> reference;
  double gainLin = 0.05;
  double gainAng = 0.08;
  reference(0) = gainLin*errorSwapped(4);
  reference(1) = gainLin*errorSwapped(5);
  reference(2) = gainLin*errorSwapped(6);
  reference(3) = gainAng*errorSwapped(1);
  reference(4) = gainAng*errorSwapped(2);
  reference(5) = gainAng*errorSwapped(3);


  reference.topRows(3) = FRM::saturateVectorEigen(reference.topRows(3), 0.1);
  reference.bottomRows(3) = FRM::saturateVectorEigen(reference.bottomRows(3), 0.1);


  return reference;

}

Eigen::Matrix4d changeGoalLin(double gain, Eigen::Vector3d forcePegTip, Eigen::Matrix4d wTgoalTool_eigen){

  if (forcePegTip(1) != 0 || forcePegTip(2) != 0) {
    Eigen::Vector3d tchange;
    Eigen::Vector3d wChange;

    tchange << gain*forcePegTip;
    tchange(0) = 0; //nullified modification on x axis (x axis of tool)
    wChange = wTgoalTool_eigen.topLeftCorner<3,3>() * tchange;
    wChange = FRM::saturateVectorEigen(wChange, 0.0007); //orig 0.0005

    wTgoalTool_eigen.topRightCorner<3,1>() += wChange;
  }

  return wTgoalTool_eigen;
}





/// change ANG not work properly
/// version all in world
Eigen::Matrix4d changeGoalAng(double gain, Eigen::Vector3d torquePegTip,
                              Eigen::Matrix3d wRt_eigen, Eigen::Matrix4d wTgoalTool_eigen){

  /// angular part TO ASK IF RIGHT TODO
  if (torquePegTip(1) != 0 || torquePegTip(2) != 0) {

    double dt = 0.1; //frequency of arrived sensor info is 10 hz
    Eigen::Vector3d w_angVel; //angular vel of new goal respect old goal projected on frame old goal
    //torquePegTip(0) = 0; //nullified modification on x axis (x axis of tool)

    //integrating torque (it should be multiplied by dt first (integration)
    // but I "include" dt in the gain. So gain must be less than dt. Multiply also for gain and not
    // only by dt is necessary to reduce the intensity of the torque (as done in previous function for forces)
    w_angVel = gain * (wRt_eigen * torquePegTip); //torque is respect to the tool
    w_angVel = FRM::saturateVectorEigen(w_angVel, 0.0001);

    CMAT::RotMatrix wRg_cmat = CONV::matrix_eigen2cmat(wTgoalTool_eigen.topLeftCorner<3,3>());
    //Strap down: Out = e^[wdt^] * gRnewg_0_cmat
    CMAT::RotMatrix wRnewg = wRg_cmat.StrapDown(CONV::matrix_eigen2cmat(w_angVel), dt);

    ///debug
//    std::cout << "DEBug:\n torquePegTip:\n" << torquePegTip << "\n w_angVel\n:" << w_angVel << "\n wRnewg:\n";
//    wRnewg.PrintMtx();
//    std::cout << "angleAxis: \n";
//    CMAT::Vect3 vec = CONV::matrix_eigen2cmat(w_angVel);
//    vec.AngleAxis().PrintMtx();
//    std::cout << std::endl;

    wTgoalTool_eigen.topLeftCorner<3,3>() = CONV::matrix_cmat2eigen(wRnewg);
  }

  return wTgoalTool_eigen;
}


/// version old with g prime
//Eigen::Matrix4d changeGoalAng(double gain, Eigen::Vector3d torquePegTip,
//                              Eigen::Matrix3d wRt_eigen, Eigen::Matrix4d wTgoalTool_eigen){

//  /// angular part TO ASK IF RIGHT TODO
//  if (torquePegTip(1) != 0 || torquePegTip(2) != 0) {

//    double dt = 0.1; //frequency of arrived sensor info is 10 hz
//    Eigen::Matrix3d gRt = (wTgoalTool_eigen.topLeftCorner<3,3>().transpose()) * (wRt_eigen);
//    Eigen::Vector3d g_angVel; //angular vel of new goal respect old goal projected on frame old goal

//    torquePegTip(0) = 0; //nullified modification on x axis (x axis of tool)

//    //integrating torque (it should be multiplied by dt first (integration)
//    // but I "include" dt in the gain. So gain must be less than dt. Multiply also for gain and not
//    // only by dt is necessary to reduce the intensity of the torque (as done in previous function for forces)
//    g_angVel << gain*(gRt * torquePegTip); //torque is respect to the tool
//    g_angVel = FRM::saturateVectorEigen(g_angVel, 0.005);

//    // the rot between goal and new goal is an identity BEFORE the application of angular vel
//    CMAT::RotMatrix gRnewg_0_cmat = CMAT::Matrix::Eye(3);

//    //Strap down: Out = e^[wdt^] * gRnewg_0_cmat
//    CMAT::RotMatrix gRnewg_1_cmat = gRnewg_0_cmat.StrapDown(CONV::matrix_eigen2cmat(g_angVel), dt);

//    ///debug
//    std::cout << "DEBug:\n torquePegTip:\n" << torquePegTip << "\ng_angVel\n:" << g_angVel << "\ngRnewg_1_cmat:\n";
//    gRnewg_1_cmat.PrintMtx();
//    std::cout << std::endl;

//    wTgoalTool_eigen.topLeftCorner<3,3>() = wTgoalTool_eigen.topLeftCorner<3,3>() *
//                                            CONV::matrix_cmat2eigen(gRnewg_1_cmat);
//  }

//  return wTgoalTool_eigen;
//}
