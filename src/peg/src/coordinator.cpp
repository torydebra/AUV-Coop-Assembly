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
  //double goalLinearVectTool[] = {0.9999999999999999, -9.999999999999998, 8.378840300977453};
  double goalLinearVectTool[] = {0.9799999999999999, -9.999999999999998, 8.378840300977453}; //with error

  //std::vector<double> eulRad = {0, 0, -1.443185307179587}; //true angular
  std::vector<double> eulRad = {0.0, 0.0, -1.443185307179587}; //with error on z: 0.1rad ~ 6deg

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
  Eigen::Vector3d w_inside = FRM::eul2Rot(eulRad) * v_inside;

  /// trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0] + w_inside(0);
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1] + w_inside(1);
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2] + w_inside(2);

  Eigen::Matrix4d wTgoalTool_ideal = wTgoalTool_eigen; //the perfect goal for log purpose



  CoordInterfaceCoord coordInterface(nh, robotNameA, robotNameB);
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> refTool;
  Eigen::Matrix<double, VEHICLE_DOF, 1> coopVelToolFeasible;
  Eigen::Vector3d forcePegTip;
  Eigen::Vector3d torquePegTip;
//  coordInterface.getForceTorque(&forcePegTip, &torquePegTip);
//  forcePegTip = FRM::saturateVectorEigen(forcePegTip, 5);
//  torquePegTip = FRM::saturateVectorEigen(torquePegTip, 5);
  double changeMagnitude = 0.0005; //gain for change the goals according to force arrived

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
        forcePegTip = FRM::saturateVectorEigen(forcePegTip, 5);
        torquePegTip = FRM::saturateVectorEigen(torquePegTip, 5);

        if (forcePegTip.norm() > 0 || torquePegTip.norm() > 0) {
          wTgoalTool_eigen = changeGoal(changeMagnitude, forcePegTip, wTgoalTool_eigen, wTt);
          coordInterface.publishUpdatedGoal(wTgoalTool_eigen);

          ///DEBUGG
          std::cout << "Goal modified:\n";
          std::cout << wTgoalTool_eigen;
          std::cout << "\n\n";
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

      CMAT::TransfMatrix wTgoaltool_cmat = CONV::matrix_eigen2cmat(wTt);
      CMAT::TransfMatrix wTgoalTool_ideal_cmat = CONV::matrix_eigen2cmat(wTgoalTool_ideal);
      CMAT::Vect6 errorSwapped = CMAT::CartError(wTgoaltool_cmat, wTgoalTool_ideal_cmat);//ang;lin
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
  reference(0) = errorSwapped(4);
  reference(1) = errorSwapped(5);
  reference(2) = errorSwapped(6);
  reference(3) = errorSwapped(1);
  reference(4) = errorSwapped(2);
  reference(5) = errorSwapped(3);

  reference *= 0.01;

  reference.topRows(3) = FRM::saturateVectorEigen(reference.topRows(3), 0.05);
  reference.bottomRows(3) = FRM::saturateVectorEigen(reference.bottomRows(3), 0.05);


  return reference;

}

Eigen::Matrix4d changeGoal(double changeMagnitude, Eigen::Vector3d forcePegTip, Eigen::Matrix4d wTgoalTool_eigen,
                           Eigen::Matrix4d wTt_eigen){

  if (forcePegTip(1) != 0 || forcePegTip(2) != 0) {
    Eigen::Vector3d change;
    Eigen::Vector3d wChange;

    change << changeMagnitude*forcePegTip;
    change(0) = 0; //nullified modification on x axis
    wChange = wTgoalTool_eigen.topLeftCorner<3,3>() * change;
    wChange = FRM::saturateVectorEigen(wChange, 0.001); //orig 0.0005

    wTgoalTool_eigen.topRightCorner<3,1>() += wChange;
  }

  return wTgoalTool_eigen;
}
