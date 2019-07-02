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

  /// GOAL TOOL
  //double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
  double goalLinearVectTool[] = {-0.27, -1.102, 9.000};

  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();

   //rot part
  wTgoalTool_eigen.topLeftCorner<3,3>() << 0, 1,  0,
                                           -1,  0,  0,
                                           0,  0,  1;

  //wTgoalTool_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  //trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0];
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1];
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2];


  //TODO put these in interfaces ? this should be a single publisher...


  /// Interfaces
  // world interface needed to find peg position to calculate barX_t (reference that brings tool in goal)
  std::string toolName = "pipe";
  std::string toolName2 = "pipe2";
  WorldInterface worldInterface("COORDINATOR");
  worldInterface.waitReady(toolName);
  worldInterface.waitReady(toolName2);


  CoordInterfaceCoord coordInterface(nh, robotNameA, robotNameB);
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolA_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolB_eigen;
  Eigen::Matrix<double, VEHICLE_DOF, 1> refTool;
  Eigen::Matrix<double, VEHICLE_DOF, 1> coopVelToolFeasible;

  Eigen::Matrix4d wTt;

  ///Log things
  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger("Coordinator", pathLog);
    logger.createDirectoryForNode();
  }

  int ms = 100;
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
      if (pathLog.size() != 0){
        Eigen::Matrix4d wTt2;
        worldInterface.getwT(&wTt2, toolName2);
        logger.logCartError(wTt, wTt2, "stressTool");
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

  reference *= 0.05;

  reference.topRows(3) = FRM::saturateVectorEigen(reference.topRows(3), 0.3);
  reference.bottomRows(3) = FRM::saturateVectorEigen(reference.bottomRows(3), 0.1);


  return reference;

}
