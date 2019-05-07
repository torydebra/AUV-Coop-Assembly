#include "header/coordinator.h"


int main(int argc, char **argv){

  if (argc < 3){
    std::cout << "[COORDINATOR] Please insert the two robots name"
              << std::endl;
    return -1;
  }
  std::string robotName1 = argv[1];
  std::string robotName2 = argv[2];
  std::string pathLog;
  if (LOG && (argc > 3)){   //if flag log setted to 1 and path log is given
    pathLog = argv[3];
  }

  bool readyRob1 = false;
  bool readyRob2 = false;

  ros::init (argc, argv, "Coordinator");
  std::cout << "[COORDINATOR] Start" << std::endl;
  ros::NodeHandle nh;

  /// GOAL TOOL
  //double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
  double goalLinearVectTool[] = {-0.27, -1.102, 9.000};

  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();

   //rot part
  wTgoalTool_eigen.topLeftCorner<3,3>() << 0, -1,  0,
                                           1,  0,  0,
                                           0,  0,  1;

  //wTgoalTool_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  //trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0];
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1];
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2];


  //TODO put these in interfaces ? this should be a single publisher...
  std::string topicStart = "/uwsim/Coord/StartStopBoth";
  ros::Publisher startBothPub = nh.advertise<std_msgs::Bool>(topicStart, 1);

  std::string topicCoopVel = "/uwsim/Coord/CoopVel";
  ros::Publisher coopVelPub = nh.advertise<geometry_msgs::TwistStamped>(topicCoopVel, 1);



  std_msgs::Bool start;
  start.data = false;


  /// Interfaces
  // world interface needed to find peg position to calculate barX_t (reference that brings tool in goal)
  std::string toolName = "pipe";
  std::string toolName2 = "pipe2";
  WorldInterface worldInterface("COORDINATOR");
  worldInterface.waitReady(toolName);
  worldInterface.waitReady(toolName2);


  CoordInterfaceCoord coordInterfaceA(nh, robotName1);
  CoordInterfaceCoord coordInterfaceB(nh, robotName2);
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

    start.data = false;
    while(!(readyRob1 && readyRob2)){
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      startBothPub.publish(start);

      readyRob1 = coordInterfaceA.getReadyRob();
      readyRob2 = coordInterfaceB.getReadyRob();
      ros::spinOnce();
      loopRater.wait();
    }

    std::cout << "[COORDINATOR] Now both robot are ready" << std::endl;


    start.data = true;
    while(readyRob1 && readyRob2){
      boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));
      startBothPub.publish(start); //TODO publish once? but first mess always lost...

      bool arrived[4] = {false, false, false, false};
      //if at least one did not arrive, repeat loop, but call each get only once
      while (arrived[0] == false || arrived[1] == false || arrived[2] == false || arrived[3] == false){
        if (arrived[0] == false){
          arrived[0] = (coordInterfaceA.getNonCoopCartVel(&nonCoopCartVelA_eigen)  == 0) ? true : false;
        }
        if (arrived[1] == false){
          arrived[1] = (coordInterfaceA.getJJsharp(&admisVelToolA_eigen) == 0) ? true : false;
        }
        if (arrived[2] == false){
          arrived[2] = (coordInterfaceB.getNonCoopCartVel(&nonCoopCartVelB_eigen)  == 0) ? true : false;
        }
        if (arrived[3] == false){
          arrived[3] = (coordInterfaceB.getJJsharp(&admisVelToolB_eigen) == 0) ? true : false;
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


      publishCoopVel(coopVelPub, coopVelToolFeasible);


      ///LOGGING
      if (pathLog.size() != 0){
        Eigen::Matrix4d wTt2;
        worldInterface.getwT(&wTt2, toolName2);

        logger.writeStressTool(wTt, wTt2);

      }


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
    logger->writeScalar(muA, "weightA");
    logger->writeScalar(muB, "weightB");
    logger->writeEigenMatrix(coopVelTool, "notFeasibleCoopVel");
    logger->writeNonCoopVel(nonCoopCartVelA_eigen, "g500_A");
    logger->writeNonCoopVel(nonCoopCartVelB_eigen, "g500_B");
    logger->writeCoopVel(coopVelToolFeasible);
    logger->writeEigenMatrix(refTool, "idealTool");
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

void publishCoopVel(ros::Publisher coopVelPub, Eigen::Matrix<double, VEHICLE_DOF, 1> coopVel){

  geometry_msgs::TwistStamped msg;
  msg.twist.linear.x = coopVel(0);
  msg.twist.linear.y = coopVel(1);
  msg.twist.linear.z = coopVel(2);
  msg.twist.angular.x = coopVel(3);
  msg.twist.angular.y = coopVel(4);
  msg.twist.angular.z = coopVel(5);

  coopVelPub.publish(msg);

}
