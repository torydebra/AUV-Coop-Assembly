#include "header/missionManagerPeg.h"
#include <chrono>


/**
 * @brief missionManagerPeg
 * @param argc number of input (minimum 3)
 * @param argv:
 * argv[1] robotName
 * argv[2] otherRobotName
 * argv[3] (optional) path for logging files (folders will be created if not exist)
 * @return
 * @note keep eye on execution time of control loop to check if it cant
 * do computations in the interval of frequency given
 */

int main(int argc, char **argv)
{
  if (argc < 3){
    std::cout << "[MISSION_MANAGER] Please insert the two robots name"<< std::endl;
    return -1;
  }
  std::string robotName = argv[1];
  std::string otherRobotName = argv[2];
  std::string pathLog;
  if (LOG && (argc > 3)){   //if flag log setted to 1 and path log is given
    pathLog = argv[3];
  }


  /// ROS NODE
  ros::init(argc, argv, robotName + "_MissionManagerPeg");
  std::cout << "[" << robotName << "][MISSION_MANAGERPEG] Start" << std::endl;
  ros::NodeHandle nh;


/** **************************************************************************************************************
                               SETTING GOALS
*****************************************************************************************************************/


  /// GOAL VEHICLE    TO grasp peg    OLD values
  double goalLinearVect[] = {-0.287, -0.090, 8};
  Eigen::Matrix4d wTgoalVeh_eigen = Eigen::Matrix4d::Identity();
  //rot part
  wTgoalVeh_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
  //trasl part
  wTgoalVeh_eigen(0, 3) = goalLinearVect[0];
  wTgoalVeh_eigen(1, 3) = goalLinearVect[1];
  wTgoalVeh_eigen(2, 3) = goalLinearVect[2];

  /// GOAL EE  to grasp PEG  OLD values
  double goalLinearVectEE[3];
  if (robotName.compare("g500_A") == 0){
    goalLinearVectEE[0] = 2.89;
    goalLinearVectEE[1] = -0.090;
    goalLinearVectEE[2] = 9.594;

  } else {
    goalLinearVectEE[0] = -1.11;
    goalLinearVectEE[1] = -0.090;
    goalLinearVectEE[2] = 9.594;
  }
  Eigen::Matrix4d wTgoalEE_eigen = Eigen::Matrix4d::Identity();
  //rot part
  //wTgoalEE_eigen.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();
  //the ee must rotate of 90 on z axis degree to catch the peg
  wTgoalEE_eigen.topLeftCorner(3,3) << 0, -1,  0,
                                       1,  0,  0,
                                       0,  0,  1;
  //trasl part
  wTgoalEE_eigen(0, 3) = goalLinearVectEE[0];
  wTgoalEE_eigen(1, 3) = goalLinearVectEE[1];
  wTgoalEE_eigen(2, 3) = goalLinearVectEE[2];

  /// GOAL TOOL
  //double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
  double goalLinearVectTool[] = {0.9999999999999999, -9.999999999999998, 8.378840300977453};
  std::vector<double> eulRad = {0, 0, -1.443185307179587}; //true
  //std::vector<double> eulRad = {0.0, 0.0, -1.303185307179587}; //with error on z: 0.1rad ~ 6deg
  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();

   //rot part
  //wTgoalTool_eigen.topLeftCorner<3,3>() <<  0,  1,  0,
   //                                        -1,  0,  0,
   //                                         0,  0,  1;
  wTgoalTool_eigen.topLeftCorner<3,3>() = FRM::eul2Rot(eulRad);
  //to get inside the hole of 0.2m:
  Eigen::Vector3d v_inside;
  //v_inside << 0.40, 0.18, 0; //rigth hole + error
  v_inside << 0.20, 0, 0;
  Eigen::Vector3d w_inside = FRM::eul2Rot(eulRad) * v_inside;

  //trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0] + w_inside(0);
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1] + w_inside(1);
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2] + w_inside(2);

  std::cout << "GOALLLLL DEGUBE\n" << wTgoalTool_eigen << "\n\n\n";


/** ***************************************************************************************************************
                               INITIALIZE THINGS
*******************************************************************************************************************/

  /// struct container data to pass among functions
  Infos robInfo;
  //struct transforms to pass them to Controller class
  robInfo.transforms.wTgoalVeh_eigen = wTgoalVeh_eigen;
  robInfo.transforms.wTgoalEE_eigen = wTgoalEE_eigen;
  robInfo.transforms.wTgoalTool_eigen = wTgoalTool_eigen;
  robInfo.robotState.vehicleVel.resize(VEHICLE_DOF);
  std::fill(robInfo.robotState.vehicleVel.begin(), robInfo.robotState.vehicleVel.end(), 0);


  ///Ros interfaces
  RobotInterface robotInterface(nh, robotName);
  robotInterface.init();

  WorldInterface worldInterface(robotName);
  worldInterface.waitReady("world", otherRobotName);

  /// Set initial state (todo, same as in control loop, make a function?)
  robotInterface.getJointState(&(robInfo.robotState.jState));
  robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
  worldInterface.getwPos(&(robInfo.exchangedInfo.otherRobPos), otherRobotName);

  /// KDL parser to after( in the control loop )get jacobian from joint position
  std::string filenamePeg; //the model of robot with a "fake joint" in the peg
  //so, it is ONLY for the already grasped scene
  if (robotName.compare("g500_A") == 0){
    filenamePeg = "/home/tori/UWsim/Peg/model/g500ARM5APeg2.urdf";
  } else {
    filenamePeg = "/home/tori/UWsim/Peg/model/g500ARM5BPeg.urdf";
  }
  std::string vehicle = "base_link";
  std::string link0 = "part0";
  std::string endEffector = "end_effector";
  std::string peg = "peg"; //pass this to kdlHelper costructor to have jacobian of the center of peg
  std::string pegHead = "pegHead"; //pass this to kdlHelper constructor to have jacobian of the tip

  KDLHelper kdlHelper(filenamePeg, link0, endEffector, vehicle, pegHead);
  // KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));
  //kdlHelper.getFixedFrame(endEffector, peg, &(robInfo.robotStruct.eeTtool));
  kdlHelper.getFixedFrame(endEffector, pegHead, &(robInfo.robotStruct.eeTtoolTip));

  //get info from sensors
  robotInterface.getForceTorque(&(robInfo.robotSensor.forcePegTip), &robInfo.robotSensor.torquePegTip);

  kdlHelper.setEESolvers();
  //get ee pose RESPECT LINK 0
  kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
  // useful have vTee: even if it is redunant, everyone use it a lot
  robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
  // get jacobian respect link0
  kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
  //get whole jacobian (arm+vehcile and projected on world
  computeWholeJacobianEE(&robInfo);

  kdlHelper.setToolSolvers();
  kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.v_Jtool_man));
  computeWholeJacobianTool(&robInfo);


  //std::cout << "DEBUG N  wTv:\n" << robInfo.robotState.wTv_eigen << "\n";
  //std::cout << "DEBUG N  vTee:\n" << robInfo.robotState.vTee_eigen << "\n";

        //TODO TRY : for B is not really fixed...
  robInfo.transforms.wTt_eigen = robInfo.robotState.wTv_eigen *
      robInfo.robotState.vTee_eigen * robInfo.robotStruct.eeTtoolTip;

  CoordInterfaceMissMan coordInterface(nh, robotName);



  ///Controller
  Controller controller(robotName);

  /// Task lists
  std::vector<Task*> tasksTPIK1; //the first, non coop, one
  std::vector<Task*> tasksCoop;
  std::vector<Task*> tasksArmVehCoord; //for arm-vehicle coord
  setTaskLists(robotName, &tasksTPIK1, &tasksCoop, &tasksArmVehCoord);


/** ***************************************************************************************************************
                               LOGGING
********************************************************************************************************************/
  /// Log folders TO DO AFTER EVERY SETTASKLIST?
  /// //TODO ulteriore subfolder per la mission

  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger(robotName, pathLog);

    //to for each list... if task was in a previous list, no problem, folder with same name... overwrite?
    logger.createTasksListDirectories(tasksTPIK1);
    logger.createTasksListDirectories(tasksCoop);

  }

/** *****************************************************************************************************+**********
                               WAITING COORDINATION
*****************************************************************************************************************/


/** *********************************************************************************************************+*********
                               MAIN CONTROL LOOP
*******************************************************************************************************************/
  int ms = MS_CONTROL_LOOP;
  boost::asio::io_service io;
  boost::asio::io_service io2;

  //debug
  double nLoop = 0.0;

  while(1){

    //auto start = std::chrono::steady_clock::now();

    // this must be inside loop
    boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));

    /// Update state
    robotInterface.getJointState(&(robInfo.robotState.jState));
    robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
    //worldInterface.getwT(&(robInfo.transforms.wTt_eigen), toolName);

    /// info from sensors
    robotInterface.getForceTorque(&(robInfo.robotSensor.forcePegTip), &robInfo.robotSensor.torquePegTip);
    robInfo.robotSensor.forcePegTip = FRM::saturateVectorEigen(robInfo.robotSensor.forcePegTip, 5);
    robInfo.robotSensor.torquePegTip = FRM::saturateVectorEigen(robInfo.robotSensor.torquePegTip, 5);

    //to try what hapens with only torque
    //robInfo.robotSensor.forcePegTip << 0, 0, 0;
    std::cout << "force torque arrive:\n"
              << robInfo.robotSensor.forcePegTip << "\n"
              << robInfo.robotSensor.torquePegTip << "\n\n";
    worldInterface.getwPos(&(robInfo.exchangedInfo.otherRobPos), otherRobotName);
    //visionInterface.getHoleTransform(&(robInfo.transforms.wTholeEstimated_eigen));


    //get ee pose RESPECT LINK 0
    kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
    // useful have vTee: even if it is redunant, tasks use it a lot
    robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
    robInfo.transforms.wTt_eigen = robInfo.robotState.wTv_eigen *
        robInfo.robotState.vTee_eigen * robInfo.robotStruct.eeTtoolTip;
    // get jacobian
    kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
    computeWholeJacobianEE(&robInfo);

    //kdlHelper.setToolSolvers(eeTtool);
    kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.v_Jtool_man));
    computeWholeJacobianTool(&robInfo);

    /** *********************************************************************************/

    /// Pass state to controller which deal with tpik
    controller.updateMultipleTasksMatrices(tasksTPIK1, &robInfo);
    std::vector<double> yDotTPIK1 = controller.execAlgorithm(tasksTPIK1);

    std::vector<double> yDotFinal(TOT_DOF);

    controller.updateMultipleTasksMatrices(tasksArmVehCoord, &robInfo);
    std::vector<double> yDotOnlyArm = controller.execAlgorithm(tasksArmVehCoord);

    for (int i=0; i<ARM_DOF; i++){
      yDotFinal.at(i) = yDotOnlyArm.at(i);
    }

    for (int i = ARM_DOF; i<TOT_DOF; i++) {
      yDotFinal.at(i) = yDotTPIK1.at(i);
      //at the moment no error so velocity are exaclty what we are giving
      robInfo.robotState.vehicleVel.at(i-ARM_DOF) = yDotFinal.at(i);
    }

    /** **********   add collision disturbances  **************************++*****************/
    std::vector<double> deltayDot(ARM_DOF);
    std::vector<double> deltayDotVeh(VEHICLE_DOF);
    std::vector<double> deltayDotTot(TOT_DOF);
    std::vector<double> yDotFinalWithCollision = yDotFinal;
    if (COLLISION_PROPAGATOR){
      if (robInfo.robotSensor.forcePegTip.norm() > 0 || robInfo.robotSensor.torquePegTip.norm() > 0){
       //if active, calculate and modify joint command accordingly
        //only forces
//              deltayDot = CollisionPropagator::calculateCollisionDisturb(
//                    robInfo.robotState.w_Jtool_robot.leftCols<4>(), robInfo.transforms.wTt_eigen,
//                    robInfo.robotSensor.forcePegTip);

        //forces and torques
        deltayDot = CollisionPropagator::calculateCollisionDisturb(
              robInfo.robotState.w_Jtool_robot.leftCols<4>(), robInfo.transforms.wTt_eigen,
              robInfo.robotSensor.forcePegTip, robInfo.robotSensor.torquePegTip);

        for (int i=0; i<ARM_DOF; i++){
          yDotFinalWithCollision.at(i) += (0.001*deltayDot.at(i)); //add disturbs to command
          //this is for logging
          deltayDotTot.at(i)= 0.001*deltayDot.at(i);
          //yDotFinalWithCollision.at(i) = deltayDot.at(i); //reset command and only move arm for disturb
          // but if I reset the force task become useless
        }

        deltayDotVeh = CollisionPropagator::calculateCollisionDisturbVeh(
              robInfo.robotState.w_Jtool_robot.rightCols<6>(), robInfo.transforms.wTt_eigen,
              robInfo.robotSensor.forcePegTip, robInfo.robotSensor.torquePegTip);

        for (int i=ARM_DOF; i<TOT_DOF; i++){
          yDotFinalWithCollision.at(i) += (0.01 * deltayDotVeh.at(i-ARM_DOF));
          deltayDotTot.at(i)= 0.01 * deltayDotVeh.at(i-ARM_DOF);
        }

        //debug try: stop vehilce until collision resolved. zeroing vehicle vel both with and withoud collision prop
        //withou collision prop is the task force which resolve the collision NOT GOOD MI SA

      }
    }

    /** **********   Send command to vehicle  *************************************************/
    //robotInterface.sendyDot(yDotTPIK1);
    //robotInterface.sendyDot(yDotOnlyVeh);
    robotInterface.sendyDot(yDotFinal);
    //yDotFinalWithCollision = FRM::saturateVectorStd(yDotFinalWithCollision, 0.01);
    //robotInterface.sendyDot(yDotFinalWithCollision);


    /** **********   Log things  ************************************************************/
    if (pathLog.size() != 0){
      logger.logAllForTasks(tasksArmVehCoord);
      //TODO flag LOGGED to not print same thing twice
      //or maybe another list with ALL task only to log here and create folder for log
      //for the moment, yDot are exactly how vehicle and arm are moving
      logger.logNumbers(yDotTPIK1, "yDotTPIK1");
      logger.logNumbers(yDotFinal, "yDotFinal");
      logger.logNumbers(yDotFinalWithCollision, "yDotFinalWithCollision");
      logger.logNumbers(robInfo.robotSensor.forcePegTip, "forces");
      logger.logNumbers(robInfo.robotSensor.torquePegTip, "torques");
      logger.logNumbers(robInfo.robotState.w_Jtool_robot *
                        CONV::vector_std2Eigen(deltayDotTot), "toolVel4Collision");
      //logger.logNumbers(admisVelTool_eigen, "JJsharp");

    }

    ///PRINT
    /// DEBUG

//    std::cout << "JJsharp\n" << admisVelTool_eigen << "\n\n";
//    std::cout << "SENDED non coop vel:\n" << nonCoopCartVel_eigen <<"\n\n\n";
//    std::cout << "ARRIVED coop vel:\n" << robInfo.exchangedInfo.coopCartVel << "\n\n\n";


//    std::cout << "J: \n"
//              << robInfo.robotState.w_Jtool_robot
//              << "\n\n";
//    std::cout << "yDot: \n"
//              << CONV::vector_std2Eigen(yDotOnlyVeh)
//              << "\n\n";
//    std::cout << "The tool velocity non coop are (J*yDot): \n"
//              << nonCoopCartVel_eigen
//              << "\n\n";
//    std::cout << "The tool velocity coop are (J*yDot): \n"
//              << robInfo.robotState.w_Jtool_robot
//                 * CONV::vector_std2Eigen(yDotFinal)
//              << "\n\n";

    std::vector<Task*> tasksDebug = tasksArmVehCoord;
    for(int i=0; i<tasksDebug.size(); i++){
      if (tasksDebug[i]->getName().compare("FORCE_INSERTION") == 0){
        std::cout << "Activation " << tasksDebug[i]->getName() << ": \n";
        tasksDebug[i]->getActivation().PrintMtx() ;
        std::cout << "\n";
        std::cout << "JACOBIAN " << tasksDebug[i]->getName() << ": \n";
        tasksDebug[i]->getJacobian().PrintMtx();
        std::cout<< "\n";

        std::cout << "REFERENCE " << tasksDebug[i]->getName() << ": \n";
        tasksDebug[i]->getReference().PrintMtx() ;
        std::cout << "\n";
      }
    }


    controller.resetAllUpdatedFlags(tasksTPIK1);
    controller.resetAllUpdatedFlags(tasksArmVehCoord);
    ros::spinOnce();


    //auto end = std::chrono::steady_clock::now();
    //auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    //std::cout << "TEMPOOOOO  "<<duration << "\n\n";


    loopRater.wait();
    //nLoop += 1;

//    if (nLoop > 10){
//      exit(-1);
//    }

  } //end controol loop




  coordInterface.pubIamReadyOrNot(false);

  //todo lista con all task per cancellarli
  deleteTasks(&tasksArmVehCoord);

  return 0;
}


/**
 * @brief setTaskListInit initialize the task list passed as first argument
 * @param robotName name of the robot
 * @param *tasks std::vector<Task*> the vector of pointer to the task, passed by reference
 * @note   note: std::vector is nicer because to change the order of priority or to leave for the moment
 * a task we can simply comment the row.
 * instead, with tasks as Task**, we need to fill the list with task[0], ...task[i] ... and changing
 * priority order would be slower.
 */
void setTaskLists(std::string robotName, std::vector<Task*> *tasks1,
                  std::vector<Task*> *tasksCoord, std::vector<Task*> *tasksArmVeh){

  bool eqType = true;
  bool ineqType = false;


  /// CONSTRAINT TASKS
  Task* coopTask6dof = new CoopTask(6, eqType, robotName);
  Task* coopTask5dof = new CoopTask(5, eqType, robotName);
  Task* constrainVel = new VehicleConstrainVelTask(6, eqType, robotName);


  /// SAFETY TASKS
  Task* jl = new JointLimitTask(4, ineqType, robotName);
  Task* ha = new HorizontalAttitudeTask(1, ineqType, robotName);
  Task* eeAvoid = new ObstacleAvoidEETask(1, ineqType, robotName);


  /// PREREQUISITE TASKS
  Task* forceInsert = new ForceInsertTask(2, ineqType, robotName);
  Task* vehStill = new VehicleNullVelTask(6, eqType, robotName);


  ///MISSION TASKS
  Task* pr5 = new PipeReachTask(5, eqType, robotName, ONLYVEH, LINANGBOTH);
  Task* pr6 = new PipeReachTask(6, eqType, robotName, BOTH, LINANGBOTH);

  Task* eer = new EndEffectorReachTask(6, eqType, robotName);
  Task* vehR = new VehicleReachTask(3, eqType, robotName, ANGULAR);
  Task* vehYaxis = new VehicleReachTask(1, eqType, robotName, YAXIS);

  /// OPTIMIZATION TASKS
  Task* shape = new ArmShapeTask(4, ineqType, robotName, PEG_GRASPED_PHASE);
  //The "fake task" with all eye and zero matrices, needed as last one for algo?
  Task* last = new LastTask(TOT_DOF, eqType, robotName);

  ///Fill tasks list
  // note: order of priority at the moment is here
  //tasks1->push_back(vehStill);
  tasks1->push_back(jl);
  tasks1->push_back(ha);
  //tasks1->push_back(eeAvoid);
  tasks1->push_back(forceInsert);
  //tasks1->push_back(pr6);
  //TODO discutere diff tra pr6 e pr5... il 5 mette più stress sull obj?
  //tasks1->push_back(vehR);
  //tasks1->push_back(vehYaxis);
  //tasks1->push_back(eer);
  //tasks1->push_back(shape);
  tasks1->push_back(last);

  //TODO ASK al momento fatte così le versioni 6dof di pr e coop sono
  //molto meglio
  //coop 5 dof assolutamente no
  //pr5 da un po di stress cmq (linearmente -0.034, -0.008, -0.020),
  //i due tubi divisi si vedono tanto agli estremi
  //si dovrebbe abbassare gain e saturaz per usare il pr5
  //soprattutto se c'è sto shape fatto cosi che fa muovere tantissimo
  //i bracci, è da cambiare lo shape desiderato
//  tasksCoord->push_back(coopTask6dof);
//  tasksCoord->push_back(jl);
//  tasksCoord->push_back(ha);
//  //tasksCoord->push_back(pr6);
//  //tasks1->push_back(eeAvoid);
//  tasksCoord->push_back(shape);
//  tasksCoord->push_back(last);

  //tasksArmVeh->push_back(coopTask6dof);
  tasksArmVeh->push_back(constrainVel);
  //tasksArmVeh->push_back(vehStill);
  tasksArmVeh->push_back(jl);
  tasksArmVeh->push_back(ha);
  tasksArmVeh->push_back(forceInsert);
  //tasksArmVeh->push_back(pr6);
  //tasksArmVeh->push_back(shape);
  tasksArmVeh->push_back(last);

}


/** @brief
 * @note It is important to delete singularly all object pointed by the vector tasks. simply tasks.clear()
 * deletes the pointer but not the object Task pointed
*/
void deleteTasks(std::vector<Task*> *tasks){
  for (std::vector< Task* >::iterator it = tasks->begin() ; it != tasks->end(); ++it)
    {
      delete (*it);
    }
    tasks->clear();
}













/********* OLD VERSIONS *********************************************************************************

///**
//void setTaskLists(std::string robotName, std::vector<Task*> *tasks){

//  /// PUT HERE NEW TASKS.
//  // note: order of priority at the moment is here
//  bool eqType = true;
//  bool ineqType = false;

//  //tasks->push_back(new VehicleNullVelTask(6, ineqType));

//  tasks->push_back(new JointLimitTask(4, ineqType, robotName));
//  tasks->push_back(new HorizontalAttitudeTask(1, ineqType, robotName));

//  //tasks->push_back(new ObstacleAvoidEETask(1, ineqType, robotName));
//  //tasks->push_back(new ObstacleAvoidVehicleTask(1, ineqType, robotName));

//  //tasks->push_back(new FovEEToToolTask(1, ineqType, robotName));

//  //tasks->push_back(new EndEffectorReachTask(6, eqType, robotName));
//  tasks->push_back(new PipeReachTask(5, eqType, robotName));
//  //tasks->push_back(new VehicleReachTask(6, eqType, robotName));

//  tasks->push_back(new ArmShapeTask(4, ineqType, robotName, MID_LIMITS));
//  //The "fake task" with all eye and zero matrices, needed as last one for algo
//  tasks->push_back(new LastTask(TOT_DOF, eqType, robotName));
//}

//void setTaskLists(std::string robotName, std::vector<Task*> *tasks1, std::vector<Task*> *tasksFinal){

//  bool eqType = true;
//  bool ineqType = false;


//  /// CONSTRAINT TASKS
//  Task* coopTask6dof = new CoopTask(6, eqType, robotName);
//  Task* coopTask5dof = new CoopTask(5, eqType, robotName);


//  /// SAFETY TASKS
//  Task* jl = new JointLimitTask(4, ineqType, robotName);
//  Task* ha = new HorizontalAttitudeTask(1, ineqType, robotName);


//  /// PREREQUISITE TASKS


//  ///MISSION TASKS
//  Task* pr5 = new PipeReachTask(5, eqType, robotName);
//  Task* pr6 = new PipeReachTask(6, eqType, robotName);

//  Task* tr = new EndEffectorReachTask(6, eqType, robotName);

//  /// OPTIMIZATION TASKS
//  Task* shape = new ArmShapeTask(4, ineqType, robotName, MID_LIMITS);
//  //The "fake task" with all eye and zero matrices, needed as last one for algo?
//  Task* last = new LastTask(TOT_DOF, eqType, robotName);

//  ///Fill tasks list, MID_LIMITS
//  // note: order of priority at the moment is here
//  tasks1->push_back(jl);
//  tasks1->push_back(ha);
//  tasks1->push_back(pr5);
//  //tasks1->push_back(tr);
//  //tasks1->push_back(shape);
//  tasks1->push_back(last);

//  tasksFinal->push_back(coopTask5dof);
// // tasksFinal->push_back(jl);
//  //tasksFinal->push_back(ha);
//  //tasksFinal->push_back(shape);
//  tasksFinal->push_back(last);
//}






/// DEBUG
//    std::cout << "JACOBIAN " << i << ": \n";
//    tasks[i]->getJacobian().PrintMtx();
//    std::cout<< "\n";
//    std::cout << "REFERENCE " << i << ": \n";
//    tasks[i]->getReference().PrintMtx() ;
//    std::cout << "\n";


//std::cout << nonCoopCartVel_eigen << "\n\n";
//std::cout << admisVelTool_eigen << "\n\n";


//Q.PrintMtx("Q"); ///DEBUG
//yDot_cmat.PrintMtx("yDot");



//DEBUGGGG
//computeJacobianToolNoKdl(&robInfo, robotName);

//std::cout << "con KDL:\n" << robInfo.robotState.w_Jtool_robot << "\n\n";
//std::cout << "senza KDL:\n" << robInfo.robotState.w_JNoKdltool_robot << "\n\n";

/*
//debug
Eigen::Matrix4d toolPose_eigen;
kdlHelper.getToolpose(robInfo.robotState.jState, eeTtool, &toolPose_eigen);
Eigen::Matrix4d W_toolPose_eigen = robInfo.robotState.wTv_eigen * robInfo.robotStruct.vTlink0 * toolPose_eigen;

std::cout << "ToolPOSE\n" <<
             W_toolPose_eigen << "\n\n";
*/
