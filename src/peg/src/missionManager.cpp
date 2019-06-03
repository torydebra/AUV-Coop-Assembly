#include "header/missionManager.h"
#include <chrono>


/**
 * @brief missionManager
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
  ros::init(argc, argv, robotName + "_MissionManager");
  std::cout << "[" << robotName << "][MISSION_MANAGER] Start" << std::endl;
  ros::NodeHandle nh;


/** **************************************************************************************************************
                               SETTING GOALS
*****************************************************************************************************************/



  /// GOAL VEHICLE    TO grasp peg
  double goalLinearVect[] = {-0.287, -0.090, 8};
  Eigen::Matrix4d wTgoalVeh_eigen = Eigen::Matrix4d::Identity();
  //rot part
  wTgoalVeh_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();
  //trasl part
  wTgoalVeh_eigen(0, 3) = goalLinearVect[0];
  wTgoalVeh_eigen(1, 3) = goalLinearVect[1];
  wTgoalVeh_eigen(2, 3) = goalLinearVect[2];

  /// GOAL EE  to grasp PEG
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
  double goalLinearVectTool[] = {-0.27, -1.102, 9.000};

  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();

   //rot part
  wTgoalTool_eigen.topLeftCorner<3,3>() << 0, 1,  0,
                                           -1,  0,  0,
                                           0,  0,  1;

 // wTgoalTool_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  //trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0];
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1];
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2];


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
  //The robot 2 has attached the tool2, we cant give him the pipe1 because
  //if it is not followed exactly for robB the tool is moving respect EE
  //TODO but try giving robB the pipe1
  std::string toolName;
  if (robotName.compare("g500_A") == 0){
    toolName = "pipe";
  } else{
    toolName = "pipe";
  }
  WorldInterface worldInterface(robotName);
  worldInterface.waitReady(toolName);
  CoordInterfaceMissMan coordInterface(nh, robotName);

  VisionInterfaceMissMan visionInterface(nh, robotName);


  /// KDL parser to after( in the control loop )get jacobian from joint position
  //std::string filename = "/home/tori/UWsim/Peg/model/g500ARM5.urdf";
  std::string filenamePeg; //the model of robot with a "fake joint" in the peg
  //so, it is ONLY for the already grasped scene
  if (robotName.compare("g500_A") == 0){
    filenamePeg = "/home/tori/UWsim/Peg/model/g500ARM5APeg.urdf";
  } else {
    filenamePeg = "/home/tori/UWsim/Peg/model/g500ARM5BPeg.urdf";
  }
  std::string vehicle = "base_link";
  std::string link0 = "part0";
  std::string endEffector = "end_effector";

  KDLHelper kdlHelper(filenamePeg, link0, endEffector, robotName);
  kdlHelper.setEESolvers();
  // KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));
 // TODO vT0 non serve più?

  /// Set initial state (todo, same as in control loop, make a function?)
  robotInterface.getJointState(&(robInfo.robotState.jState));
  robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));

  worldInterface.getwT(&(robInfo.transforms.wTt_eigen), toolName);
  //robInfo.transforms.wTgoalTool_eigen.topLeftCorner<3,3>() = robInfo.transforms.wTt_eigen.topLeftCorner<3,3>();
  worldInterface.getwPos(&(robInfo.exchangedInfo.otherRobPos), otherRobotName);

  //DEBUGG
  //robotInterface.getwTjoints(&(robInfo.robotState.wTjoints));



  //get ee pose RESPECT LINK 0
  kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
  // useful have vTee: even if it is redunant, everyone use it a lot
  robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
  // get jacobian respect link0
  kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
  //get whole jacobian (arm+vehcile and projected on world
  computeWholeJacobianEE(&robInfo);


  //TODO before this the robot must have grasped, here only to see if it is correct
  //if (true){ //TODO if grasped
    Eigen::Matrix4d eeTtool =
        FRM::invertTransf(robInfo.robotState.wTv_eigen*robInfo.robotState.vTee_eigen)
          * robInfo.transforms.wTt_eigen;
    kdlHelper.setToolSolvers(eeTtool);
  //}
  kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.v_Jtool_man));
  computeWholeJacobianTool(&robInfo);

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

  //wait coordinator to start
  double msinit = 1;
  boost::asio::io_service ioinit;
  std::cout << "[" << robotName<< "][MISSION_MANAGER] Wating for "<<
               "Coordinator to say I can start...\n";
  while(!coordInterface.getStartFromCoord()){
    boost::asio::deadline_timer loopRater(ioinit, boost::posix_time::milliseconds(msinit));
    coordInterface.pubIamReadyOrNot(true);

    ros::spinOnce();
    loopRater.wait();
  }

  std::cout << "[" << robotName<< "][MISSION_MANAGER] Start from coordinator "<<
               "received\n";


/** *********************************************************************************************************+*********
                               MAIN CONTROL LOOP
*******************************************************************************************************************/
  int ms = 100;
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
    worldInterface.getwT(&(robInfo.transforms.wTt_eigen), toolName);
    worldInterface.getwPos(&(robInfo.exchangedInfo.otherRobPos), otherRobotName);
    visionInterface.getHoleTransform(&(robInfo.transforms.wTholeEstimated_eigen));

    //DEBUG
    //std::cout <<"ARRIVED wThole" << robInfo.transforms.wTholeEstimated_eigen << "\n\n";
    robInfo.transforms.wTgoalTool_eigen = robInfo.transforms.wTholeEstimated_eigen;

    //get ee pose RESPECT LINK 0
    kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
    // useful have vTee: even if it is redunant, tasks use it a lot
    robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
    // get jacobian respect link0
    kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
    computeWholeJacobianEE(&robInfo);

    Eigen::Matrix4d eeTtool =
        FRM::invertTransf(robInfo.robotState.wTv_eigen*robInfo.robotState.vTee_eigen)
          * robInfo.transforms.wTt_eigen;

    kdlHelper.setToolSolvers(eeTtool);
    kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.v_Jtool_man));
    computeWholeJacobianTool(&robInfo);

    /// Pass state to controller which deal with tpik
    controller.updateMultipleTasksMatrices(tasksTPIK1, &robInfo);
    std::vector<double> yDotTPIK1 = controller.execAlgorithm(tasksTPIK1);

    std::vector<double> yDotFinal(TOT_DOF);

    /// COORD
    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen =
        robInfo.robotState.w_Jtool_robot * CONV::vector_std2Eigen(yDotTPIK1);

    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen =
        robInfo.robotState.w_Jtool_robot * FRM::pseudoInverse(robInfo.robotState.w_Jtool_robot);


    coordInterface.publishToCoord(nonCoopCartVel_eigen, admisVelTool_eigen);

    while (coordInterface.getCoopCartVel(&(robInfo.exchangedInfo.coopCartVel)) == 1){ //wait for coopVel to be ready
      boost::asio::deadline_timer loopRater2(io2, boost::posix_time::milliseconds(1));
      loopRater2.wait();
      ros::spinOnce();
    }

    controller.updateMultipleTasksMatrices(tasksCoop, &robInfo);
    std::vector<double> yDotOnlyVeh = controller.execAlgorithm(tasksCoop);


    controller.updateMultipleTasksMatrices(tasksArmVehCoord, &robInfo);
    std::vector<double> yDotOnlyArm = controller.execAlgorithm(tasksArmVehCoord);

    for (int i=0; i<ARM_DOF; i++){
      yDotFinal.at(i) = yDotOnlyArm.at(i);

    }

    for (int i = ARM_DOF; i<TOT_DOF; i++) {
      yDotFinal.at(i) = yDotOnlyVeh.at(i);
      //at the moment no error so velocity are exaclty what we are giving
      robInfo.robotState.vehicleVel.at(i-ARM_DOF) = yDotFinal.at(i);
    }




    ///Send command to vehicle
    //robotInterface.sendyDot(yDotTPIK1);
    //robotInterface.sendyDot(yDotOnlyVeh);
    robotInterface.sendyDot(yDotFinal);


    ///Log things
    if (pathLog.size() != 0){
      logger.logAllForTasks(tasksTPIK1);
      //TODO flag LOGGED to not print same thing twice
      //or maybe another list with ALL task only to log here and create folder for log
      //for the moment, yDot are exactly how vehicle and arm are moving
      logger.logNumbers(yDotTPIK1, "yDotTPIK1");
      logger.logNumbers(yDotFinal, "yDotFinal");
      //logger.logNumbers(admisVelTool_eigen, "JJsharp");
      logger.logNumbers(robInfo.robotState.w_Jtool_robot
                              * CONV::vector_std2Eigen(yDotOnlyVeh), "toolCartVelCoop");
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

//    std::vector<Task*> tasksDebug = tasksTPIK1;
//    for(int i=0; i<tasksDebug.size(); i++){
//      std::cout << "Activation " << tasksDebug[i]->getName() << ": \n";
//      tasksDebug[i]->getActivation().PrintMtx() ;
//      std::cout << "\n";
//      std::cout << "JACOBIAN " << tasksDebug[i]->getName() << ": \n";
//      tasksDebug[i]->getJacobian().PrintMtx();
//       std::cout<< "\n";

//      std::cout << "REFERENCE " << tasksDebug[i]->getName() << ": \n";
//      tasksDebug[i]->getReference().PrintMtx() ;
//      std::cout << "\n";
//    }


    controller.resetAllUpdatedFlags(tasksTPIK1);
    controller.resetAllUpdatedFlags(tasksCoop);
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


  ///MISSION TASKS
  Task* pr5 = new PipeReachTask(5, eqType, robotName);
  Task* pr6 = new PipeReachTask(6, eqType, robotName);

  Task* eer = new EndEffectorReachTask(6, eqType, robotName);
  Task* vehR = new VehicleReachTask(3, eqType, robotName, ANGULAR);
  Task* vehYaxis = new VehicleReachTask(1, eqType, robotName, YAXIS);

  /// OPTIMIZATION TASKS
  Task* shape = new ArmShapeTask(4, ineqType, robotName, PEG_GRASPED_PHASE);
  //The "fake task" with all eye and zero matrices, needed as last one for algo?
  Task* last = new LastTask(TOT_DOF, eqType, robotName);

  ///Fill tasks list
  // note: order of priority at the moment is here
  tasks1->push_back(jl);
  tasks1->push_back(ha);
  //tasks1->push_back(eeAvoid);
  tasks1->push_back(pr6);
  //TODO discutere diff tra pr6 e pr5... il 5 mette più stress sull obj?
  //tasks1->push_back(vehR);
  //tasks1->push_back(vehYaxis);
  //tasks1->push_back(eer);
  tasks1->push_back(shape);
  tasks1->push_back(last);

  //TODO ASK al momento fatte così le versioni 6dof di pr e coop sono
  //molto meglio
  //coop 5 dof assolutamente no
  //pr5 da un po di stress cmq (linearmente -0.034, -0.008, -0.020),
  //i due tubi divisi si vedono tanto agli estremi
  //si dovrebbe abbassare gain e saturaz per usare il pr5
  //soprattutto se c'è sto shape fatto cosi che fa muovere tantissimo
  //i bracci, è da cambiare lo shape desiderato
  tasksCoord->push_back(coopTask6dof);
  tasksCoord->push_back(jl);
  tasksCoord->push_back(ha);
  //tasksCoord->push_back(pr6);
  //tasks1->push_back(eeAvoid);
  tasksCoord->push_back(shape);
  tasksCoord->push_back(last);

  tasksArmVeh->push_back(coopTask6dof);
  tasksArmVeh->push_back(constrainVel);
  tasksArmVeh->push_back(jl);
  tasksArmVeh->push_back(ha);
  //tasks1->push_back(eeAvoid);

  tasksArmVeh->push_back(shape);
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



/**
 * @brief setTaskListInit initialize the task list passed as first argument
 * @param *tasks std::vector<Task*> the vector of pointer to the task, passed by reference
 * @param robotName name of the robot
 * @note   note: std::vector is nicer because to change the order of priority or to leave for the moment
 * a task we can simply comment the row.
 * instead, with tasks as Task**, we need to fill the list with task[0], ...task[i] ... and changing
 * priority order would be slower.
 */
void setTaskLists(std::string robotName, std::vector<Task*> *tasks){

  /// PUT HERE NEW TASKS.
  // note: order of priority at the moment is here
  bool eqType = true;
  bool ineqType = false;

  //tasks->push_back(new VehicleNullVelTask(6, ineqType));

  tasks->push_back(new JointLimitTask(4, ineqType, robotName));
  tasks->push_back(new HorizontalAttitudeTask(1, ineqType, robotName));

  //tasks->push_back(new ObstacleAvoidEETask(1, ineqType, robotName));
  //tasks->push_back(new ObstacleAvoidVehicleTask(1, ineqType, robotName));

  //tasks->push_back(new FovEEToToolTask(1, ineqType, robotName));

  //tasks->push_back(new EndEffectorReachTask(6, eqType, robotName));
  tasks->push_back(new PipeReachTask(5, eqType, robotName));
  //tasks->push_back(new VehicleReachTask(6, eqType, robotName));

  tasks->push_back(new ArmShapeTask(4, ineqType, robotName, MID_LIMITS));
  //The "fake task" with all eye and zero matrices, needed as last one for algo
  tasks->push_back(new LastTask(TOT_DOF, eqType, robotName));
}

void setTaskLists(std::string robotName, std::vector<Task*> *tasks1, std::vector<Task*> *tasksFinal){

  bool eqType = true;
  bool ineqType = false;


  /// CONSTRAINT TASKS
  Task* coopTask6dof = new CoopTask(6, eqType, robotName);
  Task* coopTask5dof = new CoopTask(5, eqType, robotName);


  /// SAFETY TASKS
  Task* jl = new JointLimitTask(4, ineqType, robotName);
  Task* ha = new HorizontalAttitudeTask(1, ineqType, robotName);


  /// PREREQUISITE TASKS


  ///MISSION TASKS
  Task* pr5 = new PipeReachTask(5, eqType, robotName);
  Task* pr6 = new PipeReachTask(6, eqType, robotName);

  Task* tr = new EndEffectorReachTask(6, eqType, robotName);

  /// OPTIMIZATION TASKS
  Task* shape = new ArmShapeTask(4, ineqType, robotName, MID_LIMITS);
  //The "fake task" with all eye and zero matrices, needed as last one for algo?
  Task* last = new LastTask(TOT_DOF, eqType, robotName);

  ///Fill tasks list, MID_LIMITS
  // note: order of priority at the moment is here
  tasks1->push_back(jl);
  tasks1->push_back(ha);
  tasks1->push_back(pr5);
  //tasks1->push_back(tr);
  //tasks1->push_back(shape);
  tasks1->push_back(last);

  tasksFinal->push_back(coopTask5dof);
 // tasksFinal->push_back(jl);
  //tasksFinal->push_back(ha);
  //tasksFinal->push_back(shape);
  tasksFinal->push_back(last);
}






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
