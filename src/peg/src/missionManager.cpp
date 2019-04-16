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
  wTgoalEE_eigen.topLeftCorner<3,3>() = Eigen::Matrix3d::Identity();
//  wTgoalEE_eigen.topLeftCorner(3,3) << 0.5147, 0 , -0.8574,
//                                          0    ,1,     0    ,
//                                        0.8573 , 0  , 0.5147;
  //trasl part
  wTgoalEE_eigen(0, 3) = goalLinearVectEE[0];
  wTgoalEE_eigen(1, 3) = goalLinearVectEE[1];
  wTgoalEE_eigen(2, 3) = goalLinearVectEE[2];

  /// GOAL TOOL
  double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
  //double goalLinearVectTool[] = {-0.27, -3.102, 5.124};

  // (rob and world have downward z)
  Eigen::Matrix4d wTgoalTool_eigen = Eigen::Matrix4d::Identity();
  //rot part
  wTgoalTool_eigen.topLeftCorner(3,3) = Eigen::Matrix3d::Identity();

  //trasl part
  wTgoalTool_eigen(0, 3) = goalLinearVectTool[0];
  wTgoalTool_eigen(1, 3) = goalLinearVectTool[1];
  wTgoalTool_eigen(2, 3) = goalLinearVectTool[2];


  /// struct container data to pass among functions
  Infos robInfo;
  //struct transforms to pass them to Controller class
  robInfo.transforms.wTgoalVeh_eigen = wTgoalVeh_eigen;
  robInfo.transforms.wTgoalEE_eigen = wTgoalEE_eigen;
  robInfo.transforms.wTgoalTool_eigen = wTgoalTool_eigen;
  robInfo.robotState.vehicleVel.resize(VEHICLE_DOF);
  std::fill(robInfo.robotState.vehicleVel.begin(), robInfo.robotState.vehicleVel.end(), 0);


  ///Ros interfaces
  RobotInterface robotInterface(nh, robotName, otherRobotName);
  robotInterface.init();
  //WorldInterface worldInterface(robotName, "pipe");
  //debug
  WorldInterface worldInterface;
  if (robotName.compare("g500_A") == 0){
    worldInterface = WorldInterface(robotName, "pipe");
  } else {
    worldInterface = WorldInterface(robotName, "pipe2");
  }
  worldInterface.init();
  CoordInterfaceMissMan coordInterface(nh, robotName);


  /// KDL parser to after( in the control loop )get jacobian from joint position
  std::string filename = "/home/tori/UWsim/Peg/model/g500ARM5.urdf";
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
  //std::string debug = "peg";
  KDLHelper kdlHelper(filenamePeg, link0, endEffector);
  kdlHelper.setEESolvers();
  // KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));


  /// Set initial state (todo, same as in control loop, make a function?)
  robotInterface.getJointState(&(robInfo.robotState.jState));
  robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
  worldInterface.getwTt(&(robInfo.transforms.wTt_eigen));
  robInfo.transforms.wTgoalTool_eigen.topLeftCorner<3,3>() = robInfo.transforms.wTt_eigen.topLeftCorner<3,3>();
  robotInterface.getOtherRobPos(&(robInfo.exchangedInfo.otherRobPos));

  //get ee pose RESPECT LINK 0
  kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
  // useful have vTee: even if it is redunant, everyone use it a lot
  robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
  // get jacobian respect link0
  kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
  //get whole jacobian (arm+vehcile and projected on world
  computeWholeJacobian(&robInfo, ee);


  //TODO before this the robot must have grasped, here only to see if it is correct
  if (true){ //TODO if grasped
    Eigen::Matrix4d eeTtool =
        FRM::invertTransf(robInfo.robotState.wTv_eigen*robInfo.robotState.vTee_eigen)
          * robInfo.transforms.wTt_eigen;
    kdlHelper.setToolSolvers(eeTtool);
  }
  kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.link0_Jtool_man));
  computeWholeJacobian(&robInfo, tool);

  ///DEBUG... correct Jacobian
  //std::cout << "JACOBIAN urdf\n" << robInfo.robotState.link0_Jee_man <<"\n\n";
  //std::cout << "JACOBIAN addSeg\n" << robInfo.robotState.link0_Jtool_man <<"\n\n";


  ///Controller
  Controller controller(robotName);

  /// Task lists
  std::vector<Task*> tasksTPIK1; //the first, non coop, one
  std::vector<Task*> tasksCoop;
  std::vector<Task*> tasksArmVehCoord; //for arm-vehicle coord
  setTaskLists(robotName, &tasksTPIK1, &tasksCoop, &tasksArmVehCoord);


  /// Log folders TO DO AFTER EVERY SETTASKLIST?
  /// //TODO ulteriore subfolder per la mission

  Logger logger;
  if (pathLog.size() > 0){
    logger = Logger(robotName, pathLog);

    //to for each list... if task was in a previous list, no problem, folder with same name... overwrite?
    logger.createTasksListDirectories(tasksTPIK1);
    logger.createTasksListDirectories(tasksCoop);

  }

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


  int ms = 100;
  boost::asio::io_service io;
  boost::asio::io_service io2;

  while(1){
    auto start = std::chrono::steady_clock::now();

    // this must be inside loop
    boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));

    /// Update state
    robotInterface.getJointState(&(robInfo.robotState.jState));
    robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
    worldInterface.getwTt(&(robInfo.transforms.wTt_eigen));
    robotInterface.getOtherRobPos(&(robInfo.exchangedInfo.otherRobPos));

    //get ee pose RESPECT LINK 0
    kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
    // useful have vTee: even if it is redunant, tasks use it a lot
    robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
    // get jacobian respect link0
    kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
    computeWholeJacobian(&robInfo, ee);
    //if(grasped)
    kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.link0_Jtool_man));
    computeWholeJacobian(&robInfo, tool);


    /// Pass state to controller which deal with tpik
    controller.updateMultipleTasksMatrices(tasksTPIK1, &robInfo);
    std::vector<double> yDotTPIK1 = controller.execAlgorithm(tasksTPIK1);


    std::vector<double> yDotFinal(TOT_DOF);

    /// COORD
    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen =
        robInfo.robotState.w_Jtool_robot * CONV::vector_std2Eigen(yDotTPIK1);

    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen =
        robInfo.robotState.w_Jtool_robot * FRM::pseudoInverse(robInfo.robotState.w_Jtool_robot);

    std::cout << "JJsharp\n" << admisVelTool_eigen << "\n\n";

    coordInterface.publishToCoord(nonCoopCartVel_eigen, admisVelTool_eigen);

    while (coordInterface.getCoopCartVel(&(robInfo.exchangedInfo.coopCartVel)) == 1){ //wait for coopVel to be ready
      boost::asio::deadline_timer loopRater2(io2, boost::posix_time::milliseconds(1));
      loopRater2.wait();
      ros::spinOnce();
    }

    std::cout << "SENDED non coop vel:\n" << nonCoopCartVel_eigen <<"\n\n\n";
    std::cout << "ARRIVED coop vel:\n" << robInfo.exchangedInfo.coopCartVel << "\n\n\n";

    controller.resetAllUpdatedFlags(tasksTPIK1);
    controller.resetAllUpdatedFlags(tasksCoop);
    controller.resetAllUpdatedFlags(tasksArmVehCoord);
    controller.updateMultipleTasksMatrices(tasksCoop, &robInfo);
    std::vector<double> yDotOnlyVeh = controller.execAlgorithm(tasksCoop);


    controller.resetAllUpdatedFlags(tasksTPIK1);
    controller.resetAllUpdatedFlags(tasksCoop);
    controller.resetAllUpdatedFlags(tasksArmVehCoord);


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
    robotInterface.sendyDot(yDotFinal);


    ///Log things
    if (pathLog.size() != 0){
      logger.writeAllForTasks(tasksTPIK1);
      //TODO flag LOGGED to not print same thing twice
      //or maybe another list with ALL task only to log here and create folder for log
      //for the moment, yDot are exactly how vehicle and arm are moving
      logger.writeYDot(yDotTPIK1, "yDotTPIK1");
      logger.writeYDot(yDotFinal, "yDotFinal");
      logger.writeEigenMatrix(admisVelTool_eigen, "JJsharp");
    }

    ///PRINT
    /// DEBUG
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


    auto end = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( end - start ).count();
    std::cout << "TEMPOOOOO  "<<duration << "\n\n";


    loopRater.wait();

  } //end controol loop




  coordInterface.pubIamReadyOrNot(false);

  //TODO
  //destroy() per la task init list;
  return 0;
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

  tasks->push_back(new ArmShapeTask(4, ineqType, robotName));
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
  Task* shape = new ArmShapeTask(4, ineqType, robotName);
  //The "fake task" with all eye and zero matrices, needed as last one for algo?
  Task* last = new LastTask(TOT_DOF, eqType, robotName);

  ///Fill tasks list
  // note: order of priority at the moment is here
  tasks1->push_back(jl);
  tasks1->push_back(ha);
  tasks1->push_back(pr5);
  //tasks1->push_back(tr);
  //tasks1->push_back(shape);
  tasks1->push_back(last);

  tasksFinal->push_back(coopTask5dof);
  tasksFinal->push_back(jl);
  tasksFinal->push_back(ha);
  //tasksFinal->push_back(shape);
  tasksFinal->push_back(last);
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


  /// PREREQUISITE TASKS


  ///MISSION TASKS
  Task* pr5 = new PipeReachTask(5, eqType, robotName);
  Task* pr6 = new PipeReachTask(6, eqType, robotName);

  Task* tr = new EndEffectorReachTask(6, eqType, robotName);

  /// OPTIMIZATION TASKS
  Task* shape = new ArmShapeTask(4, ineqType, robotName);
  //The "fake task" with all eye and zero matrices, needed as last one for algo?
  Task* last = new LastTask(TOT_DOF, eqType, robotName);

  ///Fill tasks list
  // note: order of priority at the moment is here
  tasks1->push_back(jl);
  tasks1->push_back(ha);
  tasks1->push_back(pr6);
  //tasks1->push_back(tr);
  //tasks1->push_back(shape);
  tasks1->push_back(last);

  tasksCoord->push_back(coopTask6dof);
  tasksCoord->push_back(jl);
  tasksCoord->push_back(ha);
  //tasksFinal->push_back(shape);
  tasksCoord->push_back(last);

  tasksArmVeh->push_back(constrainVel);
  tasksArmVeh->push_back(jl);
  tasksArmVeh->push_back(ha);
  //tasksArmVeh->push_back(shape);
  tasksArmVeh->push_back(last);

}


//TODO LA DESTROY
/** @brief
 * @note It is important to delete singularly all object pointed by the vector tasks. simply tasks.clear()
 * deletes the pointer but not the object Task pointed
*/
//Controller::~Controller(){
//  for (std::vector< Task* >::iterator it = tasks.begin() ; it != tasks.end(); ++it)
//    {
//      delete (*it);
//    }
//    tasks.clear();
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
