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
    pathLog += "/" + robotName;
  }
  start_glob = false;


  /// ROS NODE
  ros::init(argc, argv, robotName + "_MissionManager");
  std::cout << "[" << robotName << "][MISSION_MANAGER] Start" << std::endl;
  ros::NodeHandle nh;
  //pub to let coord that is ready (e.g. to see if both missManager A & B started)
  // ASK towrite in doc  _missionManager" topic are for mission manager topic...
  //withotu missman (only robname) are topic of robotInterface
  std::string topicReady = "/uwsim/" + robotName+"_MissionManager"  + "/ready";
  ros::Publisher readyPub = nh.advertise<std_msgs::Bool>(topicReady, 1);

  //sub to know if other robot is ready
  //even if we can read directly the topic of the other robot,
  // for consistency ALL infos received from extern (ie subscribed topic)
  // are given by coordinator.
  // for make each miss manager comunicate with its own robot, there is
  // the ros interface
  std::string topicStart = "/uwsim/Coord/StartStopBoth";
  ros::Subscriber startSub = nh.subscribe(topicStart, 1, startSubCallback);



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

  /// GOAL TOOL
  double goalLinearVectTool[] = {-0.27, -0.102, 2.124};
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


  ///Ros interfaces
  RobotInterface robotInterface(nh, robotName, otherRobotName, "pipe");
  robotInterface.init();
  CoordInterfaceMissMan coordInterface(nh, robotName);


  /// KDL parser to after( in the control loop )get jacobian from joint position
  std::string filename = "/home/tori/UWsim/Peg/model/g500ARM5.urdf";
  std::string vehicle = "base_link";
  std::string link0 = "part0";
  std::string endEffector = "end_effector";
  KDLHelper kdlHelper(filename, link0, endEffector);
  kdlHelper.setEESolvers();
  // KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));


  /// Set initial state (todo, same as in control loop, make a function?)
  robotInterface.getJointState(&(robInfo.robotState.jState));
  robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
  robotInterface.getwTt(&(robInfo.transforms.wTt_eigen));
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


  ///Controller
  Controller controller(robotName);
  std::vector<Task*> tasks;
  setTaskListInit(&tasks, robotName);
  controller.setTaskList(tasks);


  /// Log folders TO DO AFTER EVERY SETTASKLIST?
  /// //TODO ulteriore subfolder per la mission

  if (pathLog.size() > 0){
    for (int i =0; i< tasks.size(); ++i){
      PRT::createDirectory(pathLog +"/" +tasks[i]->getName());
    }

    std::cout << "[" << robotName
              << "][MISSION_MANAGER] Created Log Folders in  "
              << pathLog  << std::endl;
  }

  //wait coordinator to start TODO put in coordInterface
  std_msgs::Bool ready;
  ready.data = true;
  double msinit = 100;
  boost::asio::io_service ioinit;
  while(!start_glob){
    boost::asio::deadline_timer loopRater(ioinit, boost::posix_time::milliseconds(msinit));
    readyPub.publish(ready);
    std::cout << "[" << robotName<< "][MISSION_MANAGER] Wating for "<<
                 "Coordinator to say I can start...\n";
    ros::spinOnce();
    loopRater.wait();
  }


  int ms = 100;
  boost::asio::io_service io;
  while(1){
    //auto start = std::chrono::steady_clock::now();

    // this must be inside loop
    boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));

    /// Update state
    robotInterface.getJointState(&(robInfo.robotState.jState));
    robotInterface.getwTv(&(robInfo.robotState.wTv_eigen));
    robotInterface.getwTt(&(robInfo.transforms.wTt_eigen));
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
    controller.updateTransforms(&robInfo);
    std::vector<double> yDot = controller.execAlgorithm();

    /// COORD
    ///
    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVel_eigen =
        robInfo.robotState.w_Jtool_robot * CONV::vector_std2Eigen(yDot);

    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelTool_eigen =
        robInfo.robotState.w_Jtool_robot * FRM::pseudoInverse(robInfo.robotState.w_Jtool_robot);

    std::cout << nonCoopCartVel_eigen << "\n\n";
    std::cout << admisVelTool_eigen << "\n\n";

    coordInterface.publishToCoord(&robInfo, yDot);

    ///Send command to vehicle
    robotInterface.sendyDot(yDot);


    ///Log things
    if (pathLog.size() != 0){
      for(int i=0; i<tasks.size(); i++){
        std::string pathname = pathLog + "/" + tasks[i]->getName();
        PRT::matrixCmat2file(pathname + "/activation.txt",
                             tasks[i]->getActivation().GetDiag());
        PRT::matrixCmat2file(pathname + "/reference.txt",
                             tasks[i]->getReference());
        PRT::matrixCmat2file(pathname + "/error.txt",
                             tasks[i]->getError());

      }
      //for the moment, yDot are exactly how vehicle and arm are moving
      std::string pathyDot = pathLog + "/yDot.txt";
      PRT::vectorStd2file(pathyDot, yDot);
    }

//    ///PRINT
//    for(int i=3; i<tasks.size(); i++){
//      /// DEBUG
//      std::cout << "Activation " << tasks[i]->getName() << ": \n";
//      tasks[i]->getActivation().PrintMtx() ;
//      std::cout << "\n";
////      std::cout << "JACOBIAN " << tasks[i]->getName() << ": \n";
////      tasks[i]->getJacobian().PrintMtx();
////      std::cout<< "\n";
//      std::cout << "REFERENCE " << tasks[i]->getName() << ": \n";
//      tasks[i]->getReference().PrintMtx() ;
//      std::cout << "\n";
//    }


    ros::spinOnce();
    loopRater.wait();
  }




  //TODO
  //destroy();
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
void setTaskListInit(std::vector<Task*> *tasks, std::string robotName){

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

  //tasks->push_back(new EndEffectorReachTask(6, eqType, robotName, tool));
  tasks->push_back(new PipeReachTask(5, eqType, robotName));
  //tasks->push_back(new VehicleReachTask(6, eqType, robotName));

  tasks->push_back(new ArmShapeTask(4, ineqType, robotName));
  //The "fake task" with all eye and zero matrices, needed as last one for algo
  tasks->push_back(new LastTask(TOT_DOF, eqType, robotName));
}


void startSubCallback(const std_msgs::Bool::ConstPtr& start){
  start_glob = start->data;
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


//Q.PrintMtx("Q"); ///DEBUG
//yDot_cmat.PrintMtx("yDot");
