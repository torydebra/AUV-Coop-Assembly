#include "header/missionManager.h"
#include <chrono>

/**
 * @brief main of missionmanager
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
    std::cout << "[MAIN] Please insert the two robots name"<< std::endl;
    return -1;
  }

  MissionManager missionManagerA(argc, argv);
  missionManagerA.execute();
  return 0;
}




/// DEBUG
//    std::cout << "JACOBIAN " << i << ": \n";
//    tasks[i]->getJacobian().PrintMtx();
//    std::cout<< "\n";
//    std::cout << "REFERENCE " << i << ": \n";
//    tasks[i]->getReference().PrintMtx() ;
//    std::cout << "\n";


//Q.PrintMtx("Q"); ///DEBUG
//qDot_cmat.PrintMtx("qdot");




/*****************************************************************************
 * PRINCIPAL FUNCTION
 ******************************************************************************/


MissionManager::MissionManager(int argc, char **argv)
{
  robotName = argv[1];

  ros::init(argc, argv, robotName + "_MissionManager");
  std::cout << "[" << robotName << "][MISSION_MANAGER] Start" << std::endl;

  if (LOG && (argc > 3)){
    pathLog = argv[3];
  }


}


MissionManager::~MissionManager(){clearTaskList();}


int MissionManager::execute(){

  setGoals();
  initRosInterface();
  initKdlHelper();
  setInitialState();

  //TODO before this the robot must have grasped, here only to see if it is correct
  if (true){ //TODO if grasped
    setKdl4Tool();
  }

  ///Controller
  initController();

  double ms = 100;
  boost::asio::io_service io;
  while(1){
    controlLoop(io, ms);
  }

  return 0;
}




/** ***************************************************************************
 *   FUNCTION OF EXEC
 ***********************************************************************************/

void MissionManager::setGoals(){

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


  //struct transforms to pass them to Controller class
  robInfo.transforms.wTgoalVeh_eigen = wTgoalVeh_eigen;
  robInfo.transforms.wTgoalEE_eigen = wTgoalEE_eigen;
  robInfo.transforms.wTgoalTool_eigen = wTgoalTool_eigen;

}

void MissionManager::initRosInterface(){
  ///Ros interface
  rosInterface(argc, argv, "pipe");
  rosInterface.init();
}

void MissionManager::initKdlHelper(){
  /// KDL parser to after( in the control loop )get jacobian from joint position
  std::string filename = "/home/tori/UWsim/Peg/model/g500ARM5.urdf";
  std::string vehicle = "base_link";
  std::string link0 = "part0";
  std::string endEffector = "end_effector";
  kdlHelper(filename, link0, endEffector);

  kdlHelper.setEESolvers();

  // KDL parse for fixed things (e.g. vehicle and one of his sensor)
  //TODO Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
  kdlHelper.getFixedFrame(vehicle, link0, &(robInfo.robotStruct.vTlink0));
}

void MissionManager::setInitialState(){
  /// Set initial state (todo, same as in control loop, make a function?)
  rosInterface.getJointState(&(robInfo.robotState.jState));
  rosInterface.getwTv(&(robInfo.robotState.wTv_eigen));
  rosInterface.getwTt(&(robInfo.transforms.wTt_eigen));
  rosInterface.getOtherRobPos(&(robInfo.exchangedInfo.otherRobPos));
  //get ee pose RESPECT LINK 0
  kdlHelper.getEEpose(robInfo.robotState.jState, &(robInfo.robotState.link0Tee_eigen));
  // useful have vTee: even if it is redunant, everyone use it a lot
  robInfo.robotState.vTee_eigen = robInfo.robotStruct.vTlink0 * robInfo.robotState.link0Tee_eigen;
  // get jacobian respect link0
  kdlHelper.getJacobianEE(robInfo.robotState.jState, &(robInfo.robotState.link0_Jee_man));
  //get whole jacobian (arm+vehcile and projected on world
  computeWholeJacobian(&robInfo, ee);
}

void MissionManager::setKdl4Tool(){
  Eigen::Matrix4d eeTtool =
      FRM::invertTransf(robInfo.robotState.wTv_eigen*robInfo.robotState.vTee_eigen)
        * robInfo.transforms.wTt_eigen;
  kdlHelper.setToolSolvers(eeTtool);
  kdlHelper.getJacobianTool(robInfo.robotState.jState, &(robInfo.robotState.link0_Jtool_man));
  computeWholeJacobian(&robInfo, tool);
}

void MissionManager::initController(){
  controller(robotName);
  setTaskListInit(&tasks, robotName);
  controller.setTaskList(tasks);
}


void MissionManager::ControlLoop(boost::asio::io_service io, double ms){
  //auto start = std::chrono::steady_clock::now();

  // this must be inside loop
  boost::asio::deadline_timer loopRater(io, boost::posix_time::milliseconds(ms));

  /// Update state
  rosInterface.getJointState(&(robInfo.robotState.jState));
  rosInterface.getwTv(&(robInfo.robotState.wTv_eigen));
  rosInterface.getwTt(&(robInfo.transforms.wTt_eigen));
  rosInterface.getOtherRobPos(&(robInfo.exchangedInfo.otherRobPos));

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
  std::vector<double> qDot = controller.execAlgorithm();


  ///Send command to vehicle
  rosInterface.sendQDot(qDot);
  rosInterface.spinOnce(); // actually the spinonce is called here and not in sendQdot

  /// Timer
//        auto end = std::chrono::steady_clock::now();
//        auto diff = end - start;
//        std::cout << std::chrono::duration<double, std::milli> (diff).count()
//            << " ms" << std::endl;

  loopRater.wait(); //wait for the remaning time until period setted (ms)
}



/** ************************************************************************************************************
 *   OTHER SUBFUNCTIONS
 *****************************************************************************************************************/

/**
 * @brief setTaskListInit initialize the task list passed as first argument
 * @param *tasks std::vector<Task*> the vector of pointer to the task, passed by reference
 * @param robotName name of the robot
 * @note   note: std::vector is nicer because to change the order of priority or to leave for the moment
 * a task we can simply comment the row.
 * instead, with tasks as Task**, we need to fill the list with task[0], ...task[i] ... and changing
 * priority order would be slower.
 */
void MissionManager::setTaskListInit(){

  clearTaskList(); //reset, useful when we will change tasks order for change mission phase
  /// PUT HERE NEW TASKS.
  // note: order of priority at the moment is here
  bool eqType = true;
  bool ineqType = false;

  //tasks->push_back(new VehicleNullVelTask(6, ineqType));

  tasks->push_back(new JointLimitTask(4, ineqType, robotName));
  tasks->push_back(new HorizontalAttitudeTask(1, ineqType, robotName));

  tasks->push_back(new ObstacleAvoidEETask(1, ineqType, robotName));
  tasks->push_back(new ObstacleAvoidVehicleTask(1, ineqType, robotName));


  //tasks->push_back(new FovEEToToolTask(1, ineqType, robotName));

  tasks->push_back(new EndEffectorReachTask(6, eqType, robotName, tool));

  //tasks->push_back(new VehicleReachTask(6, eqType, robotName));

  //The "fake task" with all eye and zero matrices, needed as last one for algo
  tasks->push_back(new LastTask(TOT_DOF, eqType, robotName));

  createLogFolders(); //if setted so
}


/**
 * @brief MissionManager::clearTaskList
 * @note It is important to delete singularly all object pointed by the vector tasks. simply tasks.clear()
 * deletes the pointer but not the object Task pointe
 */
void MissionManager::clearTaskList(){
  for (std::vector< Task* >::iterator it = tasks.begin() ; it != tasks.end(); ++it)
  {
      delete (*it);
  }
  tasks.clear();
}


void MissionManager::createLogFolders(){
  /// Log folders
  //if flag log setted to 1 and path log is given
  if (pathLog.size() != 0){

    for (int i =0; i< tasks.size(); ++i){
      PRT::createDirectory(pathLog +"/" +tasks[i]->getName());
    }
    std::cout << "[" << robotName
              << "][MISSION_MANAGER] Created Log Folders in  "
              << pathLog  << std::endl;
  }
}





