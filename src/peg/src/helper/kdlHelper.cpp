#include "header/kdlHelper.h"
/**
 * @brief KDLHelper::KDLHelper
 * @param filename path and name of the urdf file of the robot (arm + vehicle is ok)
 * @param link0_name name in the urdf file of the frame of the joint 0 (actually, name of <link></link>)
 * @param endEffector_name name in the urdf file of the frame of the end effector (actually, name of <link></link>)
 */
KDLHelper::KDLHelper(std::string filename, std::string link0_name,
                     std::string endEffector_name, std::string vehicle_name)
{

  TiXmlDocument file_xml(filename);
  file_xml.LoadFile();

  // parse URDF arm model

  urdf::Model model;
  if (!model.initXml(&file_xml)){
     std::cerr << "[KDL_HELPER] Failed to parse urdf robot model\n" ;
     return ;
  }
  if (!kdl_parser::treeFromUrdfModel(model, tree)){
     std::cerr << "[KDL_HELPER] Failed to construct kdl tree\n";
     return ;
  }

  nJoints = 0;
  this->link0_name = link0_name;
  this->endEffector_name = endEffector_name;
  this->vehicle_name = vehicle_name;

}

KDLHelper::~KDLHelper(){

  delete eePose_solver;
  delete jacobEE_solver;
  delete jacobTool_solver;

}



/**
 * @brief KDLHelper::setEESolvers Initialize the kinematic solvers (jacob and pose)
 * for the arm End Effector
 * @return 0 for correct execution
 * @note DH (denavit hartenberg) convention is used.
 * So, link 0 is the base of the arm (FIXED respect to the vehicle)
 * @note it is ok do the same tree.getChain in this and other function
 * because these setxxxSolvers are called only once to associate the solver to the kin chain
 */
int KDLHelper::setEESolvers(){

  //generate the chain from link0 to final (end effector)
  KDL::Chain kinChain;
  tree.getChain(link0_name, endEffector_name, kinChain);

  nJoints = kinChain.getNrOfJoints();
  if (nJoints != ARM_DOF){
    std::cerr << "[KDL_HELPER] Something wrong with urdf:\n" <<
                 "I have found from urdf file " << nJoints << " joints " <<
                 "but from define list file I read " << ARM_DOF <<". Please check\n";
    return -1;
  }

  std::cout << "[KDL_HELPER] found " << nJoints << " joints from " << link0_name
            << " to " << endEffector_name << "\n";

  eePose_solver = new KDL::ChainFkSolverPos_recursive(kinChain);
  jacobEE_solver = new KDL::ChainJntToJacSolver(kinChain);


  return 0;
}



/**
 * @brief KDLHelper::setToolSolvers
 * @return 0 correct execution
 * @note it is ok do the same tree.getChain in this and other function
 * because these setxxxSolvers are called only once to associate the solver to the kin chain
 */
int KDLHelper::setToolSolvers(Eigen::Matrix4d eeTtool_eigen){

  KDL::Chain kinChain;

  tree.getChain(vehicle_name, endEffector_name, kinChain);
  // now kinChain is a chain for only the arm


  // now we add a fake joint to add to kinChain a segment which is for the tool
  //(that is fixed respect to the ee)
  KDL::Frame eeTtool_kdl = CONV::transfMatrix_eigen2kdl(eeTtool_eigen);

  kinChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), eeTtool_kdl));

  //even if we add a segment, the number of joint of kin chain remain 4. Probably because we add a fixed joint.
  //however, this is good.

  /// kinChain until tool now
  jacobTool_solver = new KDL::ChainJntToJacSolver(kinChain);

}

int KDLHelper::getEEpose(std::vector<double> jointPos, Eigen::Matrix4d *eePose_eigen){

  KDL::JntArray jointPositions = KDL::JntArray(nJoints);
  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=jointPos[i];
  }
  KDL::Frame eePose_kdl;
  eePose_solver->JntToCart(jointPositions, eePose_kdl);

  *eePose_eigen = CONV::transfMatrix_kdl2eigen(eePose_kdl);

  return 0;

}


/**
 * @brief KDLHelper::getJacobianEE respect base frame of arm (link0)
 * @param jointPos
 * @param jacobianEE_eigen
 * @return
 */
int KDLHelper::getJacobianEE(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobianEE_eigen){

  KDL::JntArray jointPositions = KDL::JntArray(nJoints);

  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=jointPos[i];
  }

  KDL::Jacobian jacobian_kdl;
  jacobian_kdl.resize(nJoints);
  jacobEE_solver->JntToJac(jointPositions, jacobian_kdl);

  *jacobianEE_eigen = CONV::jacobian_kdl2eigen(jacobian_kdl);

  return 0;
}

/**
* @brief KDLHelper::getJacobianTool respect base frame of arm (link0)
* @param jointPos
* @param jacobianTool_eigen
* @return
*/
int KDLHelper::getJacobianTool(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobianTool_eigen){

  KDL::JntArray jointPositions = KDL::JntArray(nJoints);

  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=jointPos[i];
  }

  KDL::Jacobian jacobian_kdl;
  jacobian_kdl.resize(nJoints); //il fixed non è considerato joint
  jacobTool_solver->JntToJac(jointPositions, jacobian_kdl);

  *jacobianTool_eigen = CONV::jacobian_kdl2eigen(jacobian_kdl);

  return 0;
}



/**
 * @brief KDLHelper::getFixedFrame get relative position of things inside the urdf file
 * of the robot that NOT change their relative position
 * (e.g. vehicle and a sensor fixed on the vehicle )
 * @param frameOrigin
 * @param frameTarget
 * @param *xTx_eigen FIXED transformation between origin and target, passed by reference
 * @return 0 correct execution
 * @warning DO NOT USE for trasformations that change during the mission.
 * @todo Maybe exist an easier method to parse fixed frame from urdf without needed of kdl solver
 */
int KDLHelper::getFixedFrame(std::string frameOrigin, std::string frameTarget, Eigen::Matrix4d *xTx_eigen){

  //generate the chain
  KDL::Chain kinChain;
  tree.getChain(frameOrigin, frameTarget, kinChain);

  int nJoints = kinChain.getNrOfJoints();
  if (nJoints != 0) {
    std::cerr << "[KDL_HELPER] Use this function only for FIXED things relative each other\n";
    return -1;
  }

  KDL::ChainFkSolverPos_recursive transfSolver(kinChain);

  //joint pos are needed to the function of the solver but they does not count for fixed thing
  KDL::JntArray jointPositions = KDL::JntArray(nJoints);
  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=0.0;
  }

  KDL::Frame xTx_kdl;
  transfSolver.JntToCart(jointPositions, xTx_kdl);

  *xTx_eigen = CONV::transfMatrix_kdl2eigen(xTx_kdl);

  return 0;
}


int KDLHelper::getNJoints(){return nJoints;}


















/**
 * @brief KDLHelper::getToolpose OnLY FOR DEBUG FOR NOW
 * @param jointPos
 * @param eePose_eigen
 * @return
 */
int KDLHelper::getToolpose(std::vector<double> jointPos, Eigen::Matrix4d eeTtool_eigen, Eigen::Matrix4d *toolPose_eigen){

  KDL::JntArray jointPositions = KDL::JntArray(nJoints);
  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=jointPos[i];
  }
  KDL::Frame toolPose_kdl;
  KDL::Chain kinChain;
  tree.getChain(link0_name, endEffector_name, kinChain);
  KDL::Frame eeTtool_kdl = CONV::transfMatrix_eigen2kdl(eeTtool_eigen);

  kinChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), eeTtool_kdl));
  KDL::ChainFkSolverPos_recursive toolPose_solver(kinChain);

  toolPose_solver.JntToCart(jointPositions, toolPose_kdl);

  *toolPose_eigen = CONV::transfMatrix_kdl2eigen(toolPose_kdl);

  return 0;

}











/**
 * @brief KDLHelper::setToolSolvers_old the one with link 0
 * @return 0 correct execution
 * @note it is ok do the same tree.getChain in this and other function
 * because these setxxxSolvers are called only once to associate the solver to the kin chain
 */
int KDLHelper::setToolSolvers_old(Eigen::Matrix4d eeTtool_eigen){

  KDL::Chain kinChain;

  tree.getChain(link0_name, endEffector_name, kinChain);
  // now kinChain is a chain for only the arm

  // now we add a fake joint to add to kinChain a segment which is for the tool
  //(that is fixed respect to the ee)
  KDL::Frame eeTtool_kdl = CONV::transfMatrix_eigen2kdl(eeTtool_eigen);

  kinChain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::None), eeTtool_kdl));

  //even if we add a segment, the number of joint of kin chain remain 4. Probably because we add a fixed joint.
  //however, this is good.

  /// kinChain until tool now
  jacobTool_solver = new KDL::ChainJntToJacSolver(kinChain);

}
