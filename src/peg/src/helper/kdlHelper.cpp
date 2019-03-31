/// TODO divide parseURDF and CreateChain... so parse urdf is used only once

#include "header/kdlHelper.h"
/**
 * @brief KDLHelper::KDLHelper
 * @param filename path and name of the urdf file of the robot (arm + vehicle is ok)
 */
KDLHelper::KDLHelper(std::string filename)
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

}

KDLHelper::~KDLHelper(){

  delete eePose_solver;
  delete jacob_solver;
}



/**
 * @brief KDLHelper::setSolvers Initialize the kinematic solvers (jacob and pose) for the arm
 * @param link0 name in the urdf file of the frame of the joint 0 (actually, name of <link></link>)
 * @param endEffector name in the urdf file of the frame of the end effector (actually, name of <link></link>)
 * @return 0 for correct execution
 * @note DH (denavit hartenberg) convention is used. So, link 0 is the base of the arm (FIXED respect to the vehicle)
 */
int KDLHelper::setSolvers(std::string link0, std::string endEffector){

  //generate the chain from link0 to final (end effector)
  KDL::Chain kinChain;
  tree.getChain(link0, endEffector, kinChain);

  nJoints = kinChain.getNrOfJoints();
  if (nJoints != ARM_DOF){
    std::cerr << "[KDL_HELPER] Something wrong with urdf:\n" <<
                 "I have found from urdf file " << nJoints << " joints " <<
                 "but from define list file I read " << ARM_DOF <<". Please check\n";
    return -1;
  }

  std::cout << "[KDL_HELPER] found " << nJoints << " joints from " << link0 << " to " << endEffector << "\n";

  eePose_solver = new KDL::ChainFkSolverPos_recursive(kinChain);
  jacob_solver = new KDL::ChainJntToJacSolver(kinChain);

  return 0;
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

int KDLHelper::getJacobian(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobian_eigen){

  KDL::JntArray jointPositions = KDL::JntArray(nJoints);

  for(unsigned int i=0; i<nJoints; i++){
    jointPositions(i)=jointPos[i];
  }

  KDL::Jacobian jacobian_kdl;
  jacobian_kdl.resize(nJoints);
  jacob_solver->JntToJac(jointPositions, jacobian_kdl);

  *jacobian_eigen = CONV::jacobian_kdl2eigen(jacobian_kdl);

  return 0;
}

int KDLHelper::getNJoints(){
  return nJoints;
}

/**
 * @brief KDLHelper::getFixedFrame get relative position of things that NOT change
 * their relative position (e.g. vehicle and a sensor fixed on the vehicle )
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



