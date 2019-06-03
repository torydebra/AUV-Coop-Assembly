#include "header/obstacleAvoidEETask.h"


ObstacleAvoidEETask::ObstacleAvoidEETask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "OBSTACLE_AVOID_EE"){

  gain = 0.6;
  safe_dist = 0.3; //in meters

}


int ObstacleAvoidEETask::updateMatrices(struct Infos* const robInfo){

  double wZseafloor = SEAFLOOR_DEPTH;
  Eigen::Matrix4d wTee = robInfo->robotState.wTv_eigen * robInfo->robotState.vTee_eigen;
  Eigen::Vector3d w_ee_posiz= wTee.topRightCorner<3,1>();

  // the distance is a scalar because only along z axis
  double w_dist = wZseafloor - w_ee_posiz(2);

  // if norm is too little we cant provide a ref because will tend to infinite;
  // so we set a minimu value possible
  // However, if norm is so little the robot are already compenetrating so problems are other:
  //gain, safe_dist, and veh_dim maybe are too little
  if (w_dist < 0.001) {
    std::cout << "[" << robotName << "][" << taskName << "] WARNING: Norm too little,"
              << " I will set it to 0.001" << std::endl;
    w_dist = 0.001;
  }

  setActivation(w_dist);
  //A.PrintMtx("A");

  setJacobian(robInfo->robotState.w_Jee_robot);
  //J.PrintMtx("J");

  setReference(w_dist);
  //reference.PrintMtx("REF");


  return 0;
}

void ObstacleAvoidEETask::setActivation(double w_dist){

  A(1) = CMAT::DecreasingBellShapedFunction(0, 0 + safe_dist, 0, 1, w_dist);

}

void ObstacleAvoidEETask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){

  Eigen::MatrixXd jacobian_eigen(dimension, TOT_DOF);
  jacobian_eigen = Eigen::MatrixXd::Zero(dimension, TOT_DOF);

  Eigen::Vector3d zComponent;
  zComponent << 0, 0, 1;

  //top rows are the linear part of jacobian
  jacobian_eigen = - zComponent.transpose() * w_J_robot.topRows(3);

  J = CONV::matrix_eigen2cmat(jacobian_eigen);

}

void ObstacleAvoidEETask::setReference(double w_dist){

  //TODO check where the frame is to understand well distance minimum

  //if the frame is in the middle, distance minim is two times the half of lenght
  //(plus the safe dist)
  //(1): we know this is a scalar task
  this->reference(1) = gain * (safe_dist - w_dist);

}


