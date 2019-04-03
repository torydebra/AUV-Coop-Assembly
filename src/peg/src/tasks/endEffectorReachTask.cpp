#include "header/endEffectorReachTask.h"

/**
 * @brief EndEffectorReachTask::EndEffectorReachTask Constructor of specific task simply calls the parent constructor
 * through inizializer list
 * @param dimension dimension of the task (e.g. 1 for scalar task)
 * @param eqType true or false for equality or inequality task
 * @param activeLog bool to set logger prints
 */
EndEffectorReachTask::EndEffectorReachTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "ENDEFFECTOR_REACHING_GOAL"){
  gain = 0.2;
}

/**
 * @brief VehicleReachTask::updateMatrices overriden of the pure virtual method of Task parent class
 * @param robInfo struct filled with all transformations needed by the task to compute the matrices
 * @return 0 for correct execution
 */
int EndEffectorReachTask::updateMatrices(struct Infos* const robInfo){

  setActivation();

  setJacobian(robInfo->robotState.w_J_robot);

  setReference(robInfo->transforms.wTgoalEE_eigen,
               robInfo->robotState.wTv_eigen*robInfo->robotState.vTee_eigen);
  //setJacobian(transf->vTjoints, transf->vTee_eigen);
  //setReference(transf->vTee_eigen, transf->wTgoalEE_eigen, transf->wTv_eigen);
  return 0;
}

void EndEffectorReachTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){
  J = CONV::matrix_eigen2cmat(w_J_robot);
}

int EndEffectorReachTask::setActivation(){

  double vectDiag[6];
  std::fill_n(vectDiag, 6, 1);
  this->A.SetDiag(vectDiag);

  return 0;

}

int EndEffectorReachTask::setReference(Eigen::Matrix4d wTgoalEE_eigen, Eigen::Matrix4d wTee_eigen){

  CMAT::TransfMatrix wTgoalEE_cmat = CONV::matrix_eigen2cmat(wTgoalEE_eigen);
  CMAT::TransfMatrix wTee_cmat = CONV::matrix_eigen2cmat(wTee_eigen);

  //vTgoalEE_cmat.PrintMtx("vTgoalEE"); ///DEBUG: GIUSTA
  CMAT::Vect6 errorSwapped = CMAT::CartError(wTgoalEE_cmat, wTee_cmat);//ang;lin
  // ang and lin must be swapped because in qDot and jacob linear part is before
  CMAT::Vect6 error;
  error.SetFirstVect3(errorSwapped.GetSecondVect3());
  error.SetSecondVect3(errorSwapped.GetFirstVect3());
  //error.PrintMtx("error"); ///DEBUG: GIUSTOOO
  this->reference = this->gain * (error); //lin; ang


}

//int EndEffectorReachTask::setReference(
//    Eigen::Matrix4d vTee_eigen, Eigen::Matrix4d wTgoalEE_eigen, Eigen::Matrix4d wTv_eigen){

//  CMAT::TransfMatrix vTee_cmat = CONV::matrix_eigen2cmat(vTee_eigen);
//  CMAT::TransfMatrix wTgoalEE_cmat = CONV::matrix_eigen2cmat(wTgoalEE_eigen);
//  CMAT::TransfMatrix wTv_cmat = CONV::matrix_eigen2cmat(wTv_eigen);

//  CMAT::TransfMatrix vTgoalEE_cmat = wTv_cmat.Inverse() * wTgoalEE_cmat;
//  //vTgoalEE_cmat.PrintMtx("vTgoalEE"); ///DEBUG: GIUSTA
//  CMAT::Vect6 errorSwapped = CMAT::CartError(vTgoalEE_cmat, vTee_cmat);//ang;lin
//  // ang and lin must be swapped because in qDot and jacob linear part is before
//  CMAT::Vect6 error;
//  error.SetFirstVect3(errorSwapped.GetSecondVect3());
//  error.SetSecondVect3(errorSwapped.GetFirstVect3());
//  //error.PrintMtx("error"); ///DEBUG: GIUSTOOO
//  this->reference = this->gain * (error); //lin; ang


//}


//TODO considerare anche il tool (wTt)
//int EndEffectorReachTask::setJacobian(std::vector<Eigen::Matrix4d> vTjoints, Eigen::Matrix4d vTee_eigen){

//  Eigen::MatrixXd J_eigen = Eigen::MatrixXd::Zero(dimension, dof);

//  Eigen::Vector3d v_ree = vTee_eigen.topRightCorner(3,1); //position of ee respect vehicle
//  Eigen::Vector3d khat; //versor along z axis
//  khat << 0, 0, 1;

//  /// taking rotation matrices and translational vectors
//  Eigen::Matrix3d v_R_[ARM_DOF];
//  Eigen::Vector3d v_r[ARM_DOF];

//  for (int i =0; i<ARM_DOF; i++){
//    ///DEBUG::POSIZ e ROTAZ SONO CORRETTE
//    v_R_[i] = vTjoints[i].topLeftCorner(3,3);
//    v_r[i] = vTjoints[i].topRightCorner(3,1); //position of joint i respect vehicle
//    //std::cout <<"\n\n JOINT "<< i << ": \n" << v_R_[i] << "\n\n";
//  }

//  ///JOINT part (0 to 3 columns)
//  for (int i = 0; i<ARM_DOF; i++){

//    Eigen::VectorXd g(dimension);
//    g << 0,0,0, 0,0,0;

//    Eigen::Vector3d v_kHat = v_R_[i] * khat; //versor k respect vehicle frame

//    ///DEBUG cross product è GIUSTO vkHat giusto
//    g.head(3) = v_kHat.cross(v_ree - v_r[i]); //linear part
////    std::cout <<"\n\n JOINT "<< i << ": \n" << v_kHat << " x (" << v_ree << " - " << v_r[i] << ")" << "\n"
////             "result: " << g.head(3)<<"\n\n";


//    g.tail(3) = v_kHat; //angular part

//    J_eigen.col(i) = g;
//  }


//  ///VEHICLE part (4 to 9 columns)

//  //linear part (top rows)
//  J_eigen.block<3,3>(0,4) = Eigen::Matrix3d::Identity(); //linear contribution to linear velocities
//  J_eigen.topRightCorner(3,3) = -(FRM::skewMat(v_ree)); //angular contribution to linear velocites

//  //angualr part (bottom rows)
//  J_eigen.block<3,3>(3,4) = Eigen::Matrix3d::Zero(); //linear contribution to angular velocities
//  J_eigen.bottomRightCorner(3,3) = Eigen::Matrix3d::Identity(); //angular contribution to angular velocities

//  this->J = CONV::matrix_eigen2cmat(J_eigen);
//  J.PrintMtx();

//  return 0;
//}
