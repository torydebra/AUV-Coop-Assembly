#include "header/graspConstrainer.h"

GraspConstrainer::GraspConstrainer() {} //private, uncallable

std::vector<double> GraspConstrainer::calculateGraspVelocities_arm(
    Eigen::Matrix<double, 6, ARM_DOF> w_J_man,
    Eigen::Matrix4d otherEE_T_EE) {

  CMAT::TransfMatrix otherEE_T_EE_cmat = CONV::matrix_eigen2cmat(otherEE_T_EE);
  CMAT::TransfMatrix desiredT = CMAT::Matrix::Eye(4);
  CMAT::Matrix w_J_man_cmat = CONV::matrix_eigen2cmat(w_J_man);

  CMAT::Vect6 errorSwapped = CMAT::CartError(desiredT, otherEE_T_EE_cmat);//ang;lin
  // ang and lin must be swapped because in yDot and jacob linear part is before
  CMAT::Vect6 error;
  error.SetFirstVect3(errorSwapped.GetSecondVect3());
  error.SetSecondVect3(errorSwapped.GetFirstVect3());

  //gain
  error = 1 * error;

  //saturate
  //error.SetFirstVect3(FRM::saturateCmat(error.GetFirstVect3(), 0.5)); //linear
  //error.SetSecondVect3(FRM::saturateCmat(error.GetSecondVect3(), 0.2)); //angular

  CMAT::Matrix velocities_cmat =
      (w_J_man_cmat.RegPseudoInverse(THRESHOLD_DEFAULT, LAMBDA_DEFAULT)) * error;


  std::vector<double> velocities;
  velocities.resize(ARM_DOF);  //to be sure about dimension
  velocities = CONV::vector_cmat2std(velocities_cmat);

  return velocities;

}

std::vector<double> GraspConstrainer::calculateGraspVelocities_armVeh(
    Eigen::Matrix<double, 6, TOT_DOF> w_Jee_robot,
    Eigen::Matrix4d wTotherPeg, Eigen::Matrix4d wTee) {

  CMAT::TransfMatrix wTotherPeg_cmat = CONV::matrix_eigen2cmat(wTotherPeg);
  CMAT::TransfMatrix wTee_cmat = CONV::matrix_eigen2cmat(wTee);
  CMAT::Matrix w_Jee_robot_cmat = CONV::matrix_eigen2cmat(w_Jee_robot);

  CMAT::Vect6 errorSwapped = CMAT::CartError(wTotherPeg_cmat, wTee_cmat);//ang;lin
  // ang and lin must be swapped because in yDot and jacob linear part is before
  CMAT::Vect6 error;
  error.SetFirstVect3(errorSwapped.GetSecondVect3());
  error.SetSecondVect3(errorSwapped.GetFirstVect3());

  //gain
  error = 1 * error;

  //saturate
  //error.SetFirstVect3(FRM::saturateCmat(error.GetFirstVect3(), 0.5)); //linear
  //error.SetSecondVect3(FRM::saturateCmat(error.GetSecondVect3(), 0.2)); //angular

  CMAT::Matrix velocities_cmat =
      (w_Jee_robot_cmat.RegPseudoInverse(THRESHOLD_DEFAULT, LAMBDA_DEFAULT)) * error;


  std::vector<double> velocities;
  velocities.resize(TOT_DOF);  //to be sure about dimension
  velocities = CONV::vector_cmat2std(velocities_cmat);

  return velocities;



}


//std::vector<double> GraspConstrainer::calculateGraspVelocities_armVeh(
//    Eigen::Matrix<double, 6, TOT_DOF> w_Jee_robot,
//    Eigen::Matrix4d otherEE_T_EE) {

//  CMAT::TransfMatrix otherEE_T_EE_cmat = CONV::matrix_eigen2cmat(otherEE_T_EE);
//  CMAT::TransfMatrix desiredT = CMAT::Matrix::Eye(4);
//  CMAT::Matrix w_Jee_robot_cmat = CONV::matrix_eigen2cmat(w_Jee_robot);

//  CMAT::Vect6 errorSwapped = CMAT::CartError(desiredT, otherEE_T_EE_cmat);//ang;lin
//  // ang and lin must be swapped because in yDot and jacob linear part is before
//  CMAT::Vect6 error;
//  error.SetFirstVect3(errorSwapped.GetSecondVect3());
//  error.SetSecondVect3(errorSwapped.GetFirstVect3());

//  //gain
//  error = 0.1 * error;

//  //saturate
//  //error.SetFirstVect3(FRM::saturateCmat(error.GetFirstVect3(), 0.5)); //linear
//  //error.SetSecondVect3(FRM::saturateCmat(error.GetSecondVect3(), 0.2)); //angular

//  CMAT::Matrix velocities_cmat =
//      (w_Jee_robot_cmat.RegPseudoInverse(THRESHOLD_DEFAULT, LAMBDA_DEFAULT)) * error;


//  std::vector<double> velocities;
//  velocities.resize(TOT_DOF);  //to be sure about dimension
//  velocities = CONV::vector_cmat2std(velocities_cmat);

//  return velocities;



//}
