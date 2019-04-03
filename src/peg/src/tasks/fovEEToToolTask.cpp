#include "header/fovEEToToolTask.h"

FovEEToToolTask::FovEEToToolTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "FOV_EE_TO_TOOL") {

  if (dimension != 1){
    std::cerr << "[" << robotName << "]" << taskName << "] ERROR, this task is intended to be a scalar one, "<<
                 "you setted "<< dimension << " as dimension\n";
    return;
  }
  gain = 0.4;
}


int FovEEToToolTask::updateMatrices(struct Infos* const robInfo){

  Eigen::Vector3d kHat;
  kHat << 0, 0, 1; //generic k versor

  Eigen::Matrix4d wTee = robInfo->robotState.wTv_eigen * robInfo->robotState.vTee_eigen;
  //a from book
  w_kHat_ee = wTee.topLeftCorner<3,3>() * kHat;

  w__Dist_ee_t = robInfo->transforms.wTt_eigen.topRightCorner<3,1>() -
                                 wTee.topRightCorner<3,1>();

  //Problem here to find a_d norm must be not zero
  if (w__Dist_ee_t.norm() < 0.000001){
    return -1;
  }
  //a_d from book, that is the previous vector normalized
  a_d = w__Dist_ee_t / w__Dist_ee_t.norm();

  distNorm = (a_d - w_kHat_ee).norm();
  // not continue execution of task, it is useless because distance
  // is already ok. Also we cant divide by zero to find a_d
  if (distNorm < 0.000001){
    return 1;
  }

  setReference();
  setActivation();
  setJacobian(robInfo->robotState.w_J_robot);

  return 0;
}



/**
 * @brief HorizontalAttitudeTask::setActivation Active only when field of view is too far
 * away (like the horizontal attitude, when misalignement is too much)
 *
 */
void FovEEToToolTask::setActivation(){

  // activation we know that is a scalar because dimension == 1
  A(1) = CMAT::IncreasingBellShapedFunction(0.015, 0.6, 0, 1, distNorm);
}

void FovEEToToolTask::setReference(){
  //We know reference is a scalar becase task is scalar
  reference(1) = - gain * distNorm;

}

void FovEEToToolTask::setJacobian(
    Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){

  Eigen::Vector3d normalVectDist = (a_d - w_kHat_ee) / distNorm;

  Eigen::Matrix3d skew_ad = FRM::skewMat(a_d);
  Eigen::Matrix3d skew_Dist_ee_t__inv = FRM::pseudoInverse(FRM::skewMat(w__Dist_ee_t));
  Eigen::Matrix3d skew_kHat_ee = FRM::skewMat(w_kHat_ee);

  Eigen::Matrix<double, 1, TOT_DOF> J_eigen;
  J_eigen = normalVectDist.transpose() * (
                -skew_ad * skew_Dist_ee_t__inv * w_J_robot.topRows<3>() +
                     skew_kHat_ee * w_J_robot.bottomRows<3>()
            );

  J = CONV::matrix_eigen2cmat(J_eigen);

}



/// ACTUALLY, from antonelli book the misalignm is not used, it used the norm of the distance
/// to be brought to zero
/**
 * /**
 * @brief FovEEToToolTask::setPhi calculate misalignement
 * between khat of the end effector and vector joinint end effector and tool
 * @param wTv_eigen Transform from world to vehicle
 * @param wTt_eigen Transform from world to tool
 * @param vTee_eigen Transform from vehicle to end effector
 */
//void FovEEToToolTask::setPhi(Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTt_eigen,
//                            Eigen::Matrix4d vTee_eigen){

//  Eigen::Vector3d kHat;
//  kHat << 0, 0, 1; //generic k versor

//  Eigen::Matrix3d wTee = wTv_eigen * vTee_eigen;
//  Eigen::Vector3d w_kHat_ee = wTee.topLeftCorner<3,3>() * kHat; //versor k of ee projected on world
//  //distance from ee to tool projected on world
//  Eigen::Vector3d w__Dist_ee_t = wTt_eigen.topRightCorner<3,1>() - wTee.topRightCorner<3,1>();

//  phi = FRM::reducedVersorLemma(w__Dist_ee_t, w_kHat_ee);

//}
