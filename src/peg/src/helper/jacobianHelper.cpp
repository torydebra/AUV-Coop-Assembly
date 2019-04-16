#include "header/jacobianHelper.h"


//TODO: store internally matrix and vector that remain fixed? (eg posiz of link0 respect to vehicle)

/**
 * @brief computeWholeJacobianEE
 * @param robInfo (in & out) info to calculate jacobian and store it (passed by reference)
 * @return 0 correct execution
 */
int computeWholeJacobianEE(struct Infos *robInfo){


  //positional and orientational part of arm jacobian PROJECTED ON WORLD
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_pos;
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_or;
  Eigen::Matrix3d wR0 = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() *
      robInfo->robotStruct.vTlink0.topLeftCorner<3,3>();

  //J_man_xxx = wR0 * zeroJ_man_xxx
  w_J_man_pos = wR0 * robInfo->robotState.link0_Jee_man.topRows<3>();
  w_J_man_or = wR0 * robInfo->robotState.link0_Jee_man.bottomRows<3>();

  /// Compute jacobian for position (J_pos from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_pos;

  // part relative to joints
  w_J_pos.leftCols<ARM_DOF>() = w_J_man_pos;

  // part relative to linear velocities
  Eigen::Matrix3d wRv = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>();
  w_J_pos.block<3,3>(0,ARM_DOF) = wRv; //with block indexes start from 0

  // part relative to angular velocities
  Eigen::Matrix3d vRlink0 = robInfo->robotStruct.vTlink0.topLeftCorner<3,3>();
  //distance from vehicle to link 0 respect on vehicle frame
  Eigen::Vector3d v_Distv0 = robInfo->robotStruct.vTlink0.topRightCorner<3,1>();
  //distance from link0 to end effector respect to link0
  Eigen::Vector3d link0_link0DistEe = robInfo->robotState.link0Tee_eigen.topRightCorner<3,1>();
  Eigen::Matrix3d wRlink0 = wRv*vRlink0;
  //skew__XXX : skew of XXX
  Eigen::Matrix3d skew__w_Dist_v0 = FRM::skewMat(wRv * v_Distv0);
  Eigen::Matrix3d skew__w_Dist_0ee = FRM::skewMat(wRlink0 * link0_link0DistEe);
  w_J_pos.rightCols<3>() = -(skew__w_Dist_v0 + skew__w_Dist_0ee) * wRv;


  /// Compute jacobian for orientation (J_or from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_or;
  w_J_or.leftCols<ARM_DOF>() = w_J_man_or; //relative to arm
  w_J_or.block<3,3>(0,ARM_DOF) = Eigen::Matrix3d::Zero(); //relative to linear vel
  w_J_or.rightCols<3>() = wRv; //relative to angular vel

  /// Store in struct
  robInfo->robotState.w_Jee_robot.topRows<3>() = w_J_pos;
  robInfo->robotState.w_Jee_robot.bottomRows<3>() = w_J_or;


  return 0;
}


/**
 * @brief computeWholeJacobianTool
 * @param robInfo (in & out) info to calculate jacobian and store it (passed by reference)
 * @return 0 correct execution
 */
int computeWholeJacobianTool(struct Infos *robInfo){


  //positional and orientational part of arm jacobian PROJECTED ON WORLD
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_pos;
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_or;
  Eigen::Matrix4d wT0 = robInfo->robotState.wTv_eigen *
      robInfo->robotStruct.vTlink0;

  Eigen::Matrix3d wR0 = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() *
      robInfo->robotStruct.vTlink0.topLeftCorner<3,3>();

  //J_man_xxx = wR0 * zeroJ_man_xxx
  w_J_man_pos = wR0 * robInfo->robotState.link0_Jtool_man.topRows<3>();
  w_J_man_or = wR0 * robInfo->robotState.link0_Jtool_man.bottomRows<3>();

  /// Compute jacobian for position (J_pos from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_pos;

  // part relative to joints
  w_J_pos.leftCols<ARM_DOF>() = w_J_man_pos;

  // part relative to linear velocities
  Eigen::Matrix3d wRv = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>();
  w_J_pos.block<3,3>(0,ARM_DOF) = wRv; //with block indexes start from 0

  // part relative to angular velocities
  Eigen::Matrix3d vRlink0 = robInfo->robotStruct.vTlink0.topLeftCorner<3,3>();
  //distance from vehicle to link 0 respect on vehicle frame
  Eigen::Vector3d v_Distv0 = robInfo->robotStruct.vTlink0.topRightCorner<3,1>();
  //distance from link0 to TOOL respect to world (wR0 * 0_eta_0,ee in book)
  Eigen::Vector3d w_link0DistTool =
      robInfo->transforms.wTt_eigen.topRightCorner<3,1>() -
      wT0.topRightCorner<3,1>();

  //skew__XXX : skew of XXX
  Eigen::Matrix3d skew__w_Dist_v0 = FRM::skewMat(wRv * v_Distv0);
  Eigen::Matrix3d skew__w_Dist_0tool = FRM::skewMat(w_link0DistTool);
  w_J_pos.rightCols<3>() = -(skew__w_Dist_v0 + skew__w_Dist_0tool) * wRv;


  /// Compute jacobian for orientation (J_or from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_or;
  w_J_or.leftCols<ARM_DOF>() = w_J_man_or; //relative to arm
  w_J_or.block<3,3>(0,ARM_DOF) = Eigen::Matrix3d::Zero(); //relative to linear vel
  w_J_or.rightCols<3>() = wRv; //relative to angular vel

  /// Store in struct

  robInfo->robotState.w_Jtool_robot.topRows<3>() = w_J_pos;
  robInfo->robotState.w_Jtool_robot.bottomRows<3>() = w_J_or;

  return 0;
}
