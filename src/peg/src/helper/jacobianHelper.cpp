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
  Eigen::Matrix3d wRv = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>();

  //J_man_xxx = wR0 * zeroJ_man_xxx
  w_J_man_pos = wRv * robInfo->robotState.v_Jtool_man.topRows<3>();
  w_J_man_or = wRv * robInfo->robotState.v_Jtool_man.bottomRows<3>();

  /// Compute jacobian for tool position (J_pos from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_pos;

  // part relative to joints
  w_J_pos.leftCols<ARM_DOF>() = w_J_man_pos;

  // part relative to linear velocities
  w_J_pos.block<3,3>(0,ARM_DOF) = wRv; //with block indexes start from 0

  // part relative to angular velocities
  //distance from vehicle to TOOL respect to world (wRv * v_eta_v,ee)
  Eigen::Vector3d w_DistVTool =
      robInfo->transforms.wTt_eigen.topRightCorner<3,1>() -
      robInfo->robotState.wTv_eigen.topRightCorner<3,1>();


  //skew__XXX : skew of XXX
  Eigen::Matrix3d skew__w_Dist_vtool = FRM::skewMat(w_DistVTool);
  w_J_pos.rightCols<3>() = -(skew__w_Dist_vtool) * wRv;


  /// Compute jacobian for orientation (J_or from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_or;
  w_J_or.leftCols<ARM_DOF>() = w_J_man_or; //relative to arm
  w_J_or.block<3,3>(0,ARM_DOF) = Eigen::Matrix3d::Zero(); //relative to linear vel
  w_J_or.rightCols<3>() = wRv; //relative to angular vel

  /// Store in struct

  robInfo->robotState.w_Jtool_robot.topRows<3>() = w_J_pos;
  robInfo->robotState.w_Jtool_robot.bottomRows<3>() = w_J_or;



  //DEBUG
//  std::cout << "kdl first Skew\n" << skew__w_Dist_v0 << "\n\n";
//  std::cout << "kdl secodn Skew\n" << skew__w_Dist_0tool << "\n\n";
//  std::cout << "eta1\n"
//            << robInfo->robotState.wTv_eigen.topRightCorner<3,1>()
//                  << "\n\n";
//  std::cout << "eta2\n"
//            << robInfo->robotState.wTv_eigen.topLeftCorner<3,3>()
//            << "\n\n";
//  std::cout << "eta_ee1\n"
//            << robInfo->transforms.wTt_eigen.topRightCorner<3,1>()
//            << "\n\n";

  return 0;
}





//int computeJacobianToolNoKdl(struct Infos *robInfo, std::string robotName){

//    Eigen::MatrixXd J_eigen = Eigen::MatrixXd::Zero(6, TOT_DOF);

//    Eigen::Vector3d w_rtool = robInfo->transforms.wTt_eigen.topRightCorner(3,1); //position of tool respect world
//    Eigen::Vector3d khat; //versor along z axis
//    khat << 0, 0, 1;

//    /// taking rotation matrices and translational vectors
//    Eigen::Matrix3d w_R_[ARM_DOF];
//    Eigen::Vector3d w_r[ARM_DOF];

//    for (int i =0; i<ARM_DOF; i++){
//      w_R_[i] = robInfo->robotState.wTjoints[i].topLeftCorner(3,3);
//      w_r[i] = robInfo->robotState.wTjoints[i].topRightCorner(3,1); //position of joint i respect world
//      //std::cout <<"\n\n JOINT "<< i << ": \n" << w_R_[i] << "\n\n";
//    }

//    ///JOINT part (0 to 3 column)
//    for (int i = 0; i<ARM_DOF; i++){

//      Eigen::VectorXd g(6);
//      g << 0,0,0, 0,0,0;

//      Eigen::Vector3d w_kHat = w_R_[i] * khat; //versor k respect vehicle frame

//      g.head(3) = w_kHat.cross(w_rtool - w_r[i]); //linear part
//  //    std::cout <<"\n\n JOINT "<< i << ": \n" << v_kHat << " x (" << v_ree << " - " << v_r[i] << ")" << "\n"
//  //             "result: " << g.head(3)<<"\n\n";


//      g.tail(3) = w_kHat; //angular part

//      J_eigen.col(i) = g;
//    }


//    ///VEHICLE part (4 to 9 columns)

//    //linear part (top rows)
//    J_eigen.block<3,3>(0,4) = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>(); //linear contribution to linear velocities

//    Eigen::Matrix3d firstSkew = FRM::skewMat(robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() * robInfo->robotStruct.vTlink0.topRightCorner<3,1>());
//    Eigen::Vector3d ee_distEeTool;
//    if (robotName.compare("g500_A") == 0){
//      ee_distEeTool << -0.000, 2.000, -0.017;
//    }else{
//      ee_distEeTool <<-0.000, -2.000, 0.007;

//    }
//    Eigen::Vector3d link0_DistLink0Tool = robInfo->robotState.link0Tee_eigen.topRightCorner<3,1>() +
//        (robInfo->robotState.link0Tee_eigen.topLeftCorner<3,3>()*ee_distEeTool);
//    Eigen::Matrix3d secondSkew =
//        FRM::skewMat(robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() *
//                     robInfo->robotStruct.vTlink0.topLeftCorner<3,3>() *
//                     link0_DistLink0Tool);

////    std::cout << "nokdl first Skew\n" << firstSkew << "\n\n";
////    std::cout << "nokdl secodn Skew\n" << secondSkew << "\n\n";
////    std::cout << "nokdl asdasd\n distlink0EE:"<<robInfo->robotState.link0Tee_eigen.topRightCorner<3,1>()
////        << "\n distEETOOl: " << ee_distEeTool << "\n eta: " << eta  << "\n\n";
//    J_eigen.topRightCorner<3,3>() = - (firstSkew  * robInfo->robotState.wTv_eigen.topLeftCorner<3,3>())
//        - (secondSkew * robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() *
//           robInfo->robotStruct.vTlink0.topLeftCorner<3,3>());

//    //J_eigen.topRightCorner(3,3) = -(FRM::skewMat(w_rtool)); //angular contribution to linear velocites
//    //youbot version??
//    CMAT::Vect3 link0_DistLink0Tool_cmat;
//    link0_DistLink0Tool_cmat(1) = link0_DistLink0Tool(0);
//    link0_DistLink0Tool_cmat(2) = link0_DistLink0Tool(1);
//    link0_DistLink0Tool_cmat(3) = link0_DistLink0Tool(2);
//    std::cout<< "OOOOO\n\n";
//    Eigen::MatrixXd rigBodySwapped = CONV::matrix_cmat2eigen(link0_DistLink0Tool_cmat.GetRigidBodyMatrix());
//    Eigen::MatrixXd rigBody(6,6);
//    rigBody.topLeftCorner<3,3>() = (robInfo->robotState.wTv_eigen * robInfo->robotStruct.vTlink0).topLeftCorner<3,3>() *
//        rigBodySwapped.bottomLeftCorner<3,3>();
//    rigBody.topRightCorner<3,3>() = (robInfo->robotState.wTv_eigen * robInfo->robotStruct.vTlink0).topLeftCorner<3,3>() *
//        rigBodySwapped.bottomRightCorner<3,3>();
//    rigBody.bottomLeftCorner<3,3>() = (robInfo->robotState.wTv_eigen * robInfo->robotStruct.vTlink0).topLeftCorner<3,3>() *
//        rigBodySwapped.topLeftCorner<3,3>();
//    rigBody.bottomRightCorner<3,3>() = (robInfo->robotState.wTv_eigen * robInfo->robotStruct.vTlink0).topLeftCorner<3,3>() *
//        rigBodySwapped.topRightCorner<3,3>();

//    J_eigen.rightCols(6) = rigBody;
//   // J_eigen.rightCols(6) = (link0_DistLink0Tool_cmat.GetRigidBodyMatrix()); //angular contribution to linear velocites

//    //angualr part (bottom rows)
//    J_eigen.block<3,3>(3,4) = Eigen::Matrix3d::Zero(); //linear contribution to angular velocities
//    J_eigen.bottomRightCorner(3,3) = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>(); //angular contribution to angular velocities

//    robInfo->robotState.w_JNoKdltool_robot = J_eigen;

//    return 0;

//}



/**
 * @brief computeWholeJacobianTool
 * @param robInfo (in & out) info to calculate jacobian and store it (passed by reference)
 * @return 0 correct execution
 */
/*
int computeWholeJacobianTool_old(struct Infos *robInfo){


  //positional and orientational part of arm jacobian PROJECTED ON WORLD
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_pos;
  Eigen::Matrix <double, 3, ARM_DOF> w_J_man_or;
  Eigen::Matrix3d wR0 = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>() *
      robInfo->robotStruct.vTlink0.topLeftCorner<3,3>();

  //J_man_xxx = wR0 * zeroJ_man_xxx
  w_J_man_pos = wR0 * robInfo->robotState.link0_Jtool_man.topRows<3>();
  w_J_man_or = wR0 * robInfo->robotState.link0_Jtool_man.bottomRows<3>();

  /// Compute jacobian for tool position (J_pos from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_pos;

  // part relative to joints
  w_J_pos.leftCols<ARM_DOF>() = w_J_man_pos;

  // part relative to linear velocities
  Eigen::Matrix3d wRv = robInfo->robotState.wTv_eigen.topLeftCorner<3,3>();
  w_J_pos.block<3,3>(0,ARM_DOF) = wRv; //with block indexes start from 0

  // part relative to angular velocities
  //distance from vehicle to link 0 respect on vehicle frame
  Eigen::Vector3d v_Distv0 = robInfo->robotStruct.vTlink0.topRightCorner<3,1>();
  Eigen::Vector3d w_Distv0 = wRv * v_Distv0;
  //distance from link0 to TOOL respect to world (wR0 * 0_eta_0,ee in book)
  Eigen::Vector3d w_Dist0Tool =
      robInfo->transforms.wTt_eigen.topRightCorner<3,1>() -
      robInfo->robotState.wTv_eigen.topRightCorner<3,1>() -
      w_Distv0;
  Eigen::Vector3d w_DistVTool =
      robInfo->transforms.wTt_eigen.topRightCorner<3,1>() -
      robInfo->robotState.wTv_eigen.topRightCorner<3,1>();


  //skew__XXX : skew of XXX
  Eigen::Matrix3d skew__w_Dist_v0 = FRM::skewMat(w_Distv0);
  Eigen::Matrix3d skew__w_Dist_0tool = FRM::skewMat(w_Dist0Tool);
  Eigen::Matrix3d skew__w_Dist_Vtool = FRM::skewMat(w_DistVTool);
  //w_J_pos.rightCols<3>() = -(skew__w_Dist_v0 + skew__w_Dist_0tool) * wRv;
  w_J_pos.rightCols<3>() = -(skew__w_Dist_Vtool) * wRv;


  /// Compute jacobian for orientation (J_or from Antonelli book)
  Eigen::Matrix<double, 3, TOT_DOF> w_J_or;
  w_J_or.leftCols<ARM_DOF>() = w_J_man_or; //relative to arm
  w_J_or.block<3,3>(0,ARM_DOF) = Eigen::Matrix3d::Zero(); //relative to linear vel
  w_J_or.rightCols<3>() = wRv; //relative to angular vel

  /// Store in struct

  robInfo->robotState.w_Jtool_robot.topRows<3>() = w_J_pos;
  robInfo->robotState.w_Jtool_robot.bottomRows<3>() = w_J_or;



  //DEBUG
//  std::cout << "kdl first Skew\n" << skew__w_Dist_v0 << "\n\n";
//  std::cout << "kdl secodn Skew\n" << skew__w_Dist_0tool << "\n\n";
//  std::cout << "eta1\n"
//            << robInfo->robotState.wTv_eigen.topRightCorner<3,1>()
//                  << "\n\n";
//  std::cout << "eta2\n"
//            << robInfo->robotState.wTv_eigen.topLeftCorner<3,3>()
//            << "\n\n";
//  std::cout << "eta_ee1\n"
//            << robInfo->transforms.wTt_eigen.topRightCorner<3,1>()
//            << "\n\n";


//  std::cout << "J: \n"
//            << robInfo->robotState.w_Jtool_robot
//            << "\n\n";


  return 0;
}
*/
