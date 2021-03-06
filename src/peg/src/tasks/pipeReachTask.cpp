#include "header/pipeReachTask.h"

PipeReachTask::PipeReachTask(int dim, bool eqType, std::string robotName, VehArmType vehArmType, AngLinType angLinType)
  : Task(dim, eqType, robotName, "PIPE_REACHING_GOAL"){
  //gain = 0.05;
  gainLin = 0.05;
  gainAng = 0.08;
  this->vehArmType = vehArmType;
  this->angLinType = angLinType;

}


int PipeReachTask::updateMatrices(Infos * const robInfo){

  setActivation();
  setJacobian(robInfo->robotState.w_Jtool_robot);
  setReference(robInfo->transforms.wTgoalTool_eigen, robInfo->transforms.wTt_eigen);
}

int PipeReachTask::setActivation(){

  double vectDiag[dimension];
  std::fill_n(vectDiag, dimension, 1);
  this->A.SetDiag(vectDiag);

  return 0;

}

void PipeReachTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){

  switch (vehArmType){
  case ONLYVEH:
    w_J_robot.leftCols<ARM_DOF>() = Eigen::MatrixXd::Zero(6, ARM_DOF);
    break;

  case ONLYARM:
    w_J_robot.rightCols<VEHICLE_DOF>() = Eigen::MatrixXd::Zero(6, VEHICLE_DOF);
    break;

  case BOTH:
    break;
  }

  if (dimension == 6){
    J = CONV::matrix_eigen2cmat(w_J_robot);
  }
  else if (dimension == 5){
    CMAT::Matrix J_temp = CONV::matrix_eigen2cmat(w_J_robot);
    J = J_temp.DeleteRow(4);
  } else if (dimension == 3){
    switch (angLinType){
    case LIN:
    {
      J = CONV::matrix_eigen2cmat(w_J_robot.topRows<3>());
      break;
    }
    case ANG:
    {
      J = CONV::matrix_eigen2cmat(w_J_robot.bottomRows<3>());
      break;
    }
    default:
    {
      std::cerr << "ERROR: dimension 3 and not lin ang specified\n";
      exit(-1);
    }

    }

  }

}




void PipeReachTask::setReference(Eigen::Matrix4d wTgoaltool_eigen, Eigen::Matrix4d wTtool_eigen){

  CMAT::TransfMatrix wTgoaltool_cmat = CONV::matrix_eigen2cmat(wTgoaltool_eigen);
  CMAT::TransfMatrix wTtool_cmat = CONV::matrix_eigen2cmat(wTtool_eigen);



  CMAT::Vect6 errorSwapped = CMAT::CartError(wTgoaltool_cmat, wTtool_cmat);//ang;lin
  // ang and lin must be swapped because in yDot and jacob linear part is before

  if (dimension == 6){
    error(1)= errorSwapped(4);
    error(2)= errorSwapped(5);
    error(3)= errorSwapped(6);
    error(4)= errorSwapped(1);
    error(5)= errorSwapped(2);
    error(6)= errorSwapped(3);

    CMAT::Vect3 vect3_lin;
    CMAT::Vect3 vect3_ang;
    vect3_lin = (this->gainLin * errorSwapped.GetSecondVect3());
    vect3_ang= (this->gainAng * errorSwapped.GetFirstVect3());

    vect3_lin = FRM::saturateCmat(vect3_lin, 0.1); //0.1 before
    vect3_ang = FRM::saturateCmat(vect3_ang, 0.1);

    this->reference(1) = vect3_lin(1);
    this->reference(2) = vect3_lin(2);
    this->reference(3) = vect3_lin(3);
    this->reference(4) = vect3_ang(1);
    this->reference(5) = vect3_ang(2);
    this->reference(6) = vect3_ang(3);

  } else if (dimension == 5){


    CMAT::Vect3 vect3_lin;
    CMAT::Matrix vect2_ang(2,1);

    error(1)= errorSwapped(4);
    error(2)= errorSwapped(5);
    error(3)= errorSwapped(6);
    error(4)= errorSwapped(2);
    error(5)= errorSwapped(3);

    vect3_lin = (this->gainLin * errorSwapped.GetSecondVect3());
    vect2_ang(1) = this->gainAng * errorSwapped(2);
    vect2_ang(2) = this->gainAng * errorSwapped(3);

    vect3_lin = FRM::saturateCmat(vect3_lin, 0.1);
    vect2_ang = FRM::saturateCmat(vect2_ang, 0.1);

    this->reference(1) = vect3_lin(1);
    this->reference(2) = vect3_lin(2);
    this->reference(3) = vect3_lin(3);
    this->reference(4) = vect2_ang(1);
    this->reference(5) = vect2_ang(2);

  } else if (dimension == 3){
    switch (angLinType){
    case LIN:
    {
      error(1)= errorSwapped(4);
      error(2)= errorSwapped(5);
      error(3)= errorSwapped(6);

      CMAT::Vect3 vect3_lin;
      vect3_lin = (this->gainLin * errorSwapped.GetSecondVect3());

      vect3_lin = FRM::saturateCmat(vect3_lin, 0.1); //0.1 before

      this->reference(1) = vect3_lin(1);
      this->reference(2) = vect3_lin(2);
      this->reference(3) = vect3_lin(3);

      break;
    }
    case ANG:
    {
      error(1)= errorSwapped(1);
      error(2)= errorSwapped(2);
      error(3)= errorSwapped(3);

      CMAT::Vect3 vect3_ang;
      vect3_ang= (this->gainAng * errorSwapped.GetFirstVect3());

      vect3_ang = FRM::saturateCmat(vect3_ang, 0.1);

      this->reference(1) = vect3_ang(1);
      this->reference(2) = vect3_ang(2);
      this->reference(3) = vect3_ang(3);

      break;
    }
    default:
    {
      std::cerr << "ERROR: dimension 3 and not lin ang specified\n";
      exit(-1);
    }

    }
  }

}






/*** old  with cout
 *
void PipeReachTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot){

//  Eigen::Matrix<double, 5, TOT_DOF> totJ;
//  totJ.topRows<3>() = w_J_robot.topRows<3>();

//  Eigen::Matrix<double, 2, TOT_DOF> excludeRoll;
//  excludeRoll << 0, 1, 0,
//                0, 0, 1;


//  totJ.bottomRows<2>() = excludeRoll * w_J_robot.bottomRows<3>();
  if (dimension == 6){
    //debug
//    Eigen::Matrix<double, 6, TOT_DOF> temp = w_J_robot;
//    temp.leftCols<4>() << 0, 0, 0, 0,
//                          0,0,0,0,
//                          0,0,0,0,
//                          0,0,0,0,
//                          0,0,0,0,
//                          0,0,0,0,
    J = CONV::matrix_eigen2cmat(w_J_robot);
  }
  else if (dimension == 5){
    CMAT::Matrix J_temp = CONV::matrix_eigen2cmat(w_J_robot);
    J = J_temp.DeleteRow(4);
  }
}
*/
