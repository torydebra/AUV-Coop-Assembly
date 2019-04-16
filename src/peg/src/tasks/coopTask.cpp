#include "header/coopTask.h"

CoopTask::CoopTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "COOP_CONSTRAINT"){

  //no gain, it is a non reactive task;

}

int CoopTask::updateMatrices(Infos * const robInfo){

  setActivation();
  setJacobian(robInfo->robotState.w_Jtool_robot);
  setReference(robInfo->exchangedInfo.coopCartVel);
}

void CoopTask::setReference(Eigen::Matrix<double, VEHICLE_DOF, 1> coopCartVel){

  if (dimension == 6){
    reference = CONV::matrix_eigen2cmat(coopCartVel);
  } else if (dimension ==5){
    auto temp = CONV::matrix_eigen2cmat(coopCartVel);
    reference(1)= temp(1);
    reference(2)= temp(2);
    reference(3)= temp(3);
    reference(4)= temp(5);
    reference(5)= temp(6);
  }

}

void CoopTask::setActivation(){

  double vectDiag[dimension];
  std::fill_n(vectDiag, dimension, 1);
  this->A.SetDiag(vectDiag);


}

void CoopTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_Jtool_robot){

  if (dimension ==6){
    //w_Jtool_robot.row(4) << 0, 0,0,0,0,0,0,0,0,0; //TRY ASK DEBUG

    J = CONV::matrix_eigen2cmat(w_Jtool_robot);
  } else if (dimension == 5){
    auto temp = CONV::matrix_eigen2cmat(w_Jtool_robot);
    J = temp.DeleteRow(4);
  }
}
