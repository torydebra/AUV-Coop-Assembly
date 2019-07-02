#include "header/forceInsertTask.h"

/**
 * @brief ForceInsertTask::ForceInsertTask
 * @param dim
 * @param eqType
 * @param robotName
 * @note with dimension 1 (force norm type) the gain must be higher?
 */
ForceInsertTask::ForceInsertTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "FORCE_INSERTION"){
  switch (dimension){
  case 1:
    gain = 0.07;
    break;
  case 2:
    gain = 0.035;
    break;
  }

}

int ForceInsertTask::updateMatrices(Infos* const robInfo){

  setActivation(robInfo->robotSensor.forcePegTip, robInfo->robotSensor.torquePegTip);
  setJacobian(robInfo->robotState.w_Jtool_robot,
              robInfo->robotSensor.forcePegTip, robInfo->robotSensor.torquePegTip);
  setReference(robInfo->robotSensor.forcePegTip, robInfo->robotSensor.torquePegTip);
  return 0;

}

/**
 * @brief ForceInsertTask::setActivation
 * @param force
 * @param torque
 * @todo activation needed otherwise vehicle dont move
 */
void ForceInsertTask::setActivation(Eigen::Vector3d force, Eigen::Vector3d torque){
  if (eqType){
    double vectDiag[dimension];
    std::fill_n(vectDiag, dimension, 1);
    this->A.SetDiag(vectDiag);

  } else {

    double negValueForActMax = -1; //which means that when force is > |1| activation is 1
    double posValueForActMax = 1;
    double negValueForAct = -0.001;
    double posValueForAct = 0.001;

    switch (dimension){
    case 1:{
      double forceNorm = force.norm();
      A(1,1) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                  0, 1, forceNorm) +
               CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax ,
                                                   0, 1, forceNorm);
      break;
    }
    case 2:{
      double forceNorm = force.norm();
      double torqueNorm = torque.norm();

      A(1,1) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                  0, 1, forceNorm) +
               CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax ,
                                                   0, 1, forceNorm);

      A(2,2) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                  0, 1, torqueNorm) +
               CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax ,
                                                   0, 1, torqueNorm);

      break;

    }
    case 3: {
      for (int i=1; i<=dimension; i++){
        A(i,i) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                    0, 1, force(i-1)) +
                 CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax,
                                                     0, 1, force(i-1));
      }
      break;
    }
    case 6: {
      //not implemented
      break;
    }
    }
  }
}

void ForceInsertTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot,
                                  Eigen::Vector3d force, Eigen::Vector3d torque){

  switch (dimension){

  case 1:{

    Eigen::Vector3d normalForce;
    double normForce = force.norm();
    if (normForce > 0) {
      normalForce = force/normForce;
    } else {
      normalForce << 0,0,0;
    }

    //not consider vehicle part. Only arm moves to nullify forces
    w_J_robot.topRows<3>().topRightCorner<3,6>() = Eigen::MatrixXd::Zero(3, 6);
    J = CONV::matrix_eigen2cmat(
          (normalForce.transpose()) * (w_J_robot.topRows<3>())); //toprows: only linear part


    break;
  }

  case 2:{

    Eigen::MatrixXd jacobTemp = Eigen::MatrixXd::Zero(dimension, dof);

    /// forces
    Eigen::Vector3d normalForce;
    double normForce = force.norm();
    if (normForce > 0) {
      normalForce = force/normForce;
    } else {
      normalForce << 0,0,0;
    }
    //not consider vehicle part. Only arm moves to nullify forces
    //w_J_robot.topRows<3>().topRightCorner<3,6>() = Eigen::MatrixXd::Zero(3, 6);
    jacobTemp.topRows<1>() = (normalForce.transpose()) * (w_J_robot.topRows<3>());


    /// torques
    Eigen::Vector3d normalTorque;
    double normTorque = torque.norm();
    if (normTorque > 0) {
      normalTorque = torque/normTorque;
    } else {
      normalTorque << 0,0,0;
    }

    //not consider vehicle part. Only arm moves to nullify torques
    //w_J_robot.bottomRows<3>().topRightCorner<3,6>() = Eigen::MatrixXd::Zero(3, 6);
    jacobTemp.bottomRows<1>() = (normalTorque.transpose()) * (w_J_robot.bottomRows<3>());

    /// finally convert it to cmat
    J = CONV::matrix_eigen2cmat(jacobTemp);

    break;
  }

  case 3:{
    J = CONV::matrix_eigen2cmat(w_J_robot.topRows<3>());
    break;
  }
  case 6: {
    //not implemented
    break;
  }

  } // END SWITCH

}


void ForceInsertTask::setReference(Eigen::Vector3d force, Eigen::Vector3d torque){

  switch(dimension){
  case 1:{
    error(1) = 0 - force.norm();
    reference = gain * error;
    reference = FRM::saturateCmat(reference, 0.05);

    break;
  }

  case 2:{
    error(1) = 0 - force.norm();
    reference(1) = gain * error(1);
    reference(1) = FRM::saturateScalar(reference(1), 0.002);


    error(2) = 0 - torque.norm();
    reference(2) = gain * error(2);
    reference(2) = FRM::saturateScalar(reference(2), 0.002);

    break;
  }

  case 3:{
    error = -1 * CONV::matrix_eigen2cmat(force); //-1 because it is 0 - force
    reference = gain * error;
    reference = FRM::saturateCmat(reference, 0.1);

    break;
  }

  case 6: {
    /// jacob and act not implemented so I dont know why I put here this

    CMAT::Vect3 force_cmat = CONV::matrix_eigen2cmat(force);
    CMAT::Vect3 torque_cmat = CONV::matrix_eigen2cmat(torque);
    error(1) = ( - force_cmat(1));
    error(2) = ( - force_cmat(2));
    error(3) = ( - force_cmat(3));
    error(4) = ( - torque_cmat(1));
    error(5) = ( - torque_cmat(2));
    error(6) = ( - torque_cmat(3));
    reference(1) = gain * ( - force_cmat(1));
    reference(2) = gain * ( - force_cmat(2));
    reference(3) = gain * ( - force_cmat(3));
    reference(4) = gain * ( - torque_cmat(1));
    reference(5) = gain * ( - torque_cmat(2));
    reference(6) = gain * ( - torque_cmat(3));
    reference = FRM::saturateCmat(reference, 0.5);
    break;
  }
  default:
     std::cerr << "[" << taskName << "] DIMENSION OF TASK IS WRONG\n";
  }


}


