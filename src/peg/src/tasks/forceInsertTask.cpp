#include "header/forceInsertTask.h"

/**
 * @brief ForceInsertTask::ForceInsertTask
 * @param dim
 * @param eqType
 * @param robotName
 */
ForceInsertTask::ForceInsertTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "FORCE_INSERTION"){
  switch (dimension){
  case 1:
    gain = 0.09;
    break;
  case 2:
    gain = 0.01;
    gainAng = 0.01;
    break;
  case 3:
    gain = 0.001;
    break;
  case 6:
    gain = 0.001;
    gainAng = 0.001;
    break;
  }

  integralFor << 0,0,0;

}

int ForceInsertTask::updateMatrices(Infos* const robInfo){

  Eigen::Vector3d force = robInfo->robotSensor.forcePegTip;
  Eigen::Vector3d torque = robInfo->robotSensor.torquePegTip;
  force(0) *= 0.3; //forces on x are always too high, this help considering not so much the forces on x
  Eigen::Vector3d w_force = robInfo->transforms.wTt_eigen.topLeftCorner<3,3>() * force;
  Eigen::Vector3d w_torque = robInfo->transforms.wTt_eigen.topLeftCorner<3,3>() * torque;

  setActivation(w_force, w_torque);
  setJacobian(robInfo->robotState.w_Jtool_robot, w_force, w_torque);
  setReference(w_force, w_torque);

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

    double negValueForActMax = -1;
    double posValueForActMax = 2;
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

      //with norms decreasing not necessary because they are always positive
      A(1,1) = CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax ,
                                                   0, 1, forceNorm);

      A(2,2) = CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax ,
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
      for (int i=1; i<=3; i++){
        A(i,i) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                    0, 1, force(i-1)) +
                 CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax,
                                                     0, 1, force(i-1));
      }

      for (int i=4; i<=6; i++){
        A(i,i) = CMAT::DecreasingBellShapedFunction(negValueForActMax, negValueForAct,
                                                    0, 1, torque(i-4)) +
                 CMAT::IncreasingBellShapedFunction(posValueForAct, posValueForActMax,
                                                     0, 1, torque(i-4));
      }
      break;
    }
    } /// END SWITCH
  } /// END IF
}

void ForceInsertTask::setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot,
                                  Eigen::Vector3d force, Eigen::Vector3d torque){

  switch (dimension){

  case 1:{

    Eigen::Vector3d normalForce;
    double normForce = force.norm();
    if (normForce > 0.001) {
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
    //w_J_robot.rightCols<6>() = Eigen::MatrixXd::Zero(6, 6);
    jacobTemp.topRows<1>() = - (normalForce.transpose()) * (w_J_robot.topRows<3>());

    /// torques
    Eigen::Vector3d normalTorque;
    double normTorque = torque.norm();
    if (normTorque > 0) {
      normalTorque = torque/normTorque;
    } else {
      normalTorque << 0,0,0;
    }

    jacobTemp.bottomRows<1>() = - (normalTorque.transpose()) * (w_J_robot.bottomRows<3>());

    /// finally convert it to cmat
    J = CONV::matrix_eigen2cmat(jacobTemp);

    break;
  }

  case 3:{
    J = CONV::matrix_eigen2cmat(w_J_robot.topRows<3>());
    break;
  }
  case 6: {
    J = CONV::matrix_eigen2cmat(w_J_robot);
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
    error(1) = - force.norm(); //for old logs, not used anymore?
    //integralFor = (force * 0.1) + integralFor;
    reference(1) = gain*(error(1)); // - 0.005 * (integralFor.norm());
    reference(1) = FRM::saturateScalar(reference(1), 0.04);


    error(2) = - torque.norm();
    reference(2) = gainAng * error(2);
    reference(2) = FRM::saturateScalar(reference(2), 0.04);

    break;
  }

  case 3:{
    error = -1 * CONV::matrix_eigen2cmat(force); //-1 because it is 0 - force
    reference = gain * error;
    reference = FRM::saturateCmat(reference, 0.1);

    break;
  }

  case 6: {

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
    reference(4) = gainAng * ( - torque_cmat(1));
    reference(5) = gainAng * ( - torque_cmat(2));
    reference(6) = gainAng * ( - torque_cmat(3));
    reference = FRM::saturateCmat(reference, 0.001);
    break;
  }
  default:
     std::cerr << "[" << taskName << "] DIMENSION OF TASK IS WRONG\n";
  }


}


