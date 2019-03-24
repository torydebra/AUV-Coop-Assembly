#ifndef TASK_H
#define TASK_H

#include <cmat/cmat.h>
#include <Eigen/Core>
#include "../../support/header/transforms.h"
#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"


/** @brief ABSTRACT class task. Each task is a derived class of this class. It contains all the variable that the
 * task must have, methods to get these protected variables, and a pure virtual function that the derived class must
 * implement.
*/
class Task
{
private:


protected:
  int dimension; //dimension of the task (1 for scalar task); (rows)
  int dof; //total dofs of the robot (columns)
  CMAT::Matrix J; // the Jacobian
  CMAT::Matrix A; //the Activation function
  CMAT::Matrix reference;

  int flag_W;
  double mu_W;
  int flag_G;
  double mu_G;

  int threshold;
  int lambda;

  Task(int dimension, int dof, bool eqType);
  Task(int dimension, bool eqType);

public:

  virtual ~Task();

  virtual int updateMatrices(struct Transforms* const transf) = 0;

  CMAT::Matrix getJacobian();
  CMAT::Matrix getActivation();
  CMAT::Matrix getReference();

  //note why &:
  // value passed to cmar regPseudoinv that must be lvalue because:
  //Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
  // these gets are used only to pass values to that cmat function
  int& getFlag_W();
  double& getMu_W();
  int& getFlag_G();
  double& getMu_G();

  int getThreshold();
  int getLambda();
  int getDof();
  int getDimension();

  bool eqType;

};

#endif // TASK_H
