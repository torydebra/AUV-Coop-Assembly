#ifndef TASK_H
#define TASK_H

#include <iostream>
#include <cmat/cmat.h>
#include <Eigen/Core>
#include "../../support/header/defines.h"
#include "../../helper/header/infos.h"


/** @brief ABSTRACT class task. Each task is a derived class of this class. It contains all the variable that the
 * task must have, methods to get these protected variables, and a pure virtual function that the derived class must
 * implement.
 *
 * @remark VERY IMPORTANT: yDot is [joints(1...n) x, y, z, wx, wy, wz] take care when doing JACOBIAN and REFERENCES
*/
class Task
{
private:


protected:
  int dimension; //dimension of the task (1 for scalar task); (rows)
  int dof; //total dofs of the robot (columns)
  std::string taskName;
  std::string robotName;
  CMAT::Matrix J; // the Jacobian
  CMAT::Matrix A; //the Activation function
  CMAT::Matrix reference;
  CMAT::Matrix error; //that is the reference without gain and saturation, useful for plotting result


  int flag_W;
  double mu_W;
  int flag_G;
  double mu_G;

  int threshold;
  int lambda;

  Task(int dimension, bool eqType,
       std::string robotName, std::string taskName);

public:

  bool updated; //to not call computeMatrices on task that were contained also in previous TPIKs list


  virtual ~Task();

  virtual int updateMatrices(struct Infos* const robInfo) = 0;

 // int writeLogs();
  std::string getName();
  CMAT::Matrix getJacobian();
  CMAT::Matrix getActivation();
  CMAT::Matrix getReference();
  CMAT::Matrix getError();

  //note why &:
  // value passed to cmar regPseudoinv that must be lvalue because:
  //Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
  // these gets are used only to pass values to that cmat function
  int& getFlag_W();
  double& getMu_W();
  int& getFlag_G();
  double& getMu_G();
  void setFlag_W(int);
  void setMu_W(double);
  void setFlag_G(int);
  void setMu_G(double);


  int getThreshold();
  int getLambda();
  int getDof();
  int getDimension();

  bool eqType;
  double gain;

};

#endif // TASK_H
