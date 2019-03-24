#include "header/task.h"

/**
 * @brief Task::Task Constructor called by sons Constructor
 * @param dimension dimension of the task (e.g. 1 for scalar task)
 * @param dof degrees of freedom of the robot (4+6 for girona500 with 4DOF arm)
 * @param eqType true or false for equality or inequality task
 * @note There is no default Constructor (without argument) because at least dimension is necessary.
 */
Task::Task(int dimension, int dof, bool eqType)
{

  this->dimension = dimension;
  this->dof = dof;
  this->eqType = eqType;

  J = CMAT::Matrix::Zeros(dimension, dof);
  A = CMAT::Matrix::Zeros(dimension, dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

  threshold = THRESHOLD_DEFAULT;
  lambda = LAMBDA_DEFAULT;

  //for reg pseudoinverse of cmat
  flag_W = 0;
  mu_W = 0.0;
  flag_G = 0;
  mu_G = 0.0;

}

/**
 * @brief Task::Task Constructor called by sons Constructor. without argument dof, default dof from define.h
 * file is used.
 * @param dimension dimension of the task (e.g. 1 for scalar task)
 * @param eqType true or false for equality or inequality task
 * @note There is no default Constructor (without argument) because at least dimension is necessary.
 */
Task::Task(int dimension, bool eqType)
{

  this->dimension = dimension;
  this->dof = TOT_DOF;

  J = CMAT::Matrix::Zeros(dimension, dof);
  A = CMAT::Matrix::Zeros(dimension, dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

  threshold = THRESHOLD_DEFAULT;
  lambda = LAMBDA_DEFAULT;

  //for reg pseudoinverse of cmat
  flag_W = 0;
  mu_W = 0.0;
  flag_G = 0;
  mu_G = 0.0;

}

Task::~Task(){}

CMAT::Matrix Task::getJacobian(){ return this->J;}
CMAT::Matrix Task::getActivation(){return this->A;}
CMAT::Matrix Task::getReference(){return this->reference;}

/**
 * @brief Task::getFlag_W Various getter methods
 * @return
 * @note why &: value passed to cmat function regPseudoinv must be lvalue because the signature is:
 * Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
 * these gets are used only to pass values to that cmat function
 */
int& Task::getFlag_W(){return this->flag_W;}
/**
 * @brief Task::getMu_W Various getter methods
 * @return
 * @note why &: value passed to cmat function regPseudoinv must be lvalue because the signature is:
 * Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
 * these gets are used only to pass values to that cmat function
 */
double& Task::getMu_W(){return this->mu_W;}
/**
 * @brief Task::getFlag_G Various getter methods
 * @return
 * @note why &: value passed to cmat function regPseudoinv must be lvalue because the signature is:
 * Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
 * these gets are used only to pass values to that cmat function
 */
int& Task::getFlag_G(){return this->flag_G;}
/**
 * @brief Task::getMu_G Various getter methods
 * @return
 * @note why &: value passed to cmat function regPseudoinv must be lvalue because the signature is:
 * Matrix& Matrix::RegPseudoInverse(double threshold, double lambda, double& mu, int& flag) const
 * these gets are used only to pass values to that cmat function
 */
double& Task::getMu_G(){return this->mu_G;}

int Task::getThreshold(){return this->threshold;}
int Task::getLambda(){return this->lambda;}
int Task::getDof(){return this->dof;}
int Task::getDimension(){return this->dimension;}
