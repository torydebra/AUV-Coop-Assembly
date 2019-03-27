#include "header/lastTask.h"

LastTask::LastTask(int dim, int dof, bool eqType)
  : Task(dim, dof, eqType){
  gain = 1;

  J = CMAT::Matrix::Eye(dimension); //note jacobian is square
  A = CMAT::Matrix::Eye(dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

}

//to not use, implemented only to make this class non virtual (father is virtual)
int LastTask::updateMatrices(struct Transforms* const transf){

  return 0;
}
