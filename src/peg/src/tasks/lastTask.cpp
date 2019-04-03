#include "header/lastTask.h"

LastTask::LastTask(int dim, bool eqType, std::string robotName)
  : Task(dim, eqType, robotName, "LAST_TASK"){
  gain = 1;

  J = CMAT::Matrix::Eye(dimension); //note jacobian is square
  A = CMAT::Matrix::Eye(dimension);
  reference = CMAT::Matrix::Zeros(dimension,1);

}

//to not use, implemented only to make this class non virtual (father is virtual)
int LastTask::updateMatrices(struct Infos* const robInfo){

  return 0;
}
