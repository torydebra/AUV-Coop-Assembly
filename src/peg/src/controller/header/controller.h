#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmat/cmat.h>

#include "../../support/header/defines.h"
#include "../../support/header/transforms.h"

#include "../../tasks/header/vehicleReachTask.h"

/** @brief The Controller class is responsabile of taking matrices and giving them to Tasks classes,
 * so they can compute their activations, references, jacobians, etc matrices
 * Its main scope is to execute the icat algorithm.

*/
class Controller {

public:

  // the list of task. Each *tasks point to a different class (that is the specific task)
  //Task** tasks;
  std::vector<Task*> tasks;
  int numTasks;

  Controller();
  ~Controller();


  int updateTransforms(struct Transforms* const transf);
  std::vector<double> execAlgorithm();

private:
  int equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q);
  int inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) ;
};


#endif // CONTROLLER_H
