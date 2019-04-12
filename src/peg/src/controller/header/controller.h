#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmat/cmat.h>

#include "../../support/header/defines.h"
#include "../../support/header/prints.h"
#include "../../helper/header/infos.h"

#include "../../tasks/header/task.h"


/** @brief The Controller class is responsabile of
 * 1) taking matrices and giving them to Tasks classes,
 *    so they can compute their activations, references, jacobians, etc matrices
 * 2) After all tasks have setted matrices, it executes the icat algorithm and return the
 *    yDot to the caller.
*/
class Controller {

public:

  // the list of task. Each *tasks point to a different class (that is the specific task)
  std::vector<Task*> tasks;
  int numTasks;

  Controller(std::string robotName);
  int setTaskList(std::vector<Task*> tasks);

  int updateTransforms(struct Infos* const robInfo);
  std::vector<double> execAlgorithm();

private:
  std::string robotName; //to differentiate robots in log folders and printsconsole
  int equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q);
  int inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) ;
};


#endif // CONTROLLER_H
