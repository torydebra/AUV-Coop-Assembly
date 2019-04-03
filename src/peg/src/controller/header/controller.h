#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmat/cmat.h>

#include "../../support/header/defines.h"
#include "../../support/header/prints.h"

#include "../../helper/header/infos.h"

#include "../../tasks/header/lastTask.h"
#include "../../tasks/header/vehicleReachTask.h"
#include "../../tasks/header/endEffectorReachTask.h"
#include "../../tasks/header/jointLimitTask.h"
#include "../../tasks/header/horizontalAttitudeTask.h"
#include "../../tasks/header/fovEEToToolTask.h"
#include "../../tasks/header/vehicleNullVelTask.h"
#include "../../tasks/header/vehicleConstrainVelTask.h"
#include "../../tasks/header/obstacleAvoidVehicleTask.h"
#include "../../tasks/header/obstacleAvoidEETask.h"




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

  Controller(std::string robotName, std::string* pathLog = NULL);
  ~Controller();


  int updateTransforms(struct Infos* const robInfo);
  std::vector<double> execAlgorithm();

private:
  std::string pathLog;
  std::string robotName; //to differentiate robots in log folders and printsconsole
  int equalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q);
  int inequalityIcat(Task* task, CMAT::Matrix* rhop, CMAT::Matrix* Q) ;
};


#endif // CONTROLLER_H
