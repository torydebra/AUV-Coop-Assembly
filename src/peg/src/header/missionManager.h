#ifndef MISSIONMANAGER_H
#define MISSIONMANAGER_H

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
//#include <boost/chrono.hpp>
//#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../support/header/conversions.h"
#include "../support/header/defines.h"
#include "../support/header/prints.h"
#include "../support/header/formulas.h"

#include "../helper/header/infos.h"
#include "../helper/header/jacobianHelper.h"
#include "../helper/header/kdlHelper.h"
#include "../helper/header/logger.h"

#include "../controller/header/controller.h"
#include "../controller/header/collisionPropagator.h"
#include "../controller/header/graspConstrainer.h"

#include "../rosInterfaces/header/robotInterface.h"
#include "../rosInterfaces/header/worldInterface.h"
#include "../rosInterfaces/header/coordInterfaceMissMan.h"
#include "../rosInterfaces/header/visionInterfaceMissMan.h"

#include "../tasks/header/lastTask.h"
#include "../tasks/header/vehicleReachTask.h"
#include "../tasks/header/endEffectorReachTask.h"
#include "../tasks/header/jointLimitTask.h"
#include "../tasks/header/horizontalAttitudeTask.h"
#include "../tasks/header/fovEEToToolTask.h"
#include "../tasks/header/vehicleNullVelTask.h"
#include "../tasks/header/vehicleConstrainVelTask.h"
#include "../tasks/header/obstacleAvoidVehicleTask.h"
#include "../tasks/header/obstacleAvoidEETask.h"
#include "../tasks/header/pipeReachTask.h"
#include "../tasks/header/armShapeTask.h"
#include "../tasks/header/coopTask.h"
#include "../tasks/header/forceInsertTask.h"



int main(int, char**);
void setTaskLists(std::string robotName, std::vector<Task*> *tasks);


void setTaskLists(std::string robotName, std::vector<Task*>* ,
                  std::vector<Task*>*, std::vector<Task*>*);

void deleteTasks(std::vector<Task*> *tasks);

//void setTaskLists(std::string robotName, std::vector<Task*> *tasks1,
//                  std::vector<Task*> *tasksFinal);

#endif // MISSIONMANAGER_H
