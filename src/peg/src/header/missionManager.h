#ifndef MISSIONMANAGER_H
#define MISSIONMANAGER_H

#include <iostream>
#include <string>

#include <Eigen/Core>
#include <ros/ros.h>
//#include <boost/chrono.hpp>
//#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../support/header/conversions.h"
#include "../support/header/defines.h"
#include "../support/header/prints.h"

#include "../helper/header/infos.h"
#include "../helper/header/jacobianHelper.h"
#include "../helper/header/kdlHelper.h"


#include "../controller/header/controller.h"
#include "../rosInterface/header/rosInterface.h"


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

int main(int, char**);
void setTaskListInit(std::vector<Task*> *tasks, std::string robotName);

#endif // MISSIONMANAGER_H