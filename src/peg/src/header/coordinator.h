#ifndef COORDINATOR_H
#define COORDINATOR_H

#include <ros/ros.h>
#include <Eigen/Core>
#include <cmat/cmat.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include "../rosInterfaces/header/worldInterface.h"
#include "../rosInterfaces/header/coordInterfaceCoord.h"

#include "../support/header/defines.h"
#include "../support/header/formulas.h"

#include "../helper/header/logger.h"

int main(int argc, char **argv);
Eigen::Matrix<double, VEHICLE_DOF, 1> execCoordAlgo(Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelA_eigen,
                                                    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolA_eigen,
                                                    Eigen::Matrix<double, VEHICLE_DOF, 1> nonCoopCartVelB_eigen,
                                                    Eigen::Matrix<double, VEHICLE_DOF, VEHICLE_DOF> admisVelToolB_eigen,
                                                    Eigen::Matrix<double, VEHICLE_DOF, 1> refTool, Logger *logger = NULL);

Eigen::Matrix<double, VEHICLE_DOF, 1> calculateRefTool(Eigen::Matrix4d wTgoaltool_eigen,
                                                       Eigen::Matrix4d wTtool_eigen);


Eigen::Matrix4d changeGoal(double changeMagnitude, Eigen::Vector3d forcePegTip,
                           Eigen::Matrix4d wTgoalTool_eigen,
                           Eigen::Matrix4d wTt_eigen);


#endif // COORDINATOR_H
