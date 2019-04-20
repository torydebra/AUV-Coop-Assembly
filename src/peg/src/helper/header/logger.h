#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <vector>
#include <cmat/cmat.h>
#include <Eigen/Core>
#include "../../support/header/prints.h"
#include "../../support/header/conversions.h"
#include "../../tasks/header/task.h"

//stringify prepreocessor operator
#define GET_VARIABLE_NAME(variable) (#variable)


class Logger
{
public:
  Logger();
  Logger(std::string robotName, std::string pathLog);
  void createDirectoryForNode();
  int createTasksListDirectories(std::vector<Task*>);
  int writeActivation(std::vector<Task*>);
  int writeReference(std::vector<Task*>);
  int writeError(std::vector<Task*>);
  int writeAllForTasks(std::vector <Task*> tasksList);
  void writeYDot(std::vector<double> yDot, std::string yDotString);
  void writeNonCoopVel(Eigen::VectorXd nonCoopVel, std::string);
  void writeCoopVel(Eigen::VectorXd coopVel);
  void writeScalar(double scalar, std::string fileName);
  void writeEigenMatrix(Eigen::MatrixXd mat, std::string fileName);
  void writeStressTool(Eigen::Matrix4d wTt, Eigen::Matrix4d wTt2);






private:
  std::string nodeName;
  std::string pathLog;



};


#endif // LOGGER_H
