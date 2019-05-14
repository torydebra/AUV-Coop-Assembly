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
  int logTasksActivation(std::vector<Task*>);
  int logTasksReference(std::vector<Task*>);
  int logTasksError(std::vector<Task*>);
  int logAllForTasks(std::vector <Task*> tasksList);

  //log matrices, veector, scalar...
  void logNumbers(std::vector<double> matrix, std::string fileName);
  void logNumbers(Eigen::MatrixXd matrix, std::string fileName);
  void logNumbers(double scalar, std::string fileName);
  void logNumbers(CMAT::Matrix matrix, std::string fileName);

  void logCartError(Eigen::Matrix4d goal, Eigen::Matrix4d base, std::string fileName);
  void logCartError(CMAT::TransfMatrix goal, CMAT::TransfMatrix base, std::string fileName);





private:
  std::string nodeName;
  std::string pathLog;



};


#endif // LOGGER_H
