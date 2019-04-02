#ifndef LOGGER_H
#define LOGGER_H

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <cmat/cmat.h>
#include <chrono>
#include "../../support/header/prints.h"


class Logger
{
public:
  Logger();
  int createTaskDirectory(std::string taskname);
  int writeActivation(CMAT::Matrix A);
  int writeReference(CMAT::Matrix ref);

private:
  std::string path_string;
  std::string taskName;
  std::ofstream activations;
  std::ofstream references;
  std::ofstream errors;

};


#endif // LOGGER_H
