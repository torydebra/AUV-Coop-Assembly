#include "header/logger.h"

Logger::Logger(){}

Logger::Logger(std::string nodeName, std::string pathLog){

  this->nodeName = nodeName;
  this->pathLog = pathLog += "/" + nodeName;
  std::cout << "[" << nodeName
            << "][Logger] Start  " << std::endl;
}

void Logger::createDirectoryForNode(){
  PRT::createDirectory(pathLog);
  std::cout << "[" << nodeName
            << "][Logger] Created Log Folder in  "
            << pathLog  << std::endl;
}

int Logger::createTasksListDirectories(std::vector <Task*> tasksList){

  for (int i =0; i< tasksList.size(); ++i){
    PRT::createDirectory(pathLog +"/" +tasksList[i]->getName());
  }
  std::cout << "[" << nodeName
            << "][Logger] Created Log SubFolders for tasks in  "
            << pathLog  << std::endl;
  return 0;

}

int Logger::writeAllForTasks(std::vector <Task*> tasksList){

  Logger::writeActivation(tasksList);
  Logger::writeReference(tasksList);
  Logger::writeError(tasksList);

  return 0;
}

int Logger::writeActivation(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();
    PRT::matrixCmat2file(pathname + "/activation.txt",
                         tasksList[i]->getActivation().GetDiag());
  }

  return 0;

}

int Logger::writeReference(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();

    PRT::matrixCmat2file(pathname + "/reference.txt",
                         tasksList[i]->getReference());
  }
  return 0;

}

int Logger::writeError(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();

    PRT::matrixCmat2file(pathname + "/error.txt",
                         tasksList[i]->getError());
  }
  return 0;

}

void Logger::writeYDot(std::vector<double> yDot, std::string yDotString){

  std::string pathyDot = pathLog + "/" +yDotString +".txt";
  PRT::vectorStd2file(pathyDot, yDot);

}

void Logger::writeNonCoopVel(Eigen::VectorXd nonCoopVel, std::string rob){
  std::string path = pathLog + "/nonCoopVel" + rob + ".txt";
  PRT::matrixEigen2file(path, nonCoopVel);

}

void Logger::writeCoopVel(Eigen::VectorXd coopVel){

  std::string path = pathLog + "/coopVel.txt";
  PRT::matrixEigen2file(path, coopVel);

}

void Logger::writeScalar(double scalar, std::string fileName){
  std::string path = pathLog + "/" + fileName + ".txt";
  PRT::double2file(path, scalar);

}

void Logger::writeEigenMatrix(Eigen::MatrixXd mat, std::string fileName){
  std::string path = pathLog + "/" + fileName + ".txt";
  PRT::matrixEigen2file(path, mat);
}





