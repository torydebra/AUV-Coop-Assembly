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

int Logger::logAllForTasks(std::vector <Task*> tasksList){

  Logger::logTasksActivation(tasksList);
  Logger::logTasksReference(tasksList);
  Logger::logTasksError(tasksList);

  return 0;
}

int Logger::logTasksActivation(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();
    PRT::matrixCmat2file(pathname + "/activation.txt",
                         tasksList[i]->getActivation().GetDiag());
  }

  return 0;

}

int Logger::logTasksReference(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();

    PRT::matrixCmat2file(pathname + "/reference.txt",
                         tasksList[i]->getReference());
  }
  return 0;

}

int Logger::logTasksError(std::vector <Task*> tasksList){

  for(int i=0; i<tasksList.size(); i++){
    std::string pathname = pathLog + "/" + tasksList[i]->getName();

    PRT::matrixCmat2file(pathname + "/error.txt",
                         tasksList[i]->getError());
  }
  return 0;

}

void Logger::logNumbers(std::vector<double> matrix, std::string fileName){
  std::string path = pathLog + "/" + fileName +".txt";
  PRT::vectorStd2file(path, matrix);
}

void Logger::logNumbers(Eigen::MatrixXd matrix, std::string fileName){
  std::string path = pathLog + "/" + fileName +".txt";
  PRT::matrixEigen2file(path, matrix);
}

void Logger::logNumbers(double scalar, std::string fileName){
  std::string path = pathLog + "/" + fileName + ".txt";
  PRT::double2file(path, scalar);
}

void Logger::logNumbers(CMAT::Matrix matrix, std::string fileName){
  Logger::logNumbers(CONV::matrix_cmat2eigen(matrix), fileName);
}


/**
 * @brief Logger::logCartError log cartesian error for two frames
 *    The result is the error that brings in2 towards in1
 * @param wTt
 * @param wTt2
 * @note the two transformation matrix should have a common base frame,
 *      i.e. logCartError(wTg, wTt) brings the tool frame <t>
 *      towards a goal frame <g>, and returns the error projected on frame <w>
 */
void Logger::logCartError(Eigen::Matrix4d goal, Eigen::Matrix4d base,
                          std::string fileName){

  Logger::logCartError(CONV::matrix_eigen2cmat(goal),
               CONV::matrix_eigen2cmat(base), fileName);


}

void Logger::logCartError(CMAT::TransfMatrix goal, CMAT::TransfMatrix base,
                          std::string fileName){

  CMAT::Vect6 stressErrorSwapped = CMAT::CartError(goal, base);

  std::vector<double> stressError(6);
  stressError.at(0) = stressErrorSwapped(4);
  stressError.at(1) = stressErrorSwapped(5);
  stressError.at(2) = stressErrorSwapped(6);
  stressError.at(3) = stressErrorSwapped(1);
  stressError.at(4) = stressErrorSwapped(2);
  stressError.at(5) = stressErrorSwapped(3);

  std::string path = pathLog + "/" + fileName + ".txt";
  PRT::vectorStd2file(path, stressError);

}






