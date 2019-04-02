#include "header/logger.h"

Logger::Logger()
{

  path_string = "~/logPeg/" + PRT::GetCurrentDateFormatted();
  boost::filesystem::path path(path_string);
  if(boost::filesystem::create_directories(path)){
    std::cout << "[LOGGER] folder created : " << path << std::endl;
  }

}

int Logger::createTaskDirectory(std::string taskName){
  this->taskName = taskName;
  std::string path_string2 = path_string + "/" + taskName;
  boost::filesystem::path path(path_string2);
  boost::filesystem::create_directories(path);
  return 0;
}

int Logger::writeActivation(CMAT::Matrix A){

  //activations << std::chrono::system_clock::now().time_since_epoch() << std::endl;

  activations.open(path_string+ "/" + taskName + "activations.txt", std::ios_base::app);
  for (int j=1; j<=A.GetNumColumns(); ++j){

    for (int i = 1; i<=A.GetNumRows(); ++i){
      activations << A(i,j) << " ";
    }
    activations << std::endl;
  }
  activations << std::endl;

  activations.close();

}

int Logger::writeReference(CMAT::Matrix ref){

  //auto now = std::chrono::system_clock::now();
  //auto now_c = std::chrono::system_clock::to_time_t(now);

  //references << now.time_since_epoch() << std::endl;

  activations.open(path_string+ "/" + taskName + "references.txt", std::ios_base::app);

  for (int j=1; j<=ref.GetNumColumns(); ++j){

    for (int i = 1; i<=ref.GetNumRows(); ++i){
      references << ref(i,j) << " ";
    }
    references << std::endl;
  }
  references << std::endl;

  references.close();

}

//int Logger::WriteErrors(CMAT::Matrix errors){

//  activations << std::chrono::system_clock::now() << std::endl;

//  for (int j=0; j<A.GetNumColumns(); ++j){

//    for (int i = 0; i<A.GetNumRows(); ++i){
//      activations << A(i,j) << " ";
//    }
//    activations << std::endl;
//  }
//  activations << std::endl;

//}





