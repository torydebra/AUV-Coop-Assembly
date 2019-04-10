#include "header/prints.h"

int PRT::printRotMatrix_tf(tf::Transform transform){

  std::cout <<"\n\n"<< transform.getBasis().getColumn(0).getX() << "\t"
            << transform.getBasis().getColumn(1).getX() << "\t"
            << transform.getBasis().getColumn(2).getX() << "\n"
            << transform.getBasis().getColumn(0).getY() << "\t"
            << transform.getBasis().getColumn(1).getY() << "\t"
            << transform.getBasis().getColumn(2).getY() << "\n"
            << transform.getBasis().getColumn(0).getZ() << "\t"
            << transform.getBasis().getColumn(1).getZ() << "\t"
            << transform.getBasis().getColumn(2).getZ() << "\n\n";
  return 0;

}

int PRT::printMatrix3x3_tf(tf::Matrix3x3 matrix){

  std::cout <<"\n\n"<< matrix.getColumn(0).getX() << "\t"
            << matrix.getColumn(1).getX() << "\t"
            << matrix.getColumn(2).getX() << "\n"
            << matrix.getColumn(0).getY() << "\t"
            << matrix.getColumn(1).getY() << "\t"
            << matrix.getColumn(2).getY() << "\n"
            << matrix.getColumn(0).getZ() << "\t"
            << matrix.getColumn(1).getZ() << "\t"
            << matrix.getColumn(2).getZ() << "\n\n";
  return 0;

}

/// LOG things
int PRT::createDirectory(std::string pathDirectory) {

  boost::filesystem::path path(pathDirectory);
  if(boost::filesystem::create_directories(path)){
    std::cout << "[PRT] folder created : " << path << "\n";
  }
}

int PRT::matrixCmat2file(std::string pathName, CMAT::Matrix mat){
  std::ofstream file;
  file.open(pathName, std::ios_base::app);

  for (int j=1; j<=mat.GetNumRows(); ++j){

    for (int i = 1; i<=mat.GetNumColumns(); ++i){
      file << mat(j,i) << " ";
    }
    file << std::endl;
  }
  file << std::endl;

  file.close();
}

int PRT::vectorStd2file(std::string pathqDot, std::vector<double> qDot){

  std::ofstream file;
  file.open(pathqDot, std::ios_base::app);

  for(int i=0; i<qDot.size(); i++){
    file << qDot.at(i) << " ";
  }
  file << std::endl;


  file.close();


}


std::string PRT::getCurrentDateFormatted() {
    std::time_t t = std::time(NULL);
    char mbstr[20];
    std::strftime(mbstr, sizeof(mbstr), "%Y-%m-%d_%H-%M-%S", std::localtime(&t));
    std::string currentDate(mbstr);

    return currentDate;
}
