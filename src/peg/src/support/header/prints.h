#ifndef PRINTS_H
#define PRINTS_H


#include <tf/tf.h>
#include <cmat/cmat.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/filesystem.hpp>



namespace PRT{
  int printRotMatrix_tf(tf::Transform transform);
  int printMatrix3x3_tf(tf::Matrix3x3 matrix);

  // for logging things
  int createDirectory(std::string pathDirectory);
  int matrixCmat2file(std::string pathName, CMAT::Matrix);
  std::string getCurrentDateFormatted();


}


#endif // PRINT_H

