#include "header/prints.h"

int PRINT::printRotMatrix_tf(tf::Transform transform){

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

int PRINT::printMatrix3x3_tf(tf::Matrix3x3 matrix){

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
