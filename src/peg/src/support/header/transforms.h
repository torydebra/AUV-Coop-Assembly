#ifndef TRANSFORMS_H
#define TRANSFORMS_H

#include <Eigen/Core>

struct Transforms {

  // for vehicleReachTask
  Eigen::Matrix4d wTgoal_eigen;
  Eigen::Matrix4d wTv_eigen;

};

#endif
