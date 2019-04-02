#ifndef FOVEETOTOOLTASK_H
#define FOVEETOTOOLTASK_H

#include "task.h"
#include "../../support/header/formulas.h"
#include "../../support/header/conversions.h"


class FovEEToToolTask : public Task
{
public:
  FovEEToToolTask(int dimension, bool eqType);
  int updateMatrices(struct Infos* const robInfo);
private:
  void setActivation();
  void setJacobian(Eigen::Matrix<double, 6, TOT_DOF> w_J_robot);
  void setReference();

  Eigen::Vector3d w_kHat_ee;
  Eigen::Vector3d a_d;
  Eigen::Vector3d w__Dist_ee_t;
  //a_d is the normal vector: w__Dist_ee_t/norm(w__Dist_ee_t)
  double distNorm; //norm of distance a-a_d


//  int setPhi(Eigen::Matrix4d wTv_eigen, Eigen::Matrix4d wTt_eigen,
//             Eigen::Matrix4d vTee_eigen);
//  Eigen::Vector3d phi;

};



#endif // FOVEETOTOOLTASK_H
