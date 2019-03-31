#ifndef KDLHelper_H
#define KDLHelper_H

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <urdf/model.h>
#include <Eigen/Core>

#include "../../support/header/conversions.h"
#include "../../support/header/defines.h"


class KDLHelper
{
public:
  KDLHelper(std::string filename);
  ~KDLHelper();
  int setSolvers(std::string link0, std::string endEffector);
  int getFixedFrame(std::string frameOrigin, std::string frameTarget, Eigen::Matrix4d *xTx_eigen);
  int getEEpose(std::vector<double> jointPos, Eigen::Matrix4d *eePose_eigen);
  int getJacobian(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobian);
  int getNJoints();

private:
  int nJoints;
  KDL::Tree tree;
  KDL::ChainJntToJacSolver* jacob_solver;
  //TODO capire diff tra recursive e non
  KDL::ChainFkSolverPos_recursive* eePose_solver;
};

#endif // KDLHelper_H
