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
  KDLHelper(std::string filename, std::string link0_name, std::string endEffector_name, std::string vehicle_name);
  ~KDLHelper();
  int setEESolvers();
  int setToolSolvers(Eigen::Matrix4d eeTtool_eigen);
  int getFixedFrame(std::string frameOrigin, std::string frameTarget, Eigen::Matrix4d *xTx_eigen);
  int getEEpose(std::vector<double> jointPos, Eigen::Matrix4d *eePose_eigen);
  int getJacobianEE(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobianEE_eigen);
  int getJacobianTool(std::vector<double> jointPos, Eigen::Matrix<double, 6, ARM_DOF> *jacobianTool_eigen);
  int getNJoints();

  ///debug
  int getToolpose(std::vector<double> jointPos, Eigen::Matrix4d eeTtool_eigen, Eigen::Matrix4d *toolPose_eigen);
  ///old
  int setToolSolvers_old(Eigen::Matrix4d eeTtool_eigen);

private:
  int nJoints;
  std::string link0_name;
  std::string vehicle_name;
  std::string endEffector_name;
  KDL::Tree tree;
  KDL::ChainJntToJacSolver* jacobEE_solver;
  KDL::ChainJntToJacSolver* jacobTool_solver;
  //TODO capire diff tra recursive e non
  KDL::ChainFkSolverPos_recursive* eePose_solver;

};

#endif // KDLHelper_H
