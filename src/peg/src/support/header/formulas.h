#ifndef FORMULAS_H
#define FORMULAS_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
#include <cmat/cmat.h>
#include <vector>

namespace FRM {

Eigen::Matrix3d skewMat(Eigen::Vector3d vect);
Eigen::Vector3d reducedVersorLemma(Eigen::Vector3d a, Eigen::Vector3d b);

/**
 * @brief inverseTransf: useful to invert trasformation matrices taking into account that
 * they are transformation matrix (so inverse is easy). Using directrly inverse() method of Eigen
 * is less efficent becasue eigen cant know that we are inverting a transf matrix
 * @param mat eigen to invert
 * @return the matrix inverted
 */
Eigen::Matrix4d invertTransf(Eigen::Matrix4d mat);
Eigen::MatrixXd pseudoInverse(Eigen::MatrixXd mat, double tolerance = 0.0001);

Eigen::VectorXd saturateVectorEigen(Eigen::VectorXd vector, double threshold);
std::vector<double> saturateVectorStd(std::vector<double> vector, double threshold);
CMAT::Matrix saturateCmat(CMAT::Matrix mat, double threshold);
double saturateScalar(double scalar, double threshold);

Eigen::Matrix3d eul2Rot(std::vector<double> eulRad);
}
#endif // FORMULAS_H
