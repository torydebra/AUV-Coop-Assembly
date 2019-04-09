#ifndef FORMULAS_H
#define FORMULAS_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>
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

Eigen::VectorXd saturateVector(Eigen::VectorXd vector, double threshold);

}

#endif // FORMULAS_H
