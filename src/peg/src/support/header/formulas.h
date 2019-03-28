#ifndef FORMULAS_H
#define FORMULAS_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <math.h>

namespace FRM {

Eigen::Matrix3d skewMat(Eigen::Vector3d vect);
Eigen::Vector3d reducedVersorLemma(Eigen::Vector3d a, Eigen::Vector3d b);

}

#endif // FORMULAS_H
