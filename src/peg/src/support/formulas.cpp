#include "header/formulas.h"

Eigen::Matrix3d FRM::skewMat(Eigen::Vector3d vect){
  Eigen::Matrix3d skew;

  skew <<    0,   -vect(2),  vect(1),
          vect(2),     0,    -vect(0),
          -vect(1),  vect(0),    0;

  return skew;

}
/**
 * @brief reducedVersorLemma, taken from uwsim matlab exercise
 * @param a first vector
 * @param b second vector
 * @return misalignement vector
 */
Eigen::Vector3d FRM::reducedVersorLemma(Eigen::Vector3d a, Eigen::Vector3d b){

  Eigen::Vector3d misalign;

  Eigen::Vector3d vsinth = a.cross(b);
  double costh = a.dot(b);

  double sinth = vsinth.norm();

  if (sinth > 0.00000000001){
    double theta = atan2(sinth, costh);
    misalign = (vsinth * (theta/sinth));

  } else {
    misalign << 0, 0, 0;
  }

  return misalign;


}

Eigen::MatrixXd FRM::pseudoInverse(Eigen::MatrixXd mat, double tolerance){

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::MatrixXd S_inv(mat.cols(), mat.rows());
  Eigen::MatrixXd singularValues = svd.singularValues();
  S_inv = Eigen::MatrixXd::Zero(mat.cols(), mat.rows());
  for (int i=0; i< singularValues.size(); ++i){
    if (singularValues(i) > tolerance) {
      S_inv(i, i) = 1 / singularValues(i);

    } else {
      S_inv(i, i) = 0;
    }
  }
  return (svd.matrixV() * S_inv * svd.matrixU().transpose());


}

