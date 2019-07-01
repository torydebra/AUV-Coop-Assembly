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

Eigen::Matrix4d FRM::invertTransf(Eigen::Matrix4d mat){

  Eigen::Matrix4d inv = Eigen::Matrix4d::Identity();

  // inverse of rot part is the transpose
  Eigen::Matrix3d rot_t = mat.topLeftCorner<3,3>().transpose();
  inv.topLeftCorner<3,3>() = rot_t;

  inv.topRightCorner<3,1>() = - rot_t * mat.topRightCorner<3,1>();

  return inv;


}

Eigen::MatrixXd FRM::pseudoInverse(Eigen::MatrixXd mat, double tolerance){

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
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

/**
 * @brief FRM::saturateVector
 * Look all component of the input vector;
 *   if the absoule value of one (or more) component is above the threshold,
 *     scale all components considering the biggest (in abs value) component.
 *  otherwise, return vector unchanged
 * @param vector to saturate
 * @param threshold Maximum absoulte value that we want for each component
 * @return the saturated vector
 */
Eigen::VectorXd FRM::saturateVectorEigen(Eigen::VectorXd vector, double threshold){

  if (threshold <0.0){
    std::cerr << "FRM::saturateVectorEigen: ERROR: the threshold must be positive";
    return vector;
  }
  Eigen::VectorXd out = vector;

  //take abs value
  for (int i =0; i< vector.rows(); i++){
    if (vector(i) < 0.0){
      vector(i) *= -1;
    }
  }

  double maxCoeff = vector.maxCoeff();
  if ( maxCoeff > threshold){ //if so, scale vector

    out = out/maxCoeff*threshold;

  }

  return out;
}

std::vector<double> FRM::saturateVectorStd(std::vector<double> vector, double threshold){

  if (threshold <0.0){
    std::cerr << "FRM::saturateVectorStd: ERROR: the threshold must be positive";
    return vector;
  }
  std::vector<double> out = vector;

  //take abs value
  for (int i =0; i< vector.size(); i++){
    if (vector.at(i) < 0.0){
      vector.at(i) *= -1;
    }
  }

  double maxCoeff = *(std::max_element(vector.begin(), vector.end()));
  if ( maxCoeff > threshold){ //if so, scale vector

    for (int i=0; i<out.size(); i++){
      out.at(i) = (out.at(i))/maxCoeff*threshold;

    }

  }

  return out;
}


CMAT::Matrix FRM::saturateCmat(CMAT::Matrix mat, double threshold){
  if (threshold <0.0){
    std::cerr << "FRM::saturateCmat: ERROR: the threshold must be positive";
    return mat;
  }

  CMAT::Matrix out = mat;

  //take abs value & get max
  double maxCoeff = 0.0;
  for (int i=1; i<= mat.GetNumColumns(); i++){
    for (int j=1; j<= mat.GetNumRows(); j++){
      if (mat(j,i) < 0.0){
        mat(j,i) *= -1;
      }
      if (mat(j,i) > maxCoeff){
        maxCoeff = mat(j,i);
      }
    }
  }

  if ( maxCoeff > threshold){ //if so, scale vector
    out = out/maxCoeff*threshold;
  }

  return out;
}

double FRM::saturateScalar(double scalar, double threshold){

  if (threshold <0.0){
    std::cerr << "FRM::saturateScalar: ERROR: the threshold must be positive";
    return scalar;
  }

  if (scalar < 0){

    if (scalar < (-threshold) ) {
      scalar = -threshold;
    }

  } else {
    if (scalar > threshold){
      scalar = threshold;
    }
  }

  return scalar;

}

Eigen::Matrix3d FRM::eul2Rot(std::vector<double> eulRad){

  Eigen::Matrix3d R_x, R_y, R_z;

  R_x << 1,                        0,                       0,
         0,                 cos(eulRad.at(0)),       -sin(eulRad.at(0)),
         0,                 sin(eulRad.at(0)),        cos(eulRad.at(0));

  R_y << cos(eulRad.at(1)),        0,                 sin(eulRad.at(1)),
          0,                       1,                          0,
        -sin(eulRad.at(1)),        0,                 cos(eulRad.at(1));

  R_z << cos(eulRad.at(2)),    -sin(eulRad.at(2)),         0,
         sin(eulRad.at(2)),    cos(eulRad.at(2)),          0,
           0,                           0,                 1;


  return (R_x * R_y * R_z);




}


