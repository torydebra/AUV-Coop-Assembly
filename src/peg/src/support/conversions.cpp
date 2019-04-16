#include "header/conversions.h"

std::vector<double> CONV::tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3){

  std::vector<double> vector(9);
  vector[0] = matrix3x3.getColumn(0).getX();
  vector[1] = matrix3x3.getColumn(0).getY();
  vector[2] = matrix3x3.getColumn(0).getZ();
  vector[3] = matrix3x3.getColumn(1).getX();
  vector[4] = matrix3x3.getColumn(1).getY();
  vector[5] = matrix3x3.getColumn(1).getZ();
  vector[6] = matrix3x3.getColumn(2).getX();
  vector[7] = matrix3x3.getColumn(2).getY();
  vector[8] = matrix3x3.getColumn(2).getZ();

  return vector;

}

Eigen::VectorXd CONV::vector_std2Eigen(std::vector<double> vect){

  Eigen::VectorXd vec_eig(vect.size());

  for (int i=0; i<vect.size(); i++){
    vec_eig(i) = vect.at(i);
  }

  return vec_eig;
}

std::vector<double> CONV::vector_cmat2std(CMAT::Matrix mat){

  std::vector<double> vect;
  if (mat.GetNumColumns() != 1){
    std::cout << "[CONV] vector_cmat2std ERROR: argument must be a nx1 matrix (ie, a vector)";
    return vect;
  }
  vect.resize(mat.GetNumRows());

  int i = 1;
  for(std::vector<double>::iterator it = vect.begin(); it != vect.end(); ++it) {
    *it = mat(i);
    i++;
  }

  return vect;
}

Eigen::Matrix3d CONV::rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat){

  Eigen::Matrix3d mat_eigen;
  mat_eigen << mat_cmat(1,1), mat_cmat(1,2), mat_cmat(1,3),
               mat_cmat(2,1), mat_cmat(2,2), mat_cmat(2,3),
               mat_cmat(3,1), mat_cmat(3,2), mat_cmat(3,3);

  return mat_eigen;
}


Eigen::Matrix4d CONV::transfMatrix_cmat2eigen(CMAT::TransfMatrix mat_cmat){

  Eigen::Matrix4d mat_eigen = Eigen::Matrix4d::Identity();
  mat_eigen.topLeftCorner<3,3>() = CONV::rotMatrix_cmat2eigen(mat_cmat.GetRotMatrix());
  CMAT::Vect3 trasl = mat_cmat.GetTrasl();
  mat_eigen(0,3) = trasl(1);
  mat_eigen(1,3) = trasl(2);
  mat_eigen(2,3) = trasl(3);


  return mat_eigen;
}

//CMAT::TransfMatrix CONV::transfMatrix_tf2cmat(tf::Transform mat_tf){

//  std::vector<double> vector = CONV::tfMat3x3_to_vector(mat_tf.getBasis());
//  CMAT::RotMatrix rot_cmat(&vector[0]);
//  double* lin = mat_tf.getOrigin();
//  CMAT::Vect3 orient_cmat(lin);

//  CMAT::TransfMatrix mat_cmat(rot_cmat, orient_cmat);

//  return mat_cmat;
//}

CMAT::Matrix CONV::matrix_eigen2cmat(Eigen::MatrixXd mat_eigen){
  CMAT::Matrix mat_cmat(mat_eigen.rows(), mat_eigen.cols(), mat_eigen.data());
  return mat_cmat;
}


Eigen::Matrix4d CONV::transfMatrix_tf2eigen(tf::Transform mat_tf){
  Eigen::Matrix4d mat_eigen = Eigen::Matrix4d::Identity();

  //rotational part
  Eigen::Matrix3d rot_eigen;
  tf::matrixTFToEigen(mat_tf.getBasis(), rot_eigen);
  mat_eigen.topLeftCorner<3,3>() = rot_eigen;

  //trans part
  mat_eigen(0,3) = mat_tf.getOrigin().getX();
  mat_eigen(1,3) = mat_tf.getOrigin().getY();
  mat_eigen(2,3) = mat_tf.getOrigin().getZ();


  return mat_eigen;
}

//tf::Transform CONV::transfMatrix_eigen2tf(Eigen::Matrix4d mat_eigen){
//  tf::Transform mat_tf;
//  tf::transformEigenToTF(mat_eigen, mat_tf);
//  return mat_tf;
//}

/**
 * @brief CONV::transfMatrix_kdl2eigen
 * @param mat_kdl KDL matrix to convert
 * @return mat_eigen matrix converted
 * @warning WARNING: KDL is row major. Eigen and cmat column major
 */
Eigen::Matrix4d CONV::transfMatrix_kdl2eigen(KDL::Frame mat_kdl){

  Eigen::Matrix4d mat_eigen = Eigen::Matrix4d::Identity();

  // with << operand we have to indicate elements row by row. kdl store matrix rowMajor so
  // we can do this
  mat_eigen.topLeftCorner<3,3>() << mat_kdl.M.data[0], mat_kdl.M.data[1], mat_kdl.M.data[2],
                                  mat_kdl.M.data[3], mat_kdl.M.data[4], mat_kdl.M.data[5],
                                  mat_kdl.M.data[6], mat_kdl.M.data[7], mat_kdl.M.data[8];

  mat_eigen(0,3) = mat_kdl.p.x();
  mat_eigen(1,3) = mat_kdl.p.y();
  mat_eigen(2,3) = mat_kdl.p.z();

  return mat_eigen;
}

/**
 * @brief CONV::jacobian_kdl2eigen
 * @param mat_kdl KDL matrix to convert
 * @return mat_eigen matrix converted
 * @warning WARNING: KDL is row major. Eigen and cmat column major.
 * BUT kdl::jacobian is actually stored as eigen matrix so the conv is a simply copy
 * (function here to completeness, even if it is banal)
 */
Eigen::MatrixXd CONV::jacobian_kdl2eigen(KDL::Jacobian mat_kdl){
  Eigen::MatrixXd mat_eigen = mat_kdl.data;
  return mat_eigen;
}

/**
 * @brief CONV::transfMatrix_eigen2kdl
 * @param mat_eigen transf matrix to convert
 * @return mat_kdl transf matrix converted
 * @warning WARNING: KDL is row major. Eigen and cmat column major.
 */
KDL::Frame CONV::transfMatrix_eigen2kdl(Eigen::Matrix4d mat_eigen){

  KDL::Frame mat_kdl; //default construct create identity


  //REMEMBER: kdl is ROW MAJOR, EIGEN is COLUMN MAJOR
  Eigen::Matrix3d rot_eigen = mat_eigen.topLeftCorner<3,3>();

  mat_kdl.M.data[0] = rot_eigen(0,0);
  mat_kdl.M.data[1] = rot_eigen(0,1);
  mat_kdl.M.data[2] = rot_eigen(0,2);
  //kdl second row
  mat_kdl.M.data[3] = rot_eigen(1,0);
  mat_kdl.M.data[4] = rot_eigen(1,1);
  mat_kdl.M.data[5] = rot_eigen(1,2);
  //kdl third row
  mat_kdl.M.data[6] = rot_eigen(2,0);
  mat_kdl.M.data[7] = rot_eigen(2,1);
  mat_kdl.M.data[8] = rot_eigen(2,2);

  //trasl part
  mat_kdl.p.x(mat_eigen(0,3));
  mat_kdl.p.y(mat_eigen(1,3));
  mat_kdl.p.z(mat_eigen(2,3));




  return mat_kdl;

}


