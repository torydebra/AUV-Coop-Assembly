#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <Eigen/Core>
#include <cmat/cmat.h>
#include <tf_conversions/tf_eigen.h>

namespace CONV{

    std::vector<double> tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3);

    /// cmat to eigen
    Eigen::Matrix3d rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat);
    Eigen::Matrix4d transfMatrix_cmat2eigen(CMAT::TransfMatrix mat_cmat);

    /// eigen to cmat
    // one generic is ok, they are the same
    //CMAT::TransfMatrix transfMatrix_eigen2cmat(Eigen::Matrix4d mat_eigen);
    //CMAT::RotMatrix rotMatrix_eigen2cmat(Eigen::Matrix3d mat_eigen);
    CMAT::Matrix matrix_eigen2cmat(Eigen::MatrixXd mat_eigen);

    /// tf to cmat
    // not necessary at moment
    //CMAT::TransfMatrix transfMatrix_tf2cmat(tf::Transform mat_tf);

    /// tf to eigen
    Eigen::Matrix4d transfMatrix_tf2eigen(tf::Transform mat_tf);

    /// eigen to tf
    //not necessary at moment
//    tf::Transform transfMatrix_eigen2tf(Eigen::Matrix4d mat_eigen);


}

#endif // CONVERSIONS_H
