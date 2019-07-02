#ifndef CONVERSIONS_H
#define CONVERSIONS_H

#include <iostream>
#include <cmat/cmat.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/Transform.h>
#include <tf_conversions/tf_eigen.h>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <opencv2/core.hpp>

namespace CONV{

    /// to/from std vectors
    std::vector<double> tfMat3x3_to_vector(tf::Matrix3x3 matrix3x3);
    Eigen::VectorXd vector_std2Eigen(std::vector<double> vect);
    std::vector<double> vector_cmat2std(CMAT::Matrix mat);
    std::vector<double> vector_eigen2std(Eigen::VectorXd vect);

    /// cmat to eigen
    Eigen::Matrix3d rotMatrix_cmat2eigen(CMAT::RotMatrix mat_cmat);
    Eigen::Matrix4d transfMatrix_cmat2eigen(CMAT::TransfMatrix mat_cmat);
    Eigen::MatrixXd matrix_cmat2eigen (CMAT::Matrix mat_cmat);

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
    //tf::Transform transfMatrix_eigen2tf(Eigen::Matrix4d mat_eigen);

    // KDL to Eigen.
    Eigen::Matrix4d transfMatrix_kdl2eigen(KDL::Frame mat_kdl);
    Eigen::MatrixXd jacobian_kdl2eigen(KDL::Jacobian mat_kdl);

    //Eigen to KDL
    KDL::Frame transfMatrix_eigen2kdl(Eigen::Matrix4d mat_eigen);

    //openCV ~ CMAT
    //CMAT::Matrix matrix_opencv2cmat(cv::Mat1d mat_opencv);

    //visp ~ CMAT
    CMAT::Matrix matrix_visp2cmat(vpMatrix mat_visp);

    //visp ~ eigen
    Eigen::MatrixXd matrix_visp2eigen(vpMatrix mat_visp);
    vpHomogeneousMatrix transfMatrix_eigen2visp(Eigen::MatrixXd mat_eigen);

    //eigen ~ geometry_msgs
    Eigen::Matrix4d transfMatrix_geomMsgs2Eigen(geometry_msgs::Transform mat_msgs);
    geometry_msgs::Transform transfMatrix_eigen2geomMsgs(Eigen::Matrix4d mat_eigen);


}

#endif // CONVERSIONS_H
