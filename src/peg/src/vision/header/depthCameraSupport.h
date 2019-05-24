#ifndef DEPTHCAMERASUPPORT_H
#define DEPTHCAMERASUPPORT_H

#include <string>
#include <visp3/io/vpImageIo.h>
#include <pcl_ros/point_cloud.h>

namespace DCAM {

struct rs_intrinsics;

void rs_deproject_pixel_to_point(float point[3], const rs_intrinsics &intrin, const float pixel[2], float depth);
bool makePointCloud(vpImage<uint16_t> &I_depth_raw, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);


}

#endif // DEPTHCAMERASUPPORT_H
