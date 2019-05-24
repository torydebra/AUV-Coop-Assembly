#include "header/depthCameraSupport.h"

struct DCAM::rs_intrinsics {
  float ppx;
  float ppy;
  float fx;
  float fy;
  float coeffs[5];
};

void DCAM::rs_deproject_pixel_to_point(float point[3], const rs_intrinsics &intrin, const float pixel[2], float depth) {
  float x = (pixel[0] - intrin.ppx) / intrin.fx;
  float y = (pixel[1] - intrin.ppy) / intrin.fy;
  float r2 = x * x + y * y;
  float f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2;
  float ux = x * f + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x);
  float uy = y * f + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y);
  x = ux;
  y = uy;
  point[0] = depth * x;
  point[1] = depth * y;
  point[2] = depth;
}


bool DCAM::makePointCloud(vpImage<uint16_t> &I_depth_raw,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud){

  unsigned int height = I_depth_raw.getRows();
  unsigned int width = I_depth_raw.getCols();

  // Transform pointcloud
  pointcloud->width = width;
  pointcloud->height = height;
  pointcloud->reserve((size_t)width * height);
  // Params of depth camera. They are in the scene.xml
  const float depth_scale = 1.0f;
  rs_intrinsics depth_intrinsic;
  depth_intrinsic.ppx = 120.0f;
  depth_intrinsic.ppy = 160.0f;
  depth_intrinsic.fx = 257.986f;
  depth_intrinsic.fy = 257.341f;
  depth_intrinsic.coeffs[0] = 0.0f;
  depth_intrinsic.coeffs[1] = 0.0f;
  depth_intrinsic.coeffs[2] = 0.0f;
  depth_intrinsic.coeffs[3] = 0.0f;
  depth_intrinsic.coeffs[4] = 0.0f;
  for (unsigned int i = 0; i < height; i++) {
    for (unsigned int j = 0; j < width; j++) {
      float scaled_depth = I_depth_raw[i][j] * depth_scale;
      float point[3];
      float pixel[2] = {(float)j, (float)i};
      rs_deproject_pixel_to_point(point, depth_intrinsic, pixel, scaled_depth);
      pointcloud->push_back(pcl::PointXYZ(point[0], point[1], point[2]));
    }
  }
  return true;
}
