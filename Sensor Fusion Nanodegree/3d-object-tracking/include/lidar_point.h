#ifndef CAMERA_FUSION_LIDAR_POINT_H
#define CAMERA_FUSION_LIDAR_POINT_H

struct LidarPoint {   // single lidar point in space
  double x, y, z, r;  // x,y,z in [m], r is point reflectivity

  LidarPoint(double x_, double y_, double z_, double r_) : x(x_), y(y_), z(z_), r(r_) {}
};

#endif
