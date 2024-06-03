#ifndef BBOX_HPP
#define BBOX_HPP


#pragma once

#include <Eigen/Geometry>

struct BBox {
 public:
  int id;
  Eigen::Vector3f position;
  Eigen::Vector3f dimension;
  Eigen::Quaternionf quaternion;

  BBox() {}

  BBox(int id, Eigen::Vector3f position, Eigen::Vector3f dimension)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(Eigen::Quaternionf(1, 0, 0, 0)) {}

  BBox(int id, Eigen::Vector3f position, Eigen::Vector3f dimension,
      Eigen::Quaternionf quaternion)
      : id(id),
        position(position),
        dimension(dimension),
        quaternion(quaternion) {}
};
#endif // BBOX_HPP
