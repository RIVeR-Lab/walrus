#ifndef WALRUS_STAIR_DETECTOR_DETECTED_PLANE_H_
#define WALRUS_STAIR_DETECTOR_DETECTED_PLANE_H_

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/shared_ptr.hpp>
#include <boost/logic/tribool.hpp>

namespace walrus_stair_detector {

using namespace ::boost;

struct DetectedPlane {
  DetectedPlane(int id)
    : id(id),
      orientation(OtherOrientation),
      coefficients(new pcl::ModelCoefficients),
      inliers(new pcl::PointIndices()),
      cluster_projected(new pcl::PointCloud<pcl::PointXYZ>),
      cluster_hull(new pcl::PointCloud<pcl::PointXYZ>),
      is_riser(indeterminate), is_tread(indeterminate), is_wall(indeterminate) {}
  typedef shared_ptr<DetectedPlane> Ptr;

  enum Orientation {
    Horizontal,
    Vertical,
    OtherOrientation
  };


  int id;
  Orientation orientation;
  tribool is_riser;
  tribool is_tread;
  tribool is_wall;

  pcl::ModelCoefficients::Ptr coefficients;
  pcl::PointIndices::Ptr inliers;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_projected;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_hull;

  Eigen::Vector3f centroid;
  Eigen::Vector3f normal;

  double vertical_width;
  double vertical_height;
};

}

#endif
