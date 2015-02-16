#ifndef WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_H_
#define WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include <walrus_stair_detector/detected_plane.h>
#include <walrus_stair_detector/distance_ransac_model.h>
#include <walrus_stair_detector/slope_ransac_model.h>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/logic/tribool.hpp>

namespace walrus_stair_detector {
using namespace ::boost;
using namespace ::boost::multi_index;
using namespace ::boost::logic;


class WalrusStairDetector {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  WalrusStairDetector();
  ~WalrusStairDetector();
  void detect(const PointCloud::ConstPtr& cloud);
private:
  void downsizePointCloud(const PointCloud::ConstPtr& cloud, PointCloud::Ptr out);
  void computeNormals(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
		      pcl::PointCloud <pcl::Normal>::Ptr out);
  void regionGrow(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
		  const pcl::PointCloud <pcl::Normal>::Ptr normals,
		  std::vector<pcl::PointIndices>* clusters);
  bool computePlaneModel(const PointCloud::ConstPtr cloud, const pcl::PointIndices& cluster,
			 pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
  void projectPoints(const PointCloud::ConstPtr cloud, const pcl::PointIndices::ConstPtr cluster_inliers,
		     pcl::ModelCoefficients::ConstPtr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud);
  void computeVerticalPlaneSize(DetectedPlane::Ptr plane, const Eigen::Vector3f& vertical);
  void computePlaneOrientation(DetectedPlane::Ptr plane, const Eigen::Vector3f& vertical);
  void guessPlaneType(DetectedPlane::Ptr plane);
  bool computeStairOrientation(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical, Eigen::Vector3f* model);
  bool computeRiserSpacing(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& stair_orientation, DistanceModel* model, int* start_index);
  bool computeRise(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical, const Eigen::Vector3f& stair_orientation, const DistanceModel& riser_spacing, int riser_stair_index, SlopeModel* model);


  double min_riser_height_;
  double max_riser_height_;

  double min_riser_spacing_;
  double max_riser_spacing_;

  int max_skipped_risers_;

  bool shutdown_;
#if VISUALIZE
  boost::scoped_ptr<pcl::visualization::PCLVisualizer> visualizer_;
  boost::scoped_ptr<boost::thread> visualizer_thread_;
  boost::mutex visualizer_mutex_;
  bool visualizer_update_;
  void visualize();

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr region_growing_cloud_visual_;
  std::vector<DetectedPlane::Ptr> plane_visual_;
  int previous_plane_visual_count_;

  Eigen::Vector3f vertical_;
  Eigen::Vector3f stair_orientation_;
  DistanceModel riser_spacing_;
  int riser_start_index_;
  int stair_count_;
  int previous_stair_count_;
  double stair_rise_;
  double stair_vertical_offset_;
  double stair_width_;
#endif
};

}

#endif
