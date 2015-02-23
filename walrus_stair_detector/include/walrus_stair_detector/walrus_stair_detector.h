#ifndef WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_H_
#define WALRUS_STAIR_DETECTOR_WALRUS_STAIR_DETECTOR_H_

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include <walrus_stair_detector/detected_plane.h>
#include <walrus_stair_detector/distance_ransac_model.h>
#include <walrus_stair_detector/line_ransac_model.h>

#include <boost/multi_index_container.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <boost/multi_index/identity.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/logic/tribool.hpp>

namespace walrus_stair_detector {
using namespace ::boost;
using namespace ::boost::multi_index;
using namespace ::boost::logic;

struct StairModel {
  Eigen::Vector3f origin; // center bottom of the base riser
  Eigen::Vector3f vertical; // normal of a stair tread
  Eigen::Vector3f direction; // vector in tread plane in the direction of the stair case
  Eigen::Vector3f horizontal;
  double rise;
  double run;
  double width;
  unsigned int num_stairs; // number of stairs (defined as number of risers)
};

struct StairRiserModel {
  StairRiserModel(int index) : index(index),
			       min_x(std::numeric_limits<double>::infinity()), min_y(std::numeric_limits<double>::infinity()),
			       max_x(-std::numeric_limits<double>::infinity()), max_y(-std::numeric_limits<double>::infinity()) {}
  typedef shared_ptr<StairRiserModel> Ptr;
  const int index;

  double min_x, min_y, max_x, max_y;
  std::vector<DetectedPlane::Ptr> planes;
  double center_z;
};

class WalrusStairDetector {
public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

  WalrusStairDetector();
  ~WalrusStairDetector();
  void detect(const PointCloud::ConstPtr& cloud, const Eigen::Vector3f& vertical_estimate, std::vector<StairModel>* stairs);
private:
  void downsizePointCloud(const PointCloud::ConstPtr& cloud, PointCloud::Ptr out);
  void computeNormals(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
		      pcl::PointCloud <pcl::Normal>::Ptr out);
  void regionGrow(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
		  const pcl::PointCloud <pcl::Normal>::Ptr normals,
		  std::vector<pcl::PointIndices>* clusters);
  bool computePlaneModel(const PointCloud::ConstPtr cloud, const pcl::PointIndices& cluster,
			 pcl::ModelCoefficients::Ptr coefficients, pcl::PointIndices::Ptr inliers);
  void computeVertical(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical_estimate, Eigen::Vector3f* vertical);
  void projectPoints(const PointCloud::ConstPtr cloud, const pcl::PointIndices::ConstPtr cluster_inliers,
		     pcl::ModelCoefficients::ConstPtr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud);
  void computePlaneSize(DetectedPlane::Ptr plane, const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_axis);
  void computePlaneOrientation(DetectedPlane::Ptr plane, const Eigen::Vector3f& vertical);
  void guessPlaneType(DetectedPlane::Ptr plane);
  bool computeStairOrientation(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical, Eigen::Vector3f* model);
  bool computeRun(std::vector<DetectedPlane::Ptr>& planes, const StairModel& stair_model, double* run, double* base_offset_z, std::vector<StairRiserModel::Ptr>* risers);
  void computeNumStairs(std::vector<StairRiserModel::Ptr>* risers, unsigned int* num_stairs);
  bool computeRiseFromRisers(const std::vector<StairRiserModel::Ptr>& risers, double* base_y, double* rise);
  void computeWidth(const std::vector<StairRiserModel::Ptr>& risers, double* width, double* center_x);


  bool shutdown_;

  double min_rise_;
  double max_rise_;

  double min_run_;
  double max_run_;

  size_t max_skipped_risers_;

  double min_stair_width_;
#if VISUALIZE
  boost::scoped_ptr<pcl::visualization::PCLVisualizer> visualizer_;
  boost::scoped_ptr<boost::thread> visualizer_thread_;
  boost::mutex visualizer_mutex_;
  bool visualizer_update_;
  void visualize();

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr region_growing_cloud_visual_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_visual_;
  std::vector<DetectedPlane::Ptr> plane_visual_;
  std::vector<StairRiserModel::Ptr> risers_visual_;
  size_t previous_plane_visual_count_;
  size_t previous_risers_count_;
  size_t previous_stair_count_;

  StairModel model_;
#endif
};

}

#endif
