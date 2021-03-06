#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#if PCL_OMP
#include <pcl/features/normal_3d_omp.h>
#else
#include <pcl/features/normal_3d.h>
#endif
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <walrus_stair_detector/ransac.h>
#include <walrus_stair_detector/histogram.h>
#include <walrus_stair_detector/parallel_plane_ransac_model.h>

namespace walrus_stair_detector {

WalrusStairDetector::WalrusStairDetector() : shutdown_(false),
					     min_rise_(0.1), max_rise_(0.35),
					     min_run_(0.2), max_run_(0.4),
					     max_skipped_risers_(2), min_stair_width_(0.5) {
#if VISUALIZE
  visualizer_update_ = false;
  visualizer_thread_.reset(new boost::thread(boost::bind(&WalrusStairDetector::visualize, this)));
  previous_plane_visual_count_ = 0;
  previous_risers_count_ = 0;
  previous_stair_count_ = 0;
#endif
}
WalrusStairDetector::~WalrusStairDetector() {
  shutdown_ = true;
#if VISUALIZE
  visualizer_thread_->join();
#endif
}
void WalrusStairDetector::detect(const PointCloud::ConstPtr& original_cloud, const Eigen::Vector3f& vertical_estimate, std::vector<StairModel>* stairs, MultiTimer* timer) {
  stairs->clear();
#if VISUALIZE
  boost::mutex::scoped_lock lock(visualizer_mutex_);
  visualizer_update_ = true;
  plane_visual_ = std::vector<DetectedPlane::Ptr>();
  model_ = StairModel();
  risers_visual_.clear();
#endif

#if DEBUG
  std::cout << "Cloud: width = " << original_cloud->width << ", height = " << original_cloud->height << ", size = " << original_cloud->points.size() << std::endl;
#endif

  // Downsize the pointcloud
  PointCloud::Ptr downsized_cloud (new PointCloud);
  downsizePointCloud(original_cloud, downsized_cloud);
  timer->end("downsize");

  // Remove invalid points
  pcl::IndicesPtr filter_indices(new std::vector <int>);
  pcl::removeNaNFromPointCloud<PointCloud::PointType>(*downsized_cloud, *filter_indices);
  timer->end("filter");

  // Compute the normal of all points
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  computeNormals(downsized_cloud, filter_indices, normals);
#if VISUALIZE
  normals_visual_ = normals;
#endif
  timer->end("normals");

  // Grow points to form planes
  std::vector<pcl::PointIndices> clusters;
  regionGrow(downsized_cloud, filter_indices, normals, &clusters);
  timer->end("region grow");


  std::vector<DetectedPlane::Ptr> planes;
  int plane_id = 0;
  BOOST_FOREACH(const pcl::PointIndices& cluster, clusters) {
    DetectedPlane::Ptr plane(new DetectedPlane(plane_id++));

    bool plane_model_result = computePlaneModel(downsized_cloud, cluster, plane->coefficients, plane->inliers);
    if(plane_model_result) {
      plane->normal[0] = plane->coefficients->values[0];
      plane->normal[1] = plane->coefficients->values[1];
      plane->normal[2] = plane->coefficients->values[2];
      plane->normal.normalize();
      planes.push_back(plane);
    }
    timer->end("plane_model");
    if(!plane_model_result)
      continue;

    // Project all points into the computed plane
    projectPoints(downsized_cloud, plane->inliers, plane->coefficients, plane->cluster_projected);
    timer->end("project to planes");
  }

  Eigen::Vector3f vertical;
  computeVertical(planes, vertical_estimate.normalized(), &vertical);

  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    // Compute the bounding points of the plane
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(plane->cluster_projected);
    chull.reconstruct(*plane->cluster_hull);
    timer->end("convex hull");

    computePlaneOrientation(plane, vertical);

    if(plane->orientation == DetectedPlane::Vertical) {
      Eigen::Vector3f plane_horizontal = vertical.cross(plane->normal).normalized();
      computePlaneSize(plane, plane_horizontal, vertical);
    }

    guessPlaneType(plane);

    timer->end("plane properties");


    timer->skip();
  }
#if VISUALIZE
  plane_visual_ = planes;
#endif

  StairModel model;
  model.vertical = vertical;

  Eigen::Vector3f riser_direction;
  bool stair_orientation_result = computeStairOrientation(planes, vertical, &riser_direction);
  timer->end("compute stair direction");

  if(!stair_orientation_result) {
#if DEBUG
    std::cout << "Could not compute riser orientation" << std::endl;
#endif
    return;
  }

  // need to normalize because vertical and direction may not be orthagonal
  model.horizontal = vertical.cross(riser_direction).normalized();
  model.direction = model.horizontal.cross(vertical).normalized();

  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(plane->orientation == DetectedPlane::Horizontal) {
      computePlaneSize(plane, model.horizontal, model.direction);
    }
  }

#if DEBUG
  std::cout << "Found stair orientation: " << model.direction[0] << ", " << model.direction[1] << ", " << model.direction[2] << std::endl;
#endif

  std::vector<StairRiserModel::Ptr> risers;
  double base_offset_z;
  bool riser_spacing_result = computeRun(planes, model, &model.run, &base_offset_z, &risers);
  timer->end("compute riser spacing");

  if(!riser_spacing_result) {
#if DEBUG
    std::cout << "Could not compute riser spacing" << std::endl;
#endif
    return;
  }

#if DEBUG
  std::cout << "Found stair run: " << model.run << std::endl;
#endif

  computeNumStairs(&risers, &model.num_stairs);

#if DEBUG
  std::cout << "Found number of stairs: " << model.num_stairs << std::endl;
#endif

  double base_y;
  bool rise_result = computeRiseFromRisers(risers, &base_y, &model.rise);
  timer->end("compute stair rise from risers");

  if(!rise_result) {
#if DEBUG
    std::cout << "Could not compute stair rise from risers" << std::endl;
#endif
    return;
  }

#if DEBUG
  std::cout << "Found stair rise: " << model.rise << std::endl;
#endif

  double center_x;
  computeWidth(risers, &model.width, &center_x);

#if VISUALIZE
  risers_visual_ = risers;
#endif

  StairRiserModel::Ptr base_riser = risers[0];

  model.origin = model.horizontal * center_x + model.vertical * base_y + model.direction * base_offset_z;

#if DEBUG
  std::cout << "Origin: " << model.origin[0] << ", " << model.origin[1] << ", " << model.origin[2] << std::endl;
  std::cout << "Direction: " << model.direction[0] << ", " << model.direction[1] << ", " << model.direction[2] << std::endl;
#endif

  stairs->push_back(model);
  timer->end("finalize model");
#if VISUALIZE
  model_ = model;
#endif
}

void WalrusStairDetector::downsizePointCloud(const PointCloud::ConstPtr& cloud, PointCloud::Ptr out) {
  PointCloud::Ptr voxel(new PointCloud());
  pcl::VoxelGrid<PointCloud::PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f);
  voxel_grid.filter(*voxel);

  pcl::CropBox<PointCloud::PointType> crop_box;
  crop_box.setInputCloud(voxel);
  Eigen::Vector4f minPoint;
  minPoint << -4, -4, 0, 0;
  Eigen::Vector4f maxPoint;
  maxPoint << 4, 4, 6, 0;
  crop_box.setMin(minPoint);
  crop_box.setMax(maxPoint);
  crop_box.filter(*out);
}

void WalrusStairDetector::computeNormals(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
					 pcl::PointCloud <pcl::Normal>::Ptr out) {
  pcl::search::Search<PointCloud::PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointCloud::PointType> > (new pcl::search::KdTree<PointCloud::PointType>);
#if PCL_OMP
  pcl::NormalEstimationOMP<PointCloud::PointType, pcl::Normal> normal_estimator;
#else
  pcl::NormalEstimation<PointCloud::PointType, pcl::Normal> normal_estimator;
#endif
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud);
  normal_estimator.setIndices(indices);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*out);
}

void WalrusStairDetector::regionGrow(const PointCloud::ConstPtr cloud, const pcl::IndicesPtr indices,
				     const pcl::PointCloud <pcl::Normal>::Ptr normals,
				     std::vector<pcl::PointIndices>* clusters) {
  pcl::search::Search<PointCloud::PointType>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointCloud::PointType> > (new pcl::search::KdTree<PointCloud::PointType>);

  pcl::RegionGrowing<PointCloud::PointType, pcl::Normal> reg;
  reg.setMinClusterSize(50);
  reg.setMaxClusterSize(1000000);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(30);
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(4.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold(1.0);

  reg.extract(*clusters);

#if VISUALIZE
  region_growing_cloud_visual_ = cloud;
#endif
}


bool WalrusStairDetector::computePlaneModel(const PointCloud::ConstPtr cloud, const pcl::PointIndices& cluster,
					    pcl::ModelCoefficients::Ptr coefficients, const pcl::PointIndices::Ptr inliers) {
  pcl::IndicesPtr indices (new std::vector <int>);
  *indices = cluster.indices;

  pcl::SACSegmentation<PointCloud::PointType> segmentation;
  segmentation.setInputCloud(cloud);
  segmentation.setIndices(indices);
  segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setDistanceThreshold(0.01);
  segmentation.setOptimizeCoefficients(true);

  segmentation.segment(*inliers, *coefficients);

  if (inliers->indices.size() == 0)
    return false;
  else
    return true;
}

void WalrusStairDetector::computeVertical(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical_estimate, Eigen::Vector3f* vertical) {
  std::vector<DetectedPlane::Ptr> horizontal_planes;
  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(vertical_estimate.dot(plane->normal) > cos(16 * M_PI / 180)) {
      horizontal_planes.push_back(plane);
    }
  }

  // if no planes detected then just use the original estimate
  if(horizontal_planes.size() == 0) {
#if DEBUG
    std::cout << "Failed to estimate vertical, not horizontal planes found" << std::endl;
#endif
    *vertical = vertical_estimate;
    return;
  }

  bool ransac_result = false;
  if(horizontal_planes.size() >= 2) {
    ParallelPlaneRansacModel<&DetectedPlane::normal> model_description;
    model_description.setWeighBasedOnPointCount(true);
    Ransac<DetectedPlane::Ptr, Eigen::Vector3f> ransac(&model_description);
    ransac.setInput(&horizontal_planes);
    ransac.setMaxIterations(30);
    std::vector<int> inliers;
    ransac_result = ransac.estimate(&inliers, vertical);
  }
  if(ransac_result) {
#if DEBUG
    std::cout << "Computed vertical: " << (*vertical)[0] << ", " << (*vertical)[1] << ", " << (*vertical)[2] << std::endl;;
#endif
  }
  else {
    // will have at least one plane because of if statment above
    int max_points = horizontal_planes[0]->cluster_projected->size();
    DetectedPlane::Ptr plane = horizontal_planes[0];
    for(size_t i = 1; i < horizontal_planes.size(); ++i) {
      int num_points = horizontal_planes[i]->cluster_projected->size();
      if(num_points > max_points) {
	plane = horizontal_planes[0];
	max_points = num_points;
      }
    }

    if(max_points > 3000) {
      *vertical = plane->normal;
#if DEBUG
      std::cout << "Estimated vertical from largest plane [" << max_points << " points]: " << (*vertical)[0] << ", " << (*vertical)[1] << ", " << (*vertical)[2] << std::endl;
#endif
    }
    else {
      *vertical = vertical_estimate;
#if DEBUG
      std::cout << "Failed to estimate vertical from " << horizontal_planes.size() << " planes, largest plane was " << max_points << " points, falling back to initial estimate" << std::endl;
#endif
    }
  }
}


void WalrusStairDetector::projectPoints(const PointCloud::ConstPtr cloud, const pcl::PointIndices::ConstPtr cluster_inliers,
					pcl::ModelCoefficients::ConstPtr coefficients, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_cloud) {
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud);
  proj.setIndices(cluster_inliers);
  proj.setModelCoefficients(coefficients);
  proj.filter(*projected_cloud);
}

void WalrusStairDetector::computePlaneOrientation(DetectedPlane::Ptr plane, const Eigen::Vector3f& vertical) {
  if(vertical.dot(plane->normal) > cos(16 * M_PI / 180)) {
    plane->orientation = DetectedPlane::Horizontal;
  }
  else if(vertical.dot(plane->normal) < cos((90 - 30) * M_PI / 180)) {
    plane->orientation = DetectedPlane::Vertical;
  }
}
void WalrusStairDetector::computePlaneSize(DetectedPlane::Ptr plane, const Eigen::Vector3f& x_axis, const Eigen::Vector3f& y_axis) {
  // Pick a random reference point
  Eigen::Vector3f p0 = plane->cluster_hull->at(0).getVector3fMap();

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  BOOST_FOREACH(const pcl::PointXYZ& abs_point, *plane->cluster_hull) {
    Eigen::Vector3f p = abs_point.getVector3fMap() - p0;
    double x = p.dot(x_axis);
    double y = p.dot(y_axis);
    if(x > max_x)
      max_x = x;
    if(x < min_x)
      min_x = x;
    if(y > max_y)
      max_y = y;
    if(y < min_y)
      min_y = y;
  }
  plane->width = max_x - min_x;
  plane->height = max_y - min_y;

  double center_x = (min_x + max_x) / 2;
  double center_y = (min_y + max_y) / 2;
  plane->center = p0 + x_axis * center_x + y_axis * center_y;
}


void WalrusStairDetector::guessPlaneType(DetectedPlane::Ptr plane) {
  if(plane->orientation == DetectedPlane::Vertical) {
    plane->is_tread = false;
    if(plane->width < plane->height * 1.5)
      plane->is_riser = false;
    if(plane->height > max_rise_)
      plane->is_riser = false;
   }
  else if(plane->orientation == DetectedPlane::Horizontal) {
    plane->is_riser = false;
    plane->is_wall = false;
  }
  else {
    plane->is_riser = false;
    plane->is_tread = false;
    plane->is_wall = false;
  }
}


bool WalrusStairDetector::computeStairOrientation(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical, Eigen::Vector3f* model) {
  std::vector<DetectedPlane::Ptr> potential_risers;
  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(plane->is_riser || indeterminate(plane->is_riser)) {
      plane->horizontal_normal = (plane->normal - vertical * plane->normal.dot(vertical)).normalized();
      potential_risers.push_back(plane);
    }
  }
  Eigen::Vector3f riser_orientation;
  ParallelPlaneRansacModel<&DetectedPlane::horizontal_normal> model_description;
  Ransac<DetectedPlane::Ptr, Eigen::Vector3f> ransac(&model_description);
  ransac.setInput(&potential_risers);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, &riser_orientation);
  if(result) {
    for(size_t i = 0; i < potential_risers.size(); ++i) {
      if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
	potential_risers[i]->is_tread = false;
      }
    }
  }
  // compute portion of the riser orientation perp to the vertical
  *model = riser_orientation - riser_orientation.dot(vertical) * vertical;
  return result;
}

bool WalrusStairDetector::computeRun(std::vector<DetectedPlane::Ptr>& planes, const StairModel& stair_model, double* run, double* base_offset_z, std::vector<StairRiserModel::Ptr>* risers) {
  std::vector<DetectedPlane::Ptr> potential_risers;
  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(plane->is_riser || indeterminate(plane->is_riser)) {
      potential_risers.push_back(plane);
    }
  }

  if(potential_risers.size() < 2) {
#if DEBUG
    std::cout << "Not enough planes to compute riser spacing from" << std::endl;
#endif
    return false;
  }

  // compute distance between planes along stair direction
  std::vector<double> potential_risers_dist;
  for(size_t i = 0; i < potential_risers.size(); ++i) {
    const DetectedPlane::Ptr& p = potential_risers[i];
    double dist = stair_model.direction.dot(p->center);
    potential_risers_dist.push_back(dist);
  }


  std::vector<double> potential_risers_dist_sorted(potential_risers_dist.begin(), potential_risers_dist.end());
  std::sort(potential_risers_dist_sorted.begin(), potential_risers_dist_sorted.end());
  std::vector<double> dists;
  for(std::vector<double>::const_iterator itr = potential_risers_dist_sorted.begin(); itr != potential_risers_dist_sorted.end(); ++itr) {
    std::vector<double>::const_iterator itr2 = itr;
    ++itr2;
    for(; itr2 != potential_risers_dist_sorted.end(); ++itr2) {
      double dist = *itr2 - *itr;
      if(dist > max_run_ * (max_skipped_risers_ + 1))
	break;
      dists.push_back(dist);
    }
  }

  Histogram hist(min_run_, max_run_, 0.02, 100);
  BOOST_FOREACH(double dist, dists) {
    if(dist < max_run_)
      hist.add(dist);
    for(size_t i = 2; i <= max_skipped_risers_ + 1; ++i) {
      double new_dist = dist / i;
      if(new_dist > min_run_ && new_dist < max_run_)
	hist.add(dist, 1.0 / i / i / i);
    }
  }
#if DEBUG
  hist.print();
#endif

  DistanceModel model;
  DistanceRansacModel model_description(hist.largestBucket(), 0.1);
  Ransac<double, DistanceModel> ransac(&model_description);
  ransac.setInput(&potential_risers_dist);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, &model);

  if(result) {
    *run = model.spacing;
    for(size_t i = 0; i < potential_risers.size(); ++i) {
      if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
	potential_risers[i]->is_tread = false;
      }
    }
    std::map<int, std::vector<DetectedPlane::Ptr> > riser_groups;
    for(std::multimap<int, int>::iterator itr = model.groups.begin(); itr != model.groups.end(); ++itr) {
      DetectedPlane::Ptr& plane = potential_risers[inliers[itr->second]];
      riser_groups[itr->first].push_back(plane);
    }
    for(std::multimap<int, std::vector<DetectedPlane::Ptr> >::iterator itr = riser_groups.begin(); itr != riser_groups.end(); ++itr) {
      StairRiserModel::Ptr riser(new StairRiserModel(itr->first));
      riser->planes = itr->second;
      double average_z = 0;
      BOOST_FOREACH(const DetectedPlane::Ptr& plane, riser->planes) {
	double center_x = plane->center.dot(stair_model.horizontal);
	double center_y = plane->center.dot(stair_model.vertical);
	double center_z = plane->center.dot(stair_model.direction);
	average_z += center_z;
	double min_x = center_x - plane->width / 2;
	double max_x = center_x + plane->width / 2;
	double min_y = center_y - plane->height / 2;
	double max_y = center_y + plane->height / 2;
	if(min_x < riser->min_x)
	  riser->min_x = min_x;
	if(max_x > riser->max_x)
	  riser->max_x = max_x;
	if(min_y < riser->min_y)
	  riser->min_y = min_y;
	if(max_y > riser->max_y)
	  riser->max_y = max_y;
      }
      average_z /= riser->planes.size();
      riser->center_z = average_z;
      risers->push_back(riser);
    }
    *base_offset_z = model.offset;
  }
  return result;
}


void WalrusStairDetector::computeNumStairs(std::vector<StairRiserModel::Ptr>* risers, unsigned int* num_stairs) {
  std::vector<StairRiserModel::Ptr>::iterator itr = risers->begin();
  int last_index = 0;
  while(itr != risers->end()) {
    if((*itr)->index - last_index > max_skipped_risers_ + 1) {
      break;
    }
    *num_stairs = (*itr)->index + 1; // plus 1 for first stair
    last_index = (*itr)->index;
    ++itr;
  }
  risers->erase(itr, risers->end()); // remove remaining stairs
}

bool WalrusStairDetector::computeRiseFromRisers(const std::vector<StairRiserModel::Ptr>& risers, double* base_y, double* rise) {
  std::vector<std::pair<double, double> > points;
  BOOST_FOREACH(const StairRiserModel::Ptr& riser, risers) {
    double center_y = (riser->min_y + riser->max_y)/2;
    points.push_back(std::make_pair(riser->index, center_y));
  }

  LineModel slope_model;
  LineRansacModel model_description;
  model_description.setSlopeBounds(min_rise_, max_rise_);
  Ransac<std::pair<double, double>, LineModel> ransac(&model_description);
  ransac.setInput(&points);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, &slope_model);
  if(result) {
    *rise = slope_model.slope;
    *base_y = slope_model.offset - *rise/2;
  }
  return result;
}

void WalrusStairDetector::computeWidth(const std::vector<StairRiserModel::Ptr>& risers, double* width, double* center_x) {
  std::vector<std::pair<double, double> > min_points;
  std::vector<std::pair<double, double> > max_points;
  BOOST_FOREACH(const StairRiserModel::Ptr& riser, risers) {
    if(riser->max_x - riser->min_x < min_stair_width_)
      continue;
    min_points.push_back(std::make_pair(riser->index, riser->min_x));
    max_points.push_back(std::make_pair(riser->index, riser->max_x));
  }

  LineRansacModel model_description;
  model_description.setInlierThreshold(0.3);
  model_description.setMinInliers(0.75);

  LineModel min_model;
  Ransac<std::pair<double, double>, LineModel> min_ransac(&model_description);
  min_ransac.setInput(&min_points);
  min_ransac.setMaxIterations(30);
  std::vector<int> min_inliers;
  bool min_result = min_ransac.estimate(&min_inliers, &min_model);

  LineModel max_model;
  Ransac<std::pair<double, double>, LineModel> max_ransac(&model_description);
  max_ransac.setInput(&max_points);
  max_ransac.setMaxIterations(30);
  std::vector<int> max_inliers;
  bool max_result = max_ransac.estimate(&max_inliers, &max_model);

  double min, max;
  if(min_result) {
    min = min_model.offset;
#if DEBUG
    std::cout << "Used RANSAC to compute stair left" << std::endl;
#endif
  }
  else {
#if DEBUG
    std::cout << "RANSAC failed for stair left" << std::endl;
#endif
    min = risers[0]->min_x;
  }

  if(max_result) {
#if DEBUG
    std::cout << "Used RANSAC to compute stair right" << std::endl;
#endif
    max = max_model.offset;
  }
  else {
#if DEBUG
    std::cout << "RANSAC failed for stair right" << std::endl;
#endif
    max = risers[0]->max_x;
  }

  *width = max - min;
  *center_x = (max + min) / 2;
}



#if VISUALIZE
static std::string visName(const std::string& name, int index) {
  std::stringstream ss;
  ss << name << "_" << index;
  return ss.str();
}
static Eigen::Vector3f RED(1, 0, 0);
static Eigen::Vector3f GREEN(0, 1, 0);
static Eigen::Vector3f BLUE(0, 0, 1);
static Eigen::Vector3f YELLOW(1, 1, 0);
static Eigen::Vector3f CYAN(0, 1, 1);
static Eigen::Vector3f PURPLE(1, 0, 1);
static Eigen::Vector3f PINK(1, 0, 0.5);
void WalrusStairDetector::visualize() {
  visualizer_.reset(new pcl::visualization::PCLVisualizer("WALRUS Stair Detector"));
  visualizer_->addCoordinateSystem(1.0);
  visualizer_->initCameraParameters();
  visualizer_->setCameraPosition(0, 0, -1, 0, -1, 0);

  while(!visualizer_->wasStopped() && !shutdown_) {
    visualizer_->spinOnce(100);
    {
      boost::mutex::scoped_lock lock(visualizer_mutex_, boost::try_to_lock);
      if(lock && visualizer_update_) {
#if VISUALIZE_POINT_CLOUD
	if(region_growing_cloud_visual_) {
	  if(!visualizer_->updatePointCloud(region_growing_cloud_visual_, "region_growing_cloud")) {
	    visualizer_->addPointCloud(region_growing_cloud_visual_, "region_growing_cloud");
	    visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "region_growing_cloud");
	  }
	}
#endif

#if VISUALIZE_DETECTED_PLANES
	// Remove old shapes
	for(size_t i = 0; i < previous_plane_visual_count_; ++i)
	  visualizer_->removeShape(visName("plane", i));
	// Add/update shapes
	for(size_t i = 0; i < plane_visual_.size(); ++i) {
	  DetectedPlane::Ptr plane = plane_visual_[i];
	  Eigen::Vector3f color;
	  if(plane->flag)
	    color = PURPLE;
	  else if(plane->is_riser || indeterminate(plane->is_riser))
	    color = BLUE;
	  else if(plane->is_tread || indeterminate(plane->is_tread))
	    color = RED;
	  else if(plane->is_wall || indeterminate(plane->is_wall))
	    color = YELLOW;
	  else
	    color = GREEN;
	  visualizer_->addPolygon<pcl::PointXYZ>(plane->cluster_hull, color[0], color[1], color[2], visName("plane", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("plane", i));
	}
	previous_plane_visual_count_ = plane_visual_.size();
#endif
#if VISUALIZE_DETECTED_PLANE_NORMALS
	if(region_growing_cloud_visual_ && normals_visual_) {
	  visualizer_->removePointCloud("normals");
	  visualizer_->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (region_growing_cloud_visual_, normals_visual_, 2, 0.05, "normals");
	}
#endif

	std::stringstream run_text;
	run_text << "Run: " << model_.run << "m";
	if(!visualizer_->updateText(run_text.str(), 30, 30, "run")) {
	  visualizer_->addText(run_text.str(), 30, 30, 16, 1, 1, 1, "run");
	}

	std::stringstream rise_text;
	rise_text << "Rise: " << model_.rise << "m";
	if(!visualizer_->updateText(rise_text.str(), 30, 50, "rise")) {
	  visualizer_->addText(rise_text.str(), 30, 50, 16, 1, 1, 1, "rise");
	}

	std::stringstream width_text;
	width_text << "Width: " << model_.width << "m";
	if(!visualizer_->updateText(width_text.str(), 30, 70, "width")) {
	  visualizer_->addText(width_text.str(), 30, 70, 16, 1, 1, 1, "width");
	}

	std::stringstream num_text;
	num_text << "Number of stairs: " << model_.num_stairs;
	if(!visualizer_->updateText(num_text.str(), 30, 90, "num")) {
	  visualizer_->addText(num_text.str(), 30, 90, 16, 1, 1, 1, "num");
	}

	std::stringstream origin_text;
	origin_text << "Origin (stair relative): " << model_.origin.dot(model_.horizontal) << ", " << model_.origin.dot(model_.vertical) << ", " << model_.origin.dot(model_.direction);
	if(!visualizer_->updateText(origin_text.str(), 30, 110, "origin")) {
	  visualizer_->addText(origin_text.str(), 30, 110, 16, 1, 1, 1, "origin");
	}

#if VISUALIZE_RISERS
	// Remove old shapes
	for(size_t i = 0; i < previous_risers_count_; ++i) {
	  visualizer_->removeShape(visName("initial_riser", i));
	}
	for(size_t i = 0; i < risers_visual_.size(); ++i) {
	  Eigen::Vector3f z = model_.direction * risers_visual_[i]->center_z;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr riser_points(new pcl::PointCloud<pcl::PointXYZ>());
	  riser_points->resize(4);
	  riser_points->at(0).getVector3fMap() = z + model_.horizontal * risers_visual_[i]->min_x + model_.vertical * risers_visual_[i]->min_y;
	  riser_points->at(1).getVector3fMap() = z + model_.horizontal * risers_visual_[i]->max_x + model_.vertical * risers_visual_[i]->min_y;
	  riser_points->at(2).getVector3fMap() = z + model_.horizontal * risers_visual_[i]->max_x + model_.vertical * risers_visual_[i]->max_y;
	  riser_points->at(3).getVector3fMap() = z + model_.horizontal * risers_visual_[i]->min_x + model_.vertical * risers_visual_[i]->max_y;
	  visualizer_->addPolygon<pcl::PointXYZ>(riser_points, RED[0], RED[1], RED[2], visName("initial_riser", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("initial_riser", i));
	}
	previous_risers_count_ = risers_visual_.size();
#endif

#if VISUALIZE_STAIRS
	// Remove old shapes
	for(size_t i = 0; i < previous_stair_count_; ++i) {
	  visualizer_->removeShape(visName("riser", i));
	  visualizer_->removeShape(visName("tread", i));
	}

	for(size_t i = 0; i < model_.num_stairs; ++i) {
	  Eigen::Vector3f base = model_.origin + model_.vertical * model_.rise * i + model_.direction * model_.run * i;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr riser_points(new pcl::PointCloud<pcl::PointXYZ>());
	  riser_points->resize(4);
	  riser_points->at(0).getVector3fMap() = base + model_.horizontal * model_.width/2 + model_.vertical*model_.rise;
	  riser_points->at(1).getVector3fMap() = base - model_.horizontal * model_.width/2 + model_.vertical*model_.rise;
	  riser_points->at(2).getVector3fMap() = base - model_.horizontal * model_.width/2;
	  riser_points->at(3).getVector3fMap() = base + model_.horizontal * model_.width/2;
	  visualizer_->addPolygon<pcl::PointXYZ>(riser_points, CYAN[0], CYAN[1], CYAN[2], visName("riser", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("riser", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, visName("riser", i));


	  pcl::PointCloud<pcl::PointXYZ>::Ptr tread_points(new pcl::PointCloud<pcl::PointXYZ>());
	  tread_points->resize(4);
	  tread_points->at(0).getVector3fMap() = base + model_.vertical*model_.rise + model_.horizontal * model_.width/2 + model_.direction*model_.run;
	  tread_points->at(1).getVector3fMap() = base + model_.vertical*model_.rise - model_.horizontal * model_.width/2 + model_.direction*model_.run;
	  tread_points->at(2).getVector3fMap() = base + model_.vertical*model_.rise - model_.horizontal * model_.width/2;
	  tread_points->at(3).getVector3fMap() = base + model_.vertical*model_.rise + model_.horizontal * model_.width/2;
	  visualizer_->addPolygon<pcl::PointXYZ>(tread_points, PINK[0], PINK[1], PINK[2], visName("tread", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("tread", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, visName("tread", i));
	}
	previous_stair_count_ = model_.num_stairs;
#endif

#if VISUALIZE_STAIR_MEASUREMENTS
	pcl::PointXYZ p1, p2;
	p1.getVector3fMap() = model_.origin + model_.vertical * model_.rise * 1 + model_.direction * model_.run * 0 - model_.horizontal * (model_.width/2 +0.05);
	p2.getVector3fMap() = p1.getVector3fMap() + model_.direction * model_.run;

	visualizer_->removeShape("run_measure");
	visualizer_->addLine(p1, p2, "run_measure");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "run_measure");

	p1.getVector3fMap() = model_.origin + model_.vertical * model_.rise * 1 + model_.direction * model_.run * 1 - model_.horizontal * (model_.width/2 +0.05);
	p2.getVector3fMap() = p1.getVector3fMap() + model_.vertical * model_.rise;
	visualizer_->removeShape("rise_measure");
	visualizer_->addLine(p1, p2, "rise_measure");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "rise_measure");

	p1.getVector3fMap() = model_.origin + model_.vertical * model_.rise * 1 + model_.direction * model_.run * 1 - model_.horizontal * (model_.width/2 +0.02);
	p2.getVector3fMap() = p1.getVector3fMap() - model_.horizontal * 0.06;
	visualizer_->removeShape("middle_cross");
	visualizer_->addLine(p1, p2, "middle_cross");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "middle_cross");

	p1.getVector3fMap() = model_.origin + model_.vertical * model_.rise * 2 + model_.direction * model_.run * 1 - model_.horizontal * (model_.width/2 +0.02);
	p2.getVector3fMap() = p1.getVector3fMap() - model_.horizontal * 0.06;
	visualizer_->removeShape("first_cross");
	visualizer_->addLine(p1, p2, "first_cross");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "first_cross");

	p1.getVector3fMap() = model_.origin + model_.vertical * model_.rise * 1 + model_.direction * model_.run * 0 - model_.horizontal * (model_.width/2 +0.02);
	p2.getVector3fMap() = p1.getVector3fMap() - model_.horizontal * 0.06;
	visualizer_->removeShape("last_cross");
	visualizer_->addLine(p1, p2, "last_cross");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "last_cross");

	p1.getVector3fMap() = model_.origin - model_.horizontal * model_.width/2 - model_.direction * 0.05;
	p2.getVector3fMap() = p1.getVector3fMap() + model_.horizontal * model_.width;
	visualizer_->removeShape("width_measure");
	visualizer_->addLine(p1, p2, "width_measure");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "width_measure");

	p1.getVector3fMap() = model_.origin - model_.horizontal * model_.width/2 - model_.direction * 0.02;
	p2.getVector3fMap() = p1.getVector3fMap() - model_.direction * 0.06;
	visualizer_->removeShape("width_right");
	visualizer_->addLine(p1, p2, "width_right");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "width_right");

	p1.getVector3fMap() = model_.origin + model_.horizontal * model_.width/2 - model_.direction * 0.02;
	p2.getVector3fMap() = p1.getVector3fMap() - model_.direction * 0.06;
	visualizer_->removeShape("width_left");
	visualizer_->addLine(p1, p2, "width_left");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, "width_left");
#endif

#if VISUALIZE_STAIR_AXES
	visualizer_->removeShape("vertical_axis");
	visualizer_->removeShape("direction_axis");
	visualizer_->removeShape("horizontal_axis");
	Eigen::Vector3f axis_origin = model_.origin + model_.vertical * model_.rise * 0 + model_.direction * model_.run * 0;
	double axis_len = 0.5;
	double axis_radius = 0.007;
	pcl::ModelCoefficients cylinder_coeff;
	cylinder_coeff.values.resize (7);
	cylinder_coeff.values[0] = axis_origin[0];
	cylinder_coeff.values[1] = axis_origin[1];
	cylinder_coeff.values[2] = axis_origin[2];
	cylinder_coeff.values[6] = axis_radius;

	cylinder_coeff.values[3] = model_.vertical[0]*axis_len;
	cylinder_coeff.values[4] = model_.vertical[1]*axis_len;
	cylinder_coeff.values[5] = model_.vertical[2]*axis_len;
	visualizer_->addCylinder(cylinder_coeff, "vertical_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "vertical_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, GREEN[0], GREEN[1], GREEN[2], "vertical_axis");

	cylinder_coeff.values[3] = model_.direction[0]*axis_len;
	cylinder_coeff.values[4] = model_.direction[1]*axis_len;
	cylinder_coeff.values[5] = model_.direction[2]*axis_len;
	visualizer_->addCylinder(cylinder_coeff, "direction_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "direction_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, BLUE[0], BLUE[1], BLUE[2], "direction_axis");

	cylinder_coeff.values[3] = model_.horizontal[0]*axis_len;
	cylinder_coeff.values[4] = model_.horizontal[1]*axis_len;
	cylinder_coeff.values[5] = model_.horizontal[2]*axis_len;
	visualizer_->addCylinder(cylinder_coeff, "horizontal_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "horizontal_axis");
	visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, RED[0], RED[1], RED[2], "horizontal_axis");
#endif


	visualizer_update_ = false;
      }
    }
    boost::this_thread::sleep(boost::posix_time::microseconds(1000));
  }
  visualizer_->close();
  visualizer_.reset();
}
#endif


}
