#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <walrus_stair_detector/multi_timer.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
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
					     min_riser_height_(0.1), max_riser_height_(0.35),
					     min_riser_spacing_(0.2), max_riser_spacing_(0.4),
					     max_skipped_risers_(3) {
#if VISUALIZE
  visualizer_update_ = false;
  visualizer_thread_.reset(new boost::thread(boost::bind(&WalrusStairDetector::visualize, this)));
  previous_plane_visual_count_ = 0;
  previous_stair_count_ = 0;
#endif
}
WalrusStairDetector::~WalrusStairDetector() {
  shutdown_ = true;
#if VISUALIZE
  visualizer_thread_->join();
#endif
}
void WalrusStairDetector::detect(const PointCloud::ConstPtr& original_cloud, const Eigen::Vector3f& vertical_estimate, std::vector<StairModel>* stairs) {
  stairs->clear();
#if VISUALIZE
  boost::mutex::scoped_lock lock(visualizer_mutex_);
  visualizer_update_ = true;
  plane_visual_ = std::vector<DetectedPlane::Ptr>();
  model_ = StairModel();
#endif

  ROS_INFO("Cloud: width = %d, height = %d, size = %ld\n", original_cloud->width, original_cloud->height, original_cloud->points.size());

  MultiTimer timer;

  Eigen::Vector3f vertical = vertical_estimate;

  // Downsize the pointcloud
  PointCloud::Ptr downsized_cloud (new PointCloud);
  downsizePointCloud(original_cloud, downsized_cloud);
  timer.end("downsize");

  // Remove invalid points
  pcl::IndicesPtr filter_indices(new std::vector <int>);
  pcl::removeNaNFromPointCloud<PointCloud::PointType>(*downsized_cloud, *filter_indices);
  timer.end("filter");

  // Compute the normal of all points
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  computeNormals(downsized_cloud, filter_indices, normals);
  timer.end("normals");

  // Grow points to form planes
  std::vector<pcl::PointIndices> clusters;
  regionGrow(downsized_cloud, filter_indices, normals, &clusters);
  timer.end("region grow");


  std::vector<DetectedPlane::Ptr> planes;
  int plane_id = 0;
  BOOST_FOREACH(const pcl::PointIndices& cluster, clusters) {
    DetectedPlane::Ptr plane(new DetectedPlane(plane_id++));

    bool plane_model_success = computePlaneModel(downsized_cloud, cluster, plane->coefficients, plane->inliers);
    plane->normal[0] = plane->coefficients->values[0];
    plane->normal[1] = plane->coefficients->values[1];
    plane->normal[2] = plane->coefficients->values[2];
    plane->normal.normalize();
    timer.end("plane_model");

    if(!plane_model_success)
      continue;

    // Project all points into the computed plane
    projectPoints(downsized_cloud, plane->inliers, plane->coefficients, plane->cluster_projected);
    timer.end("project to planes");

    // Compute the bounding points of the plane
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud(plane->cluster_projected);
    chull.reconstruct(*plane->cluster_hull);
    timer.end("convex hull");

    computePlaneOrientation(plane, vertical);

    if(plane->orientation == DetectedPlane::Vertical) {
      computeVerticalPlaneSize(plane, vertical);
    }

    guessPlaneType(plane);

    timer.end("plane properties");


    planes.push_back(plane);
    timer.skip();
  }
#if VISUALIZE
  plane_visual_ = planes;
#endif

  StairModel model;
  model.vertical = vertical;

  Eigen::Vector3f stair_orientation;
  bool stair_orientation_result = computeStairOrientation(planes, vertical, &stair_orientation);
  timer.end("compute stair direction");

  if(!stair_orientation_result) {
    ROS_WARN("Could not compute riser orientation");
    return;
  }

  ROS_INFO("Found stair orientation: %f, %f, %f", stair_orientation[0], stair_orientation[1], stair_orientation[2]);
  model.direction = stair_orientation;

  DistanceModel riser_spacing;
  int riser_start_index;
  bool riser_spacing_result = computeRiserSpacing(planes, stair_orientation, &riser_spacing, &riser_start_index);
  timer.end("compute riser spacing");

  if(!riser_spacing_result) {
    ROS_WARN("Could not compute riser spacing");
    return;
  }
  model.run = riser_spacing.spacing;

  SlopeModel stair_rise;
  bool rise_result = computeRise(planes, vertical, stair_orientation, riser_spacing, riser_start_index, &stair_rise);
  timer.end("compute stair rise");

  if(!rise_result) {
    ROS_WARN("Could not compute stair rise");
    return;
  }
  model.rise = stair_rise.slope * model.run;

  model.num_stairs = 10;
  model.width = 0.7;

  model.origin = planes[riser_start_index]->vertical_center + model.vertical * (stair_rise.offset - model.rise/2);

  std::cout << "Origin: " << model.origin[0] << ", " << model.origin[1] << ", " << model.origin[2] << std::endl;
  std::cout << "Direction: " << model.direction[0] << ", " << model.direction[1] << ", " << model.direction[2] << std::endl;

  stairs->push_back(model);
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
  pcl::NormalEstimation<PointCloud::PointType, pcl::Normal> normal_estimator;
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
  region_growing_cloud_visual_ = reg.getColoredCloud();
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
  else if(vertical.dot(plane->normal) < cos((90 - 16) * M_PI / 180)) {
    plane->orientation = DetectedPlane::Vertical;
  }
}
void WalrusStairDetector::computeVerticalPlaneSize(DetectedPlane::Ptr plane, const Eigen::Vector3f& vertical) {
  Eigen::Vector3f horizontal = vertical.cross(plane->normal);

  // Pick a random reference point
  Eigen::Vector3f p0 = plane->cluster_hull->at(0).getVector3fMap();

  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  BOOST_FOREACH(const pcl::PointXYZ& abs_point, *plane->cluster_hull) {
    Eigen::Vector3f p = abs_point.getVector3fMap() - p0;
    double x = p.dot(horizontal);
    double y = p.dot(vertical);
    if(x > max_x)
      max_x = x;
    if(x < min_x)
      min_x = x;
    if(y > max_y)
      max_y = y;
    if(y < min_y)
      min_y = y;
  }
  plane->vertical_width = max_x - min_x;
  plane->vertical_height = max_y - min_y;

  double center_x = (min_x + max_x) / 2;
  double center_y = (min_y + max_y) / 2;
  plane->vertical_center = p0 + horizontal * center_x + vertical * center_y;
}


void WalrusStairDetector::guessPlaneType(DetectedPlane::Ptr plane) {
  if(plane->orientation == DetectedPlane::Vertical) {
    plane->is_tread = false;
    if(plane->vertical_width < plane->vertical_height * 1.25)
      plane->is_riser = false;
    if(plane->vertical_height > max_riser_height_)
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
      potential_risers.push_back(plane);
    }
  }
  Eigen::Vector3f riser_orientation;
  ParallelPlaneRansacModel model_description;
  Ransac<DetectedPlane::Ptr, Eigen::Vector3f> ransac(&model_description);
  ransac.setInput(&potential_risers);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, &riser_orientation);
  if(result) {
    for(int i = 0; i < potential_risers.size(); ++i) {
      if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
	potential_risers[i]->is_tread = false;
      }
    }
  }
  // compute portion of the riser orientation perp to the vertical
  *model = riser_orientation - riser_orientation.dot(vertical) * vertical;
  return result;
}

bool WalrusStairDetector::computeRiserSpacing(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& stair_orientation, DistanceModel* model, int* start_index) {
  std::vector<DetectedPlane::Ptr> potential_risers;
  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(plane->is_riser || indeterminate(plane->is_riser)) {
      potential_risers.push_back(plane);
    }
  }

  if(potential_risers.size() < 2) {
    ROS_WARN("Not enough planes to compute riser spacing from");
    return false;
  }

  // pick an arbitrary plane to compute relative distances off of
  DetectedPlane::Ptr& p0 = potential_risers[0];

  // compute distance between planes along stair direction
  std::vector<double> potential_risers_dist;
  for(int i = 0; i < potential_risers.size(); ++i) {
    const DetectedPlane::Ptr& p = potential_risers[i];
    Eigen::Vector3f vec_to_p =  p->vertical_center - p0->vertical_center;
    double dist = stair_orientation.dot(vec_to_p);
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
      if(dist > max_riser_spacing_ * max_skipped_risers_)
	break;
      dists.push_back(dist);
    }
  }

  Histogram hist(min_riser_spacing_, max_riser_spacing_, 0.02, 100);
  BOOST_FOREACH(double dist, dists) {
    if(dist < max_riser_spacing_)
      hist.add(dist);
    for(int i = 2; i <= max_skipped_risers_; ++i) {
      double new_dist = dist / i;
      if(new_dist > min_riser_spacing_ && new_dist < max_riser_spacing_)
	hist.add(dist, 1.0 / i / i / i);
    }
  }
  //hist.print();


  DistanceRansacModel model_description(hist.largestBucket(), 0.05);
  Ransac<double, DistanceModel> ransac(&model_description);
  ransac.setInput(&potential_risers_dist);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, model);
  if(result) {
    for(int i = 0; i < potential_risers.size(); ++i) {
      if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
	potential_risers[i]->is_tread = false;
      }
    }
    for(std::multimap<int, int>::iterator itr = model->groups.begin(); itr != model->groups.end(); ++itr) {
      DetectedPlane::Ptr& plane = potential_risers[inliers[itr->second]];
      plane->stair_group = itr->first;
      if(itr->first == 0)
	*start_index = plane->id;
      itr->second = plane->id;
    }
  }
  return result;
}



bool WalrusStairDetector::computeRise(std::vector<DetectedPlane::Ptr>& planes, const Eigen::Vector3f& vertical, const Eigen::Vector3f& stair_orientation, const DistanceModel& riser_spacing, int riser_start_index, SlopeModel* model) {
  DetectedPlane::Ptr start_plane = planes[riser_start_index];

  std::vector<std::pair<double, double> > points;
  std::vector<DetectedPlane::Ptr> potential_risers;
  BOOST_FOREACH(DetectedPlane::Ptr& plane, planes) {
    if(plane->is_riser || indeterminate(plane->is_riser)) {
      potential_risers.push_back(plane);
      Eigen::Vector3f vect_to_p = plane->vertical_center - start_plane->vertical_center;
      points.push_back(std::make_pair(vect_to_p.dot(stair_orientation), vect_to_p.dot(vertical)));
    }
  }

  SlopeRansacModel model_description;
  Ransac<std::pair<double, double>, SlopeModel> ransac(&model_description);
  ransac.setInput(&points);
  ransac.setMaxIterations(30);
  std::vector<int> inliers;
  bool result = ransac.estimate(&inliers, model);
  if(result) {
    for(int i = 0; i < potential_risers.size(); ++i) {
      if(std::find(inliers.begin(), inliers.end(), i) == inliers.end()) {
	potential_risers[i]->is_tread = false;
	potential_risers[i]->flag = true;
      }
      else {
	potential_risers[i]->is_tread = true;
      }
    }
  }
  return result;


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
	for(int i = 0; i < previous_plane_visual_count_; ++i)
	  visualizer_->removeShape(visName("plane", i));
	// Add/update shapes
	for(int i = 0; i < plane_visual_.size(); ++i) {
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
	}
	previous_plane_visual_count_ = plane_visual_.size();
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

	// Remove old shapes
	for(int i = 0; i < previous_stair_count_; ++i) {
	  visualizer_->removeShape(visName("riser", i));
	  visualizer_->removeShape(visName("tread", i));
	}

	Eigen::Vector3f horizontal = model_.vertical.cross(model_.direction);
	for(int i = 0; i < model_.num_stairs; ++i) {
	  Eigen::Vector3f base = model_.origin + model_.vertical * model_.rise * i + model_.direction * model_.run * i;

	  pcl::PointCloud<pcl::PointXYZ>::Ptr riser_points(new pcl::PointCloud<pcl::PointXYZ>());
	  riser_points->resize(4);
	  riser_points->at(0).getVector3fMap() = base + horizontal * model_.width/2 + model_.vertical*model_.rise;
	  riser_points->at(1).getVector3fMap() = base - horizontal * model_.width/2 + model_.vertical*model_.rise;
	  riser_points->at(2).getVector3fMap() = base - horizontal * model_.width/2;
	  riser_points->at(3).getVector3fMap() = base + horizontal * model_.width/2;
	  visualizer_->addPolygon<pcl::PointXYZ>(riser_points, CYAN[0], CYAN[1], CYAN[2], visName("riser", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("riser", i));


	  pcl::PointCloud<pcl::PointXYZ>::Ptr tread_points(new pcl::PointCloud<pcl::PointXYZ>());
	  tread_points->resize(4);
	  tread_points->at(0).getVector3fMap() = base + model_.vertical*model_.rise + horizontal * model_.width/2 + model_.direction*model_.run;
	  tread_points->at(1).getVector3fMap() = base + model_.vertical*model_.rise - horizontal * model_.width/2 + model_.direction*model_.run;
	  tread_points->at(2).getVector3fMap() = base + model_.vertical*model_.rise - horizontal * model_.width/2;
	  tread_points->at(3).getVector3fMap() = base + model_.vertical*model_.rise + horizontal * model_.width/2;
	  visualizer_->addPolygon<pcl::PointXYZ>(tread_points, PINK[0], PINK[1], PINK[2], visName("tread", i));
	  visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, visName("tread", i));
	}
	previous_stair_count_ = model_.num_stairs;


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