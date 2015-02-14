#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl/filters/voxel_grid.h>
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

namespace walrus_stair_detector {

  WalrusStairDetector::WalrusStairDetector() : shutdown_(false),
					       min_riser_height_(0.1), max_riser_height_(0.35),
					       min_tread_length_(0.2), max_tread_length_(0.4) {
#if VISUALIZE
  visualizer_update_ = false;
  visualizer_thread_.reset(new boost::thread(boost::bind(&WalrusStairDetector::visualize, this)));
  previous_plane_visual_count_ = 0;
#endif
}
WalrusStairDetector::~WalrusStairDetector() {
  shutdown_ = true;
#if VISUALIZE
  visualizer_thread_->join();
#endif
}
void WalrusStairDetector::detect(const PointCloud::ConstPtr& original_cloud) {
#if VISUALIZE
  boost::mutex::scoped_lock lock(visualizer_mutex_);
  visualizer_update_ = true;
#endif

  ROS_INFO("Cloud: width = %d, height = %d, size = %ld\n", original_cloud->width, original_cloud->height, original_cloud->points.size());
  MultiTimer timer;

  Eigen::Vector3f vertical;
  vertical[0] = 0;
  vertical[1] = -1;
  vertical[2] = 0;

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


#if VISUALIZE
  plane_visual_.clear();
#endif
  DetectedPlaneSet planes;
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

    // Compute the centroid of the plane
    Eigen::Vector4f centroid_4;
    pcl::compute3DCentroid(*plane->cluster_projected, centroid_4);
    plane->centroid = centroid_4.block<3, 1>(0, 0);
    timer.end("compute centroid");

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


    planes.insert(plane);
#if VISUALIZE
    plane_visual_.push_back(plane);
#endif
    timer.skip();
  }

  Ransac<DetectedPlane::Ptr, double> ransac(NULL);
  std::vector<int> inliers;
  double model;
  ransac.estimate(&inliers, &model);


  std::pair<DetectedPlaneSetById::iterator, DetectedPlaneSetById::iterator> itrs = planes.get<DetectedPlane::ById>().equal_range(1);
  for(DetectedPlaneSetById::iterator itr = itrs.first; itr != itrs.second; itr++)
    ROS_INFO("Plane %d", (*itr)->id);



  timer.print();
}

void WalrusStairDetector::downsizePointCloud(const PointCloud::ConstPtr& cloud, PointCloud::Ptr out) {
  pcl::VoxelGrid<PointCloud::PointType> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(0.03f, 0.03f, 0.03f);
  voxel_grid.filter(*out);
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
  double max_width = 0;
  double max_height = 0;
  BOOST_FOREACH(const pcl::PointXYZ& abs_point, *plane->cluster_hull) {
    Eigen::Vector3f p = abs_point.getVector3fMap() - plane->centroid; // point location relative to centroid
    // compute horizontal and vertical size (*2 is because centroid is in middle)
    double width = p.cwiseProduct(horizontal).norm() * 2;
    double height = p.cwiseProduct(vertical).norm() * 2;
    if(width > max_width)
      max_width = width;
    if(height > max_height)
      max_height = height;
  }
  plane->vertical_width = max_width;
  plane->vertical_height = max_height;
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
	// Remove old shapes
	for(int i = 0; i < previous_plane_visual_count_; ++i)
	  visualizer_->removeShape(visName("plane", i));
	// Add/update shapes
	for(int i = 0; i < plane_visual_.size(); ++i) {
	  DetectedPlane::Ptr plane = plane_visual_[i];
	  Eigen::Vector3f color;
	  if(plane->is_riser || indeterminate(plane->is_riser))
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
