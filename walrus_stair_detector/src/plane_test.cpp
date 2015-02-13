#include <limits>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/foreach.hpp>
#include <pcl/filters/project_inliers.h>
#include <boost/assign/list_of.hpp>
#include <queue>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

  typedef struct {
    pcl::ModelCoefficients::Ptr coefficients;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
    pcl::PointXYZ plane_center;
    Eigen::Vector3f plane_normal;
  } DetectedPlane;



// From http://stackoverflow.com/questions/10847007/using-the-gaussian-probability-density-function-in-c
double normal_pdf(double x, double m, double s)
{
  static const double inv_sqrt_2pi = 0.3989422804014327;
  double a = (x - m) / s;
  return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}


class Histogram {
public:
  Histogram(double min, double max, double sigma, int num_buckets) : min_(min), max_(max), sigma_(sigma), num_buckets_(num_buckets) {
    buckets.resize(num_buckets);
    bucket_size_ = (max_ - min_) / num_buckets;
  }
  void add(double value, double weight = 1.0) {
    for(int i = 0; i < num_buckets_; ++i) {
      buckets[i] += normal_pdf(valueAtIndex(i), value, sigma_) * weight;
    }
  }
  double largestBucket() {
    double max = -1;
    int max_index = -1;
    for(int i = 0; i < num_buckets_; ++i) {
      if(buckets[i] > max) {
	max = buckets[i];
	max_index = i;
      }
    }
    return valueAtIndex(max_index);
  }
  void print() {
    double max = 0;
    for(int i = 0; i < num_buckets_; ++i) {
      if(buckets[i] > max)
	max = buckets[i];
    }
    std::cerr << "Histogram:" << std::endl;
    std::cerr.precision(6);
    for(int i = 0; i < num_buckets_; ++ i) {
      std::cerr << valueAtIndex(i) << ": ";
      for(int j = 0; j < 40 * buckets[i] / max; ++j)
	std::cerr << "*";
      std::cerr << "        " << buckets[i] << std::endl;
    }
    std::cerr.unsetf ( std::ios::floatfield );
  }
private:
  double valueAtIndex(int i) {
    return min_ + i * bucket_size_ + bucket_size_ / 2;
  }

  double min_;
  double max_;
  double sigma_;
  double bucket_size_;
  int num_buckets_;
  std::vector<double> buckets;
};


void selectRandomIndices(int num_to_select, int sample_size, std::vector<int>* items) {
  items->clear();
  if(sample_size < num_to_select)
    return;
  for(int i = 0; i < num_to_select; ++i) {
    int index;
    do {
      index = rand() % sample_size;
    } while(std::find(items->begin(), items->end(), index) != items->end());
    items->push_back(index);
  }
}
void alignedPlaneRansac(const std::vector<boost::shared_ptr<DetectedPlane> >& planes, std::vector<int>* inliers, Eigen::Vector3f* model) {
  int max_iterations = 30;
  int items_to_fit = 2;

  double best_fit = std::numeric_limits<double>::infinity();

  int iteration = 0;
  while(iteration < max_iterations) {
    std::vector<int> selected_items;
    selectRandomIndices(items_to_fit, planes.size(), &selected_items);

    Eigen::Vector3f qa = planes[selected_items[0]]->plane_normal;
    Eigen::Vector3f qb =planes[selected_items[1]]->plane_normal;

    if((-qa).dot(qb) > qa.dot(qb)){
      qa = -qa;
    }

    Eigen::Vector3f estimated_model = (qa + qb)/2;

    std::vector<int> estimate_inliers;
    for(int i = 0; i < planes.size(); ++i) {
      if(std::find(selected_items.begin(), selected_items.end(), i) != selected_items.end())
	continue;
      const boost::shared_ptr<DetectedPlane>& plane = planes[i];
      if(plane->plane_normal.dot(estimated_model) > 0.92)
	estimate_inliers.push_back(i);
    }
    std::vector<int> all_inliers;
    all_inliers.insert(all_inliers.end(), selected_items.begin(), selected_items.end());
    all_inliers.insert(all_inliers.end(), estimate_inliers.begin(), estimate_inliers.end());
    if(all_inliers.size() > planes.size()/2) {
      Eigen::Vector3f new_model;
      for(int i = 0; i < all_inliers.size(); ++i) {
	new_model += planes[all_inliers[i]]->plane_normal;
      }
      new_model /= all_inliers.size();
      new_model.normalize();

      double fit = 0;
      for(int i = 0; i < all_inliers.size(); ++i) {
	fit += M_PI/2 - new_model.dot(planes[all_inliers[i]]->plane_normal);
      }
      fit /= all_inliers.size();
      if(fit < best_fit) {
	*model = new_model;
	*inliers = all_inliers;
	best_fit = fit;
	std::cerr << "Better fit: " << fit << " (" << iteration << ")[" << all_inliers.size() << "]" <<  std::endl;
      }
    }

    ++iteration;
  }
}


void planeDistanceRansac(const std::multimap<double, int> index_with_dist, double initial_estimate, std::vector<int>* inliers, double* model) {
  int max_iterations = 30;
  int items_to_fit = 1;

  double best_fit = std::numeric_limits<double>::infinity();

  int iteration = 0;
  while(iteration < max_iterations) {
    std::vector<int> selected_items;
    selectRandomIndices(items_to_fit, index_with_dist.size(), &selected_items);

    std::multimap<double, int>::const_iterator selected_point_itr = index_with_dist.begin();
    std::advance(selected_point_itr, selected_items[0]);
    double estimated_start = selected_point_itr->first;

    std::multimap<double, int> estimate_inliers;
    std::multimap<double, int>::const_iterator estimate_inliers_itr = index_with_dist.begin();
    for(;estimate_inliers_itr != index_with_dist.end(); ++estimate_inliers_itr) {
      double offset = fmod(std::fabs(estimate_inliers_itr->first - estimated_start), initial_estimate);
      if(offset < 0.03 || initial_estimate - offset < 0.03)
	estimate_inliers.insert(*estimate_inliers_itr);
    }

    double new_model = initial_estimate;
    // TODO: actually calculate new model

    if(estimate_inliers.size() > index_with_dist.size() * 0.3) {
      std::vector<int> all_inliers;
      double fit = 0;
      std::multimap<double, int>::const_iterator final_inliers_itr = estimate_inliers.begin();
      for(;final_inliers_itr != estimate_inliers.end(); ++final_inliers_itr) {
	double offset = std::fmod(std::fabs(final_inliers_itr->first - estimated_start), new_model);
	if(offset > new_model / 2)
	  fit += new_model - offset;
	else
	  fit += offset;
	all_inliers.push_back(final_inliers_itr->second);
      }
      fit /= all_inliers.size();
      fit *= pow(1.02, (index_with_dist.size() - all_inliers.size()));
      if(fit < best_fit) {
	*model = new_model;
	*inliers = all_inliers;
	best_fit = fit;
	std::cerr << "Better fit: " << fit << " (" << iteration << ")[" << all_inliers.size() << "] = " << new_model <<  std::endl;
      }
    }

    ++iteration;
  }
}
void planeSpacing(const std::vector<boost::shared_ptr<DetectedPlane> >& planes, const Eigen::Vector3f& stair_direction, std::vector<int>* inliers, double* model) {
  double min_spacing = 0.15;
  double max_spacing = 0.4;
  if(planes.size() == 0){
    std::cerr << "No planes: planeSpacing" << std::endl;
    return;
  }
  boost::shared_ptr<DetectedPlane> p0 = planes[0];// pick an arbitrary plane to compute relative distances off of

  // compute distance between planes along stair direction
  std::multimap<double, int> inliers_with_dist;
  for(int i = 0; i < planes.size(); ++i) {
    const boost::shared_ptr<DetectedPlane>& plane = planes[i];
    Eigen::Vector3f vec_to_p =  plane->plane_center.getVector3fMap() - p0->plane_center.getVector3fMap();
    double dist = stair_direction.dot(vec_to_p);
    inliers_with_dist.insert(std::pair<double, int>(dist, i));
  }

  std::vector<double> dists;
  for(std::multimap<double, int>::const_iterator itr = inliers_with_dist.begin(); itr != inliers_with_dist.end(); ++itr) {
    std::multimap<double, int>::const_iterator itr2 = itr;
    ++itr2;
    for(; itr2 != inliers_with_dist.end(); ++itr2) {
      Eigen::Vector3f vec = planes[itr2->second]->plane_center.getVector3fMap() - planes[itr->second]->plane_center.getVector3fMap();
      double dist = stair_direction.dot(vec);
      if(dist > max_spacing*3)
	break;
      dists.push_back(dist);
    }
  }

  Histogram hist(min_spacing, max_spacing, 0.02, 100);
  BOOST_FOREACH(double dist, dists) {
    if(dist < max_spacing)
      hist.add(dist);
    for(int i = 2; i < 4; ++i) {
      double new_dist = dist / i;
      if(new_dist > min_spacing && new_dist < max_spacing)
	hist.add(dist, 1.0 / i);
    }
  }
  //hist.print();
  std::cerr << "Hist Distance: " << hist.largestBucket() << std::endl;

  planeDistanceRansac(inliers_with_dist, hist.largestBucket(), inliers, model);
}

void planeSlopeRansac(const std::vector<boost::shared_ptr<DetectedPlane> >& planes, Eigen::Vector3f& vertical, Eigen::Vector3f& stair_direction, std::vector<int>* inliers, double* model) {
  if(planes.size() == 0){
    std::cerr << "No planes: planeVerticalRansac" << std::endl;
    return;
  }
  int max_iterations = 30;
  int items_to_fit = 2;

  double best_fit = std::numeric_limits<double>::infinity();

  int iteration = 0;
  while(iteration < max_iterations) {
    std::vector<int> selected_items;
    selectRandomIndices(items_to_fit, planes.size(), &selected_items);

    Eigen::Vector3f qa = planes[selected_items[0]]->plane_center.getVector3fMap();
    Eigen::Vector3f qb =planes[selected_items[1]]->plane_center.getVector3fMap();

    double estimated_model = (qa - qb).dot(vertical) / (qa - qb).dot(stair_direction);

    std::vector<int> estimate_inliers;
    for(int i = 0; i < planes.size(); ++i) {
      if(std::find(selected_items.begin(), selected_items.end(), i) != selected_items.end())
	continue;
      const boost::shared_ptr<DetectedPlane> plane = planes[i];
      Eigen::Vector3f q = plane->plane_center.getVector3fMap();
      double offset = std::fabs((qa - q).dot(vertical) - (qa - q).dot(stair_direction) * estimated_model);
      if(offset < 0.2)
	estimate_inliers.push_back(i);
    }
    std::vector<int> all_inliers;
    all_inliers.insert(all_inliers.end(), selected_items.begin(), selected_items.end());
    all_inliers.insert(all_inliers.end(), estimate_inliers.begin(), estimate_inliers.end());
    if(all_inliers.size() > planes.size() / 2) {
      double new_model = estimated_model;
      // TODO: actually calculate new model

      double fit = 0;
      for(int i = 0; i < all_inliers.size(); ++i) {
	Eigen::Vector3f q = planes[all_inliers[i]]->plane_center.getVector3fMap();
	double offset = std::fabs((qa - q).dot(vertical) - (qa - q).dot(stair_direction) * new_model);
	fit += offset;
      }
      fit /= all_inliers.size();
      fit *= pow(1.05, (planes.size() - all_inliers.size()));

      if(fit < best_fit) {
	*model = new_model;
	*inliers = all_inliers;
	best_fit = fit;
	std::cerr << "Better fit: " << fit << " (" << iteration << ")[" << all_inliers.size() << "] = " << new_model <<  std::endl;
      }
    }

    ++iteration;
  }
}



int main (int argc, char** argv)
{
  if(argc != 2) {
    printf("Expected a single argument");
    return -1;
  }
  // load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_in);

  std::clock_t start = std::clock();

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*cloud);

  std::clock_t voxel = std::clock();

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  std::clock_t filter = std::clock();

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod (tree);
  normal_estimator.setInputCloud (cloud);
  normal_estimator.setKSearch (50);
  normal_estimator.compute (*normals);

  std::clock_t normal = std::clock();

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize (50);
  reg.setMaxClusterSize (1000000);
  reg.setSearchMethod (tree);
  reg.setNumberOfNeighbours (30);
  reg.setInputCloud (cloud);
  reg.setIndices (indices);
  reg.setInputNormals (normals);
  reg.setSmoothnessThreshold (4.0 / 180.0 * M_PI);
  reg.setCurvatureThreshold (1.0);

  std::vector <pcl::PointIndices> clusters;
  reg.extract (clusters);

  std::clock_t region_grow = std::clock();

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
  pcl::visualization::PCLVisualizer viewer ("Cluster viewer");
  //viewer.addPointCloud(colored_cloud, "cloud");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer.addCoordinateSystem (1.0);
  viewer.initCameraParameters();
  viewer.setCameraPosition(0, 0, -1, 0, -1, 0);

  Eigen::Vector3f vertical;
  vertical[0] = 0;
  vertical[1] = -1;
  vertical[2] = 0;


  std::vector<boost::shared_ptr<DetectedPlane> > horizontal_planes;
  std::vector<boost::shared_ptr<DetectedPlane> > potential_riser_planes;
  std::vector<boost::shared_ptr<DetectedPlane> > other_vertical_planes;

  int plane_id = 0;
  BOOST_FOREACH(const pcl::PointIndices& cluster, clusters) {
    pcl::IndicesPtr indices (new std::vector <int>);
    *indices = cluster.indices;

    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    // Create the segmentation object.
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setIndices(indices);
    // Configure the object to look for a plane.
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    // Use RANSAC method.
    segmentation.setMethodType(pcl::SAC_RANSAC);
    // Set the maximum allowed distance to the model.
    segmentation.setDistanceThreshold(0.01);
    // Enable model coefficient refinement (optional).
    segmentation.setOptimizeCoefficients(true);

    pcl::PointIndices inlierIndices;
    segmentation.segment(inlierIndices, *coefficients);

    if (inlierIndices.indices.size() == 0)
      std::cout << "Could not find any points that fitted the plane model." << std::endl;
    else
      {
	/*std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		  << coefficients->values[1] << " "
		  << coefficients->values[2] << " "
		  << coefficients->values[3] << std::endl;*/

	pcl::IndicesPtr inlier_indices (new std::vector <int>);
	*inlier_indices = inlierIndices.indices;


	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setInputCloud(cloud);
	proj.setIndices(inlier_indices);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cluster_projected);

	pcl::PointXYZ plane_center;
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*cluster_projected, centroid);
	plane_center.x = centroid[0];
	plane_center.y = centroid[1];
	plane_center.z = centroid[2];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cluster_projected);

	chull.reconstruct(*cloud_hull);

	Eigen::Vector3f n, u, v;

	n[0] = coefficients->values[0];
	n[1] = coefficients->values[1];
	n[2] = coefficients->values[2];
	n.normalize();

	u[0] = cloud_hull->at(0).x - plane_center.x;
	u[1] = cloud_hull->at(0).y - plane_center.y;
	u[2] = cloud_hull->at(0).z - plane_center.z;
	u.normalize();

	v = u.cross(n);
	v.normalize();

	boost::shared_ptr<DetectedPlane> plane(new DetectedPlane());
	plane->cloud_hull = cloud_hull;
	plane->plane_center = plane_center;
	plane->plane_normal = n;
	plane->coefficients = coefficients;



	if(std::fabs(vertical.dot(n) - 1) < 0.1) {
	  std::stringstream plane_ss;
	  plane_ss << "plane_" << plane_id;
	  viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 1, 0, 0, plane_ss.str());
	  horizontal_planes.push_back(plane);
	}

	else if(std::fabs(vertical.dot(n)) < 0.4) {
	  Eigen::Vector3f horizontal = vertical.cross(n);
	  double max_width = 0;
	  double max_height = 0;
	  BOOST_FOREACH(const pcl::PointXYZ& abs_point, *cloud_hull) {
	    Eigen::Vector3f p = abs_point.getVector3fMap() - plane_center.getVector3fMap();
	    Eigen::Vector3f p_vertical = p.cwiseProduct(vertical);
	    Eigen::Vector3f p_horizontal = p.cwiseProduct(horizontal);

	    double width = p_horizontal.norm();
	    double height = p_vertical.norm();
	    if(width > max_width)
	      max_width = width;
	    if(height > max_height)
	      max_height = height;
	  }
	  if(max_width > max_height * 1.25)
	    potential_riser_planes.push_back(plane);
	  else {
	    other_vertical_planes.push_back(plane);
	    std::stringstream plane_ss;
	    plane_ss << "plane_" << plane_id;
	    viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 0, 1, 1, plane_ss.str());
	  }
	}
	else {
	  std::stringstream plane_ss;
	  plane_ss << "plane_" << plane_id;
	  viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 0, 1, 0, plane_ss.str());
	}

	++plane_id;
      }
  }

  std::clock_t model_and_hull = std::clock();


  std::vector<boost::shared_ptr<DetectedPlane> > riser_planes;
  std::vector<int> inliers;
  Eigen::Vector3f model;
  alignedPlaneRansac(potential_riser_planes, &inliers, &model);

  for(int i = 0; i < potential_riser_planes.size(); ++i) {
    std::stringstream plane_ss;
    plane_ss << "asdfvplane_" << i;
    if(std::find(inliers.begin(), inliers.end(), i) != inliers.end()) {
      riser_planes.push_back(potential_riser_planes[i]);
    }
    else{
      viewer.addPolygon<pcl::PointXYZ>(potential_riser_planes[i]->cloud_hull, 1, 1, 0, plane_ss.str());
    }
  }

  double spacing_model;
  std::vector<int> spacing_inliers;
  std::vector<boost::shared_ptr<DetectedPlane> > actual_riser_planes;
  planeSpacing(riser_planes, model, &spacing_inliers, &spacing_model);

  for(int i = 0; i < riser_planes.size(); ++i) {
    std::stringstream plane_ss;
    plane_ss << "asplane_" << i;
    if(std::find(spacing_inliers.begin(), spacing_inliers.end(), i) != spacing_inliers.end()) {
      viewer.addPolygon<pcl::PointXYZ>(riser_planes[i]->cloud_hull, 0, 0, 1, plane_ss.str());
      actual_riser_planes.push_back(riser_planes[i]);
    }
    else{
      viewer.addPolygon<pcl::PointXYZ>(riser_planes[i]->cloud_hull, 1, 0, 0.5, plane_ss.str());
    }
  }
  std::cerr << "Horizontal Distance: " << spacing_model << std::endl;


  std::vector<int> vertical_inliers;
  double slope_model;
  planeSlopeRansac(actual_riser_planes, vertical, model, &vertical_inliers, &slope_model);

  for(int i = 0; i < actual_riser_planes.size(); ++i) {
    std::stringstream plane_ss;
    plane_ss << "adddplane_" << i;
    if(std::find(vertical_inliers.begin(), vertical_inliers.end(), i) != vertical_inliers.end()) {
      viewer.addPolygon<pcl::PointXYZ>(actual_riser_planes[i]->cloud_hull, 0, 0, 1, plane_ss.str());
    }
    else{
      viewer.addPolygon<pcl::PointXYZ>(actual_riser_planes[i]->cloud_hull, 0.5, 1, 0.5, plane_ss.str());
    }
  }
  std::cerr << "vertical Distance: " << slope_model * spacing_model << std::endl;



  std::clock_t vertical_ransac = std::clock();

  std::cout << "Voxel: " << (voxel - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Filter: " << (filter - voxel) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Normal: " << (normal - filter) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Region Grow: " << (region_grow - normal) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Model and Hull: " << (model_and_hull - region_grow) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Vertical Ransac: " << (vertical_ransac - model_and_hull) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Total Time: " << (vertical_ransac - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;



  while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
