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

  typedef struct {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull;
    pcl::PointXYZ plane_center;
    Eigen::Vector3f plane_normal;
  } DetectedPlane;



int main (int argc, char** argv)
{
  if(argc != 2) {
    printf("Expected a single argument");
    return -1;
  }
  // load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud_in);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud_in);
  sor.setLeafSize (0.03f, 0.03f, 0.03f);
  sor.filter (*cloud);

  pcl::IndicesPtr indices (new std::vector <int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  std::clock_t start = std::clock();

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


  std::vector<boost::shared_ptr<DetectedPlane> > horizontal_planes;

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
	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		  << coefficients->values[1] << " "
		  << coefficients->values[2] << " "
		  << coefficients->values[3] << std::endl;

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

	Eigen::Vector3f vertical;
	vertical[0] = 0;
	vertical[1] = -1;
	vertical[2] = 0;
	std::cerr << vertical.dot(n) << std::endl;

	pcl::PointCloud<pcl::PointXYZ> points2d;
	BOOST_FOREACH(const pcl::PointXYZ& abs_point, *cloud_hull) {
	  Eigen::Vector3f p;// = abs_point.getVector3fMap() - plane_center.getVector3fMap();
	  p[0] = abs_point.x - plane_center.x;
	  p[1] = abs_point.y - plane_center.y;
	  p[2] = abs_point.z - plane_center.z;
	  pcl::PointXYZ point2d;
	  point2d.y = (u[1] * p[0] - u[0] * p[1]) / (u[1] * v[0] - u[0] * v[1]);
	  point2d.x = (p[0] - v[0] * point2d.y) / u[0];// TODO: Divide by zero?
	  points2d.push_back(point2d);
	}

	float min_area = std::numeric_limits<float>::infinity();
	float min_area_pts[4][2];

	for(int i = 0; i < points2d.size() - 1; ++i) {
	  double theta = atan2(points2d[i + 1].y - points2d[i].y, points2d[i + 1].x - points2d[i].x);

	  bool first = true;
	  float pts[4]; // minx, miny, maxx, maxy
	  BOOST_FOREACH(const pcl::PointXYZ& p_orig, points2d) {
	    float x = p_orig.x * cos(theta) - p_orig.y * sin(theta);
	    float y = p_orig.x * sin(theta) + p_orig.y * cos(theta);

	    if(first || x < pts[0])
	      pts[0] = x;
	    if(first || y < pts[1])
	      pts[1] = y;

	    if(first || x > pts[2])
	      pts[2] = x;
	    if(first || y > pts[3])
	      pts[3] = y;

	    first = false;
	  }

	  double area = (pts[2] - pts[0]) * (pts[3] - pts[1]);
	  if(area < min_area) {
	    min_area = area;
	    for(int j = 0; j < 2; ++j) {
	      for(int k = 0; k < 2; ++k) {
		min_area_pts[j+2*k][0] = pts[j*2] * cos(-theta) - pts[j*2+1] * sin(-theta);
		min_area_pts[j+2*k][1] = pts[k*2] * sin(-theta) + pts[k*2+1] * cos(-theta);
	      }
	    }
	    std::cerr << min_area << std::endl;
	  }
	}


	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(points2d, min_pt, max_pt);

	pcl::PointCloud<pcl::PointXYZ>::Ptr rect2d (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<int> is = boost::assign::list_of(0)(1)(3)(2);
	BOOST_FOREACH(int i, is) {
	  pcl::PointXYZ minmin_pt;
	  minmin_pt.x = min_area_pts[i][0] * u[0] + min_area_pts[i][1] * v[0] + plane_center.x;
	  minmin_pt.y = min_area_pts[i][0] * u[1] + min_area_pts[i][1] * v[1] + plane_center.y;
	  minmin_pt.z = min_area_pts[i][0] * u[2] + min_area_pts[i][1] * v[2] + plane_center.z;
	  rect2d->push_back(minmin_pt);
	}



	if(std::abs(vertical.dot(n) - 1) < 0.1) {
	  std::stringstream plane_ss;
	  plane_ss << "plane_" << plane_id;
	  viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 1, 0, 0, plane_ss.str());
	  plane_ss << "_rect";
	  viewer.addPolygon<pcl::PointXYZ>(rect2d, 1, 0, 0, plane_ss.str());
	}

	else if(std::abs(vertical.dot(n)) < 0.4) {
	  std::stringstream plane_ss;
	  plane_ss << "plane_" << plane_id;
	  viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 0, 0, 1, plane_ss.str());
	  plane_ss << "_rect";
	  viewer.addPolygon<pcl::PointXYZ>(rect2d, 0, 0, 1, plane_ss.str());
	}
	else {
	  std::stringstream plane_ss;
	  plane_ss << "plane_" << plane_id;
	  viewer.addPolygon<pcl::PointXYZ>(cloud_hull, 0, 1, 0, plane_ss.str());
	  plane_ss << "_rect";
	  viewer.addPolygon<pcl::PointXYZ>(rect2d, 0, 1, 0, plane_ss.str());
	}

	++plane_id;
      }
  }

  std::clock_t model_and_hull = std::clock();


  std::cout << "Normal: " << (normal - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Region Grow: " << (region_grow - normal) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Model and Hull: " << (model_and_hull - region_grow) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;
  std::cout << "Total Time: " << (model_and_hull - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;



  while (!viewer.wasStopped ())
    {
      viewer.spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}
