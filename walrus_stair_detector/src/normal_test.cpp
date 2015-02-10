#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <ctime>

int main(int argc, char *argv[]) {
  if(argc != 2) {
    printf("Expected a single argument");
    return -1;
  }
  // load point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[1], *cloud);

  // estimate normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);


  std::clock_t start = std::clock();

  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(cloud);
  ne.compute(*normals);


  std::cout << "Time: " << (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << " ms" << std::endl;



  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor (0.0, 0.0, 0.5);
  viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);

  while (!viewer.wasStopped ()) {
    viewer.spinOnce ();
  }
  return 0;
}
