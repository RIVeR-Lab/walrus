#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>

using namespace boost::accumulators;

typedef accumulator_set<double, stats<tag::mean, tag::min, tag::max > > double_acc;

int main(int argc, char **argv)
{
  walrus_stair_detector::WalrusStairDetector detector;

  Eigen::Vector3f vertical_estimate(0, -1, 0);

  unsigned int count = 0;
  Eigen::Vector3f origin(0, 0, 0);
  Eigen::Vector3f vertical(0, 0, 0);
  Eigen::Vector3f direction(0, 0, 0);
  Eigen::Vector3f horizontal(0, 0, 0);
  double_acc rise;
  double_acc run;
  double_acc width;
  accumulator_set<unsigned int, stats<tag::min, tag::max > > num_stairs;

  for(int i = 0; i < argc - 1; ++i) {
    std::string filename = argv[i+1];
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(filename, *cloud);

    std::vector<walrus_stair_detector::StairModel> stairs;
    detector.detect(cloud, vertical_estimate, &stairs);

    if(stairs.size() != 1) {
      std::cout << "Did not find exactly one stair in " << filename << std::endl;
      continue;
    }

    walrus_stair_detector::StairModel& stair = stairs[0];
    origin += stair.origin;
    vertical += stair.vertical;
    direction += stair.direction;
    horizontal += stair.horizontal;
    rise(stair.rise);
    run(stair.run);
    width(stair.width);
    num_stairs(stair.num_stairs);
    ++count;
  }
  origin /= count;
  vertical /= count;
  direction /= count;
  horizontal /= count;

  std::cout << std::endl << std::endl << "Model Result:" << std::endl;
  std::cout << "Rise: min=" << min(rise) << ", max=" << max(rise) << " mean=" << mean(rise) << std::endl;
  std::cout << "Run: min=" << min(run) << ", max=" << max(run) << " mean=" << mean(run) << std::endl;
  std::cout << "Width: min=" << min(width) << ", max=" << max(width) << " mean=" << mean(width) << std::endl;
  std::cout << "Num Stairs: min=" << min(num_stairs) << ", max=" << max(num_stairs) << std::endl;
  std::cout << "Origin: " << origin[0] << ", " << origin[1] << ", " << origin[2] << std::endl;
  std::cout << "Vertical: " << vertical[0] << ", " << vertical[1] << ", " << vertical[2] << std::endl;
  std::cout << "Direction: " << direction[0] << ", " << direction[1] << ", " << direction[2] << std::endl;
  std::cout << "Horizontal: " << horizontal[0] << ", " << horizontal[1] << ", " << horizontal[2] << std::endl;
}
