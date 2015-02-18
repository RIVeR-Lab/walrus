#include <ros/ros.h>
#include <walrus_stair_detector/walrus_stair_detector.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <gtest/gtest.h>
#include "yaml-cpp/yaml.h"
#include <boost/filesystem.hpp>

std::string pcd_file;
std::string config_file;

namespace YAML {
template<>
struct convert<Eigen::Vector3f> {
  static bool decode(const YAML::Node& node, Eigen::Vector3f& rhs) {
    if(!node.IsSequence() || node.size() != 3) {
      return false;
    }

    rhs[0] = node[0].as<double>();
    rhs[1] = node[1].as<double>();
    rhs[2] = node[2].as<double>();
    return true;
  }
};
}

#define EXPECT_VECTOR_ANGLE_LE(val1, val2, max_angle)			\
  do {									\
    Eigen::Vector3f val1_norm = val1.normalized();			\
    Eigen::Vector3f val2_norm = val2.normalized();			\
    if(val1_norm != val2_norm) {					\
      double angle = acos(val1_norm.dot(val2_norm));			\
      EXPECT_LE(fabs(angle), max_angle)					\
	<< "Expected angle between "					\
	<< "[" << val1[0] << ", " << val1[1] << ", " << val1[2] << "]"	\
	<< " and [" << val2[0] << ", " << val2[1] << ", " << val2[2] << "]" \
	<< " <= " << max_angle << ", but was " << angle;		\
    }									\
  } while(0)

#define EXPECT_VECTOR_NEAR(val1, val2, abs_error)			\
  do {									\
    double error = (val1 - val2).norm();				\
    EXPECT_LE(error, abs_error)						\
      << "Expected error between "					\
      << "[" << val1[0] << ", " << val1[1] << ", " << val1[2] << "]"	\
      << " and [" << val2[0] << ", " << val2[1] << ", " << val2[2] << "]" \
      << " <= " << abs_error << ", but was " << error;			\
  } while(0)

struct StairDetectorTestData {
  StairDetectorTestData(std::string pcd_file, std::string config_file) : pcd_file(pcd_file), config_file(config_file) {}
  std::string pcd_file;
  std::string config_file;
};
void PrintTo(const StairDetectorTestData& data, ::std::ostream* os) {
  *os << "PCD File: " << data.pcd_file << ", Config File: " << data.config_file;
}

class StairDetectorTest : public ::testing::TestWithParam<StairDetectorTestData> {};


TEST_P(StairDetectorTest, detectStairs) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile(GetParam().pcd_file, *cloud);

  YAML::Node config = YAML::LoadFile(GetParam().config_file);
  YAML::Node sensor_config = config["sensor"];
  YAML::Node stairs_config = config["stairs"];

  Eigen::Vector3f vertical_estimate = sensor_config["vertical"].as<Eigen::Vector3f>();

  walrus_stair_detector::WalrusStairDetector detector;
  std::vector<walrus_stair_detector::StairModel> stairs;
  detector.detect(cloud, vertical_estimate, &stairs);

  ASSERT_EQ(stairs_config.size(), stairs.size());

  for(int i = 0; i < stairs.size(); ++i) {
    YAML::Node stair_config = stairs_config[i];
    walrus_stair_detector::StairModel& stair = stairs[i];
    EXPECT_NEAR(stair_config["rise"].as<double>(), stair.rise, 0.1);
    EXPECT_NEAR(stair_config["run"].as<double>(), stair.run, 0.1);

    EXPECT_VECTOR_ANGLE_LE(stair_config["direction"].as<Eigen::Vector3f>(), stair.direction, 0.06);
    EXPECT_VECTOR_NEAR(stair_config["origin"].as<Eigen::Vector3f>(), stair.origin, 0.4); // for now keep this large because the origin drifts left and right

    //EXPECT_NEAR(stair_config["width"].as<double>(), stair.width, 0.1);
    //EXPECT_EQ(stair_config["num_stairs"].as<int>(), stair.num_stairs);
  }
}

std::string find_package(const std::string& package, const std::string& path) {
  std::stringstream command;
  command << "catkin_find " << package << " " << path;

  char buf[1000];
  FILE *fp = popen(command.str().c_str(), "r");
  if (fp == NULL) {
    return "";
  }

  std::string result;
  while (fgets(buf, sizeof(buf), fp) != NULL) {
    result = buf;
    break;
  }

  if(result.size() > 0 && result[result.size()-1] == '\n')
    result = result.substr(0, result.size()-1);

  pclose(fp);
  return result;
}

std::vector<StairDetectorTestData> GenerateStairDetectorParameterList() {
  std::vector<StairDetectorTestData> data;
  std::string testdata_root_string = find_package("walrus_testdata", "kinectv2/stairs");

  if(testdata_root_string.size() > 0) {
    boost::filesystem::path testdata_root (testdata_root_string);
    std::cout << "Searching for test data in: " << testdata_root << std::endl;
    if(!boost::filesystem::is_directory(testdata_root))
      throw new std::runtime_error("Test data path is not a directory");

    boost::filesystem::directory_iterator end;
    for(boost::filesystem::directory_iterator itr(testdata_root); itr != end; ++itr) {
      boost::filesystem::path test_collection = itr->path();
      std::cout << "\tFound test collection: " << test_collection.filename() << std::endl;

      boost::filesystem::path test_collection_description = test_collection;
      test_collection_description/="description.yaml";
      if(boost::filesystem::exists(test_collection_description)){
	for(boost::filesystem::directory_iterator itr2(test_collection); itr2 != end; ++itr2) {
	  if(itr2->path().extension() == ".pcd") {
	    std::cout << "\t\tFound test: " << itr2->path().filename() << std::endl;
	    data.push_back(StairDetectorTestData(itr2->path().native(), test_collection_description.native()));
	  }
	}
      }
      else
	std::cout << "Did not find test description: " << test_collection_description << std::endl;
    }

  }
  else
    throw new std::runtime_error("Could not find test data folder in walrus_testdata");
  return data;
}
INSTANTIATE_TEST_CASE_P(
			StairTestData,
			StairDetectorTest,
			testing::ValuesIn(GenerateStairDetectorParameterList()));

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "stair_detector_test");

  return RUN_ALL_TESTS();
}
