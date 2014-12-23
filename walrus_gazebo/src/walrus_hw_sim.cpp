#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <angles/angles.h>
#include <gazebo_ros_control/robot_hw_sim.h>
#include <urdf/model.h>
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>

namespace walrus_gazebo
{

typedef struct {
  double pos;
  double vel;
  double eff;
} JointState;


class WalrusHWSim : public gazebo_ros_control::RobotHWSim
{
private:
  static std::vector<std::string> POD_POSITIONS;
  static std::vector<std::string> DRIVE_SIDES;
  static double MAX_DRIVE_TORQUE;

  std::vector<gazebo::physics::JointPtr> pod_joints;
  std::vector<double> pod_cmds;
  std::vector<JointState> pod_states;

  std::vector<std::vector<gazebo::physics::JointPtr> > drive_joints;
  std::vector<std::vector<double> > drive_joint_multipliers;
  std::vector<double> drive_cmds;
  std::vector<JointState> drive_states;

  hardware_interface::JointStateInterface js_interface;
  hardware_interface::VelocityJointInterface vj_interface;
  hardware_interface::PositionJointInterface pj_interface;

public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    // Load pod joints
    pod_cmds.resize(POD_POSITIONS.size() * DRIVE_SIDES.size(), 0);
    pod_states.resize(POD_POSITIONS.size() * DRIVE_SIDES.size());
    std::string urdf_namespace = robot_namespace.substr(1);//remove leading slash
    for(int position_index = 0; position_index < POD_POSITIONS.size(); ++position_index) {
      for(int drive_index = 0; drive_index < DRIVE_SIDES.size(); ++drive_index) {
	int pod_index = position_index + drive_index * POD_POSITIONS.size();
	const std::string& pod_name = POD_POSITIONS[position_index] + "_" + DRIVE_SIDES[drive_index]; // Create pod name front, left -> front_left
	const std::string& joint_name = urdf_namespace+"/"+pod_name+"_pod_joint";
	gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name);
	if(!joint) {
	  ROS_FATAL_STREAM("Could not load joint: " << joint_name);
	  return false;
	}
	pod_joints.push_back(joint);
	js_interface.registerHandle(hardware_interface::JointStateHandle(joint_name, &pod_states[pod_index].pos, &pod_states[pod_index].vel, &pod_states[pod_index].eff));
	pj_interface.registerHandle(hardware_interface::JointHandle(js_interface.getHandle(joint_name), &pod_cmds[pod_index]));
      }
    }

    // Load drive joints
    drive_cmds.resize(DRIVE_SIDES.size(), 0);
    drive_states.resize(DRIVE_SIDES.size());
    drive_joints.resize(DRIVE_SIDES.size());
    drive_joint_multipliers.resize(DRIVE_SIDES.size());
    for(int i = 0; i<DRIVE_SIDES.size(); ++i){
      const std::string& drive_name = DRIVE_SIDES[i];
      const std::string& drive_joint_name = urdf_namespace+"/"+drive_name+"_drive_joint";

      // Enumerate joint names for the drive
      std::vector<std::string> joint_names;
      joint_names.push_back(drive_joint_name); // Driven joint must be first
      joint_names.push_back(urdf_namespace+"/"+drive_name+"_drive_idler_joint");
      BOOST_FOREACH(const std::string& pod_position, POD_POSITIONS) {
	const std::string& pod_name = pod_position + "_" + drive_name;
	joint_names.push_back(urdf_namespace+"/"+pod_name+"_pod_drive_joint");
	joint_names.push_back(urdf_namespace+"/"+pod_name+"_pod_drive_idler_joint");
      }

      // Get drive joints
      BOOST_FOREACH(const std::string& joint_name, joint_names) {
	// Validate joint configuration
	gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name);
	if(!joint) {
	  ROS_FATAL_STREAM("Could not load joint: " << joint_name);
	  return false;
	}
	boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
	if(!urdf_joint) {
	  ROS_FATAL_STREAM("Could not load URDF joint: " << joint_name);
	  return false;
	}
	boost::shared_ptr<const urdf::JointMimic> joint_mimic = urdf_joint->mimic;
	if(drive_joints[i].empty() ^ !joint_mimic){
	  ROS_FATAL_STREAM("Drive joint mimic should be specified on all non-driven drive joints only: " << joint_name);
	  return false;
	}
	if(joint_mimic) {
	  if(joint_mimic->joint_name != drive_joint_name){
	    ROS_FATAL_STREAM("Drive joint mimic must refer to the driven joint: " << joint_name);
	    return false;
	  }
	  if(joint_mimic->multiplier == 0){
	    ROS_FATAL_STREAM("Drive joint mimic multiplier must not be 0: " << joint_name);
	    return false;
	  }
	  if(joint_mimic->offset != 0){
	    ROS_FATAL_STREAM("Drive joint mimic offset must be 0: " << joint_name);
	    return false;
	  }
	  drive_joint_multipliers[i].push_back(joint_mimic->multiplier);
	}
	else {
	  drive_joint_multipliers[i].push_back(1.0);
	}
	// Configure and store joint
	joint->SetMaxForce(0, MAX_DRIVE_TORQUE);
	drive_joints[i].push_back(joint);
      }

      js_interface.registerHandle(hardware_interface::JointStateHandle(drive_joint_name, &drive_states[i].pos, &drive_states[i].vel, &drive_states[i].eff));
      vj_interface.registerHandle(hardware_interface::JointHandle(js_interface.getHandle(drive_joint_name), &drive_cmds[i]));
    }

    registerInterface(&js_interface);
    registerInterface(&vj_interface);
    registerInterface(&pj_interface);

    return true;
  }

  // Read data from the simulator
  void readSim(ros::Time time, ros::Duration period) {
    for(int i = 0; i<pod_joints.size(); ++i){
      gazebo::physics::JointPtr joint = pod_joints[i];
      pod_states[i].pos += angles::shortest_angular_distance(pod_states[i].pos, joint->GetAngle(0).Radian());
      pod_states[i].vel = joint->GetVelocity(0);
      pod_states[i].eff = joint->GetForce((unsigned int)(0));
    }
    for(int i = 0; i<drive_joints.size(); ++i){
      gazebo::physics::JointPtr joint = drive_joints[i][0];//just use the first joint
      drive_states[i].pos += angles::shortest_angular_distance(drive_states[i].pos, joint->GetAngle(0).Radian());
      drive_states[i].vel = joint->GetVelocity(0);
      drive_states[i].eff = joint->GetForce((unsigned int)(0));
    }
  }

  // Write control to the simulator
  void writeSim(ros::Time time, ros::Duration period) {
    for(int i = 0; i<pod_joints.size(); ++i){
      gazebo::physics::JointPtr joint = pod_joints[i];
      joint->SetAngle(0, pod_cmds[i]);
    }
    for(int i = 0; i<drive_joints.size(); ++i){
      for(int j = 0; j < drive_joints[i].size(); ++j){
	gazebo::physics::JointPtr joint = drive_joints[i][j];
	double multiplier = drive_joint_multipliers[i][j];
        joint->SetVelocity(0, drive_cmds[i] * multiplier);
      }
    }
  }

};


std::vector<std::string> WalrusHWSim::POD_POSITIONS = boost::assign::list_of("front")("back");
std::vector<std::string> WalrusHWSim::DRIVE_SIDES = boost::assign::list_of("left")("right");
double WalrusHWSim::MAX_DRIVE_TORQUE = 100;

}

PLUGINLIB_EXPORT_CLASS(walrus_gazebo::WalrusHWSim, gazebo_ros_control::RobotHWSim)