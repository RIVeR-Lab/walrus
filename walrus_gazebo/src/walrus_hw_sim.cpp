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

  std::vector<gazebo::physics::JointPtr> effort_joints;
  std::vector<double> effort_cmds;
  std::vector<JointState> effort_states;

  hardware_interface::JointStateInterface js_interface;
  hardware_interface::EffortJointInterface ej_interface;

public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    std::string tf_prefix = "walrus/";

    // List of all effort joints to be initialized
    std::vector<std::string> effort_joint_names;

    // Enumerate pod joints
    for(int position_index = 0; position_index < POD_POSITIONS.size(); ++position_index) {
      for(int drive_index = 0; drive_index < DRIVE_SIDES.size(); ++drive_index) {
	const std::string& pod_name = POD_POSITIONS[position_index] + "_" + DRIVE_SIDES[drive_index]; // Create pod name front, left -> front_left
	const std::string& joint_name = tf_prefix+pod_name+"_pod_joint";
	effort_joint_names.push_back(joint_name);
      }
    }
    effort_joint_names.push_back(tf_prefix+"boom/deploy_joint");
    effort_joint_names.push_back(tf_prefix+"boom/pan_joint");
    effort_joint_names.push_back(tf_prefix+"boom/tilt_joint");

    // Load effort joints
    effort_cmds.resize(effort_joint_names.size(), 0);
    effort_states.resize(effort_joint_names.size());
    for(int i = 0; i < effort_joint_names.size(); ++i) {
      const std::string& joint_name = effort_joint_names[i];
      gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_name);
      if(!joint) {
	ROS_FATAL_STREAM("Could not load joint: " << joint_name);
	return false;
      }
      effort_joints.push_back(joint);
      js_interface.registerHandle(hardware_interface::JointStateHandle(joint_name, &effort_states[i].pos, &effort_states[i].vel, &effort_states[i].eff));
      ej_interface.registerHandle(hardware_interface::JointHandle(js_interface.getHandle(joint_name), &effort_cmds[i]));
    }


    registerInterface(&js_interface);
    registerInterface(&ej_interface);

    return true;
  }

  // Read state from the simulator
  void readSim(ros::Time time, ros::Duration period) {
    for(int i = 0; i<effort_joints.size(); ++i){
      gazebo::physics::JointPtr joint = effort_joints[i];
      effort_states[i].pos += angles::shortest_angular_distance(effort_states[i].pos, joint->GetAngle(0).Radian());
      effort_states[i].vel = joint->GetVelocity(0);
      effort_states[i].eff = joint->GetForce((unsigned int)(0));
    }
  }

  // Write control to the simulator
  void writeSim(ros::Time time, ros::Duration period) {
    for(int i = 0; i<effort_joints.size(); ++i){
      gazebo::physics::JointPtr joint = effort_joints[i];
      joint->SetForce(0, effort_cmds[i]);
    }
  }

};


std::vector<std::string> WalrusHWSim::POD_POSITIONS = boost::assign::list_of("front")("back");
std::vector<std::string> WalrusHWSim::DRIVE_SIDES = boost::assign::list_of("left")("right");

}

PLUGINLIB_EXPORT_CLASS(walrus_gazebo::WalrusHWSim, gazebo_ros_control::RobotHWSim)
