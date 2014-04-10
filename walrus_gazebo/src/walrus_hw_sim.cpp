// ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_ros_control
#include <gazebo_ros_control/robot_hw_sim.h>

// URDF
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

static const double max_pod_torque = 100;
class WalrusHWSim : public gazebo_ros_control::RobotHWSim
{
private:
  std::vector<std::string> pod_names;
  std::vector<gazebo::physics::JointPtr> pod_joints;
  std::vector<double> pod_cmds;
  std::vector<JointState> pod_states;

  std::vector<std::string> drive_names;
  std::vector<std::string> other_drive_names;
  std::vector<std::vector<gazebo::physics::JointPtr> > drive_joints;
  std::vector<double> drive_cmds;
  std::vector<JointState> drive_states;

  hardware_interface::JointStateInterface js_interface;
  hardware_interface::EffortJointInterface ej_interface;

public:
  bool initSim(const std::string& robot_namespace, ros::NodeHandle model_nh, gazebo::physics::ModelPtr parent_model,
               const urdf::Model* const urdf_model, std::vector<transmission_interface::TransmissionInfo> transmissions)
  {
    pod_names = boost::assign::list_of("front_left")("back_left")("front_right")("back_right");
    pod_cmds = std::vector<double>(pod_names.size(), 0);
    pod_states = std::vector<JointState>(pod_names.size());

    std::string urdf_namespace = robot_namespace.substr(1);//remove leading slash
    for(int i = 0; i<pod_names.size(); ++i){
      std::string& pod_name = pod_names[i];
      std::string joint_name = urdf_namespace+"/"+pod_name+"_pod_joint";
      pod_joints.push_back(parent_model->GetJoint(joint_name));
      js_interface.registerHandle(hardware_interface::JointStateHandle(joint_name, &pod_states[i].pos, &pod_states[i].vel, &pod_states[i].eff));
      ej_interface.registerHandle(
          hardware_interface::JointHandle(js_interface.getHandle(joint_name), &pod_cmds[i]));
    }

    drive_names = boost::assign::list_of("left")("right");
    other_drive_names = boost::assign::list_of("$name_center_tread_back_cylinder_joint")("front_$name_pod_extended_cylinder_joint")("front_$name_pod_joint_cylinder_joint")("back_$name_pod_extended_cylinder_joint")("back_$name_pod_joint_cylinder_joint");
    drive_cmds = std::vector<double>(drive_names.size());
    drive_states = std::vector<JointState>(drive_names.size());
    drive_joints = std::vector<std::vector<gazebo::physics::JointPtr> >(drive_names.size());
    for(int i = 0; i<drive_names.size(); ++i){
      std::string& drive_name = drive_names[i];
      std::string joint_name = urdf_namespace+"/"+drive_name+"_drive_joint";
      drive_joints[i].push_back(parent_model->GetJoint(joint_name));
      BOOST_FOREACH(std::string other_drive_name, other_drive_names){
	std::string expanded_drive_name(urdf_namespace+"/"+other_drive_name);
	size_t pos = expanded_drive_name.find("$name");
	if (pos != std::string::npos) {
	  expanded_drive_name.replace(pos, 5, drive_name);
	}
        drive_joints[i].push_back(parent_model->GetJoint(expanded_drive_name));
      }

      js_interface.registerHandle(hardware_interface::JointStateHandle(joint_name, &drive_states[i].pos, &drive_states[i].vel, &drive_states[i].eff));
      ej_interface.registerHandle(
          hardware_interface::JointHandle(js_interface.getHandle(joint_name), &drive_cmds[i]));
    }

    // Register interfaces
    registerInterface(&js_interface);
    registerInterface(&ej_interface);

    return true;
  }

  void readSim(ros::Time time, ros::Duration period)
  {
    static int j = 0;
    for(int i = 0; i<pod_names.size(); ++i){
      gazebo::physics::JointPtr joint = pod_joints[i];
      pod_states[i].pos += angles::shortest_angular_distance(pod_states[i].pos, joint->GetAngle(0).Radian());
      pod_states[i].vel = joint->GetVelocity(0);
      pod_states[i].eff = joint->GetForce((unsigned int)(0));
    }
    for(int i = 0; i<drive_names.size(); ++i){
      gazebo::physics::JointPtr joint = drive_joints[i][0];//just use the first joint
      drive_states[i].pos += angles::shortest_angular_distance(drive_states[i].pos, joint->GetAngle(0).Radian());
      drive_states[i].vel = joint->GetVelocity(0);
      drive_states[i].eff = joint->GetForce((unsigned int)(0));
    }
  }

  void writeSim(ros::Time time, ros::Duration period)
  {
    for(int i = 0; i<pod_names.size(); ++i){
      gazebo::physics::JointPtr joint = pod_joints[i];
      double cmd = pod_cmds[i];
      if(cmd > max_pod_torque)
	cmd = max_pod_torque;
      if(cmd < -max_pod_torque)
	cmd = -max_pod_torque;
      joint->SetForce(0, cmd);
    }
    for(int i = 0; i<drive_names.size(); ++i){
      BOOST_FOREACH(gazebo::physics::JointPtr joint, drive_joints[i]){
        joint->SetForce(0, drive_cmds[i]);
      }
    }
  }

};

typedef boost::shared_ptr<WalrusHWSim> WalrusHWSimPtr;

}

PLUGINLIB_EXPORT_CLASS(walrus_gazebo::WalrusHWSim, gazebo_ros_control::RobotHWSim)
