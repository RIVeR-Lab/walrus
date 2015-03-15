#ifndef WALRUS_BASE_HW_UTIL_H_
#define WALRUS_BASE_HW_UTIL_H_

#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface/transmission_parser.h>

namespace walrus_base_hw {

bool loadTransmissions(const std::string& urdf_string,
		       const std::vector<std::string> actuator_names,
		       transmission_interface::TransmissionInterfaceLoader* transmission_loader) {
  transmission_interface::TransmissionParser parser;
  std::vector<transmission_interface::TransmissionInfo> infos;
  if (!parser.parse(urdf_string, infos)) {
    ROS_ERROR("Error parsing URDF");
    return false;
  }

  BOOST_FOREACH(const transmission_interface::TransmissionInfo& info, infos) {
    bool found_some = false;
    bool found_all = true;
    BOOST_FOREACH(const transmission_interface::ActuatorInfo& actuator, info.actuators_) {
      if(std::find(actuator_names.begin(), actuator_names.end(), actuator.name_) != actuator_names.end())
	found_some = true;
      else
	found_all = false;
    }
    if(found_all) {
      if (!transmission_loader->load(info)) {
	ROS_ERROR_STREAM("Error loading transmission: " << info.name_);
	return false;
      }
      else
	ROS_DEBUG_STREAM("Loaded transmission: " << info.name_);
    }
    else if(found_some){
      ROS_ERROR_STREAM("Do not support transmissions that contain only some EPOS actuators: " << info.name_);
      return false;
    }
  }
  return true;
}

}

#endif
