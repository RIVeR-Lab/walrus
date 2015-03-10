#include <ros/ros.h>
#include "multi_usb_cam/multi_usb_cam.h"
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>

namespace multi_usb_cam {


MultiUsbCam::MultiUsbCam(const CameraDefinition& definition, image_transport::CameraPublisher pub)
  : definition_(definition), pub_(pub), state_(STATE_UNINITIALIZED) {
  img_.header.frame_id = definition_.frame_id;
}

void MultiUsbCam::activate(const CameraConfiguration& config) {
  if(active())
    deactivate();

  camera_.start(definition_.name, usb_cam::UsbCam::IO_METHOD_MMAP, usb_cam::UsbCam::PIXEL_FORMAT_YUYV, config.width, config.height, config.fps);

  state_ = STATE_ACTIVE;
}

void MultiUsbCam::deactivate() {
  if(active()) {
    camera_.shutdown();
    state_ = STATE_INACTIVE;
  }
}

void MultiUsbCam::publish_image() {
  if(!active())
    return;
  camera_.grab_image(&img_);

  // grab the camera info
  sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo());
  ci->width = img_.width;
  ci->height = img_.height;
  ci->header = img_.header;

  // publish the image
  pub_.publish(img_, *ci);
}


MultiUsbCamNode::MultiUsbCamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) {
  image_transport::ImageTransport it(nh);

  std::vector<CameraDefinition> camera_definitions;
  XmlRpc::XmlRpcValue cameras_xml;
  pnh.getParam("cameras", cameras_xml);
  ROS_ASSERT(cameras_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < cameras_xml.size(); ++i) {
    XmlRpc::XmlRpcValue camera_xml = cameras_xml[i];
    ROS_ASSERT(camera_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(camera_xml["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(camera_xml["topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
    ROS_ASSERT(camera_xml["frame_id"].getType() == XmlRpc::XmlRpcValue::TypeString);

    camera_definitions.push_back(CameraDefinition(static_cast<std::string>(camera_xml["name"]),
						  static_cast<std::string>(camera_xml["topic"]),
						  static_cast<std::string>(camera_xml["frame_id"])));
  }
  ROS_INFO("Loaded cameras:");
  BOOST_FOREACH(const CameraDefinition& camera, camera_definitions) {
    ROS_INFO_STREAM("\t" << camera.name << ": topic: " << camera.topic << ", frame_id: " << camera.frame_id);
  }

  XmlRpc::XmlRpcValue configs_xml;
  pnh.getParam("configs", configs_xml);
  ROS_ASSERT(configs_xml.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for(int i = 0; i < configs_xml.size(); ++i) {
    XmlRpc::XmlRpcValue config_xml = configs_xml[i];
    ROS_ASSERT(config_xml.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(config_xml["width"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(config_xml["height"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ROS_ASSERT(config_xml["fps"].getType() == XmlRpc::XmlRpcValue::TypeInt);

    camera_configs_.push_back(CameraConfiguration(static_cast<int>(config_xml["width"]),
						  static_cast<int>(config_xml["height"]),
						  static_cast<int>(config_xml["fps"])));
  }
  ROS_INFO("Loaded camera configurations:");
  BOOST_FOREACH(const CameraConfiguration& config, camera_configs_) {
    ROS_INFO_STREAM("\twidth: " << config.width << ", height: " << config.height << ", fps: " << config.fps);
  }

  BOOST_FOREACH(const CameraDefinition& camera, camera_definitions) {
    image_transport::CameraPublisher pub = it.advertiseCamera(camera.topic, 1,
							      boost::bind(&MultiUsbCamNode::subscribe_callback, this, _1),
							      boost::bind(&MultiUsbCamNode::subscribe_callback, this, _1));
    cameras_.push_back(boost::shared_ptr<MultiUsbCam>(new MultiUsbCam(camera, pub)));
    camera_subs_[camera.topic] = 0;
  }
}

MultiUsbCamNode::~MultiUsbCamNode()
{
}

void MultiUsbCamNode::subscribe_callback(const image_transport::SingleSubscriberPublisher& e) {
  camera_subs_[e.getTopic()] = e.getNumSubscribers();
  std::vector<std::string> active_topics;
  BOOST_FOREACH(const CameraSubs::value_type& camera_sub, camera_subs_) {
    if(camera_sub.second > 0)
      active_topics.push_back(camera_sub.first);
  }
  int cameras_to_activate = std::min(active_topics.size(), camera_configs_.size());
  ROS_INFO_STREAM("Reconfiguring cameras for " << active_topics.size() << " cameras. Actually activating " << cameras_to_activate << " cameras");
  if(cameras_to_activate > active_topics.size())
    ROS_WARN("Cannot activate all the requested cameras");
  // first deactivate all cameras so that we don't try to subscribe to more than we should while switching
  BOOST_FOREACH(boost::shared_ptr<MultiUsbCam>& camera , cameras_) {
    camera->deactivate();
  }

  const CameraConfiguration& config = camera_configs_[cameras_to_activate-1];
  int cameras_activated = 0;
  BOOST_FOREACH(boost::shared_ptr<MultiUsbCam>& camera , cameras_) {
    if(std::find(active_topics.begin(), active_topics.end(), camera->topic()) != active_topics.end()) {
      if(cameras_activated < cameras_to_activate) {
	camera->activate(config);
	++cameras_activated;
      }
    }
  }
}


void MultiUsbCamNode::spin() {
  ros::Rate loop_rate(30);
  while (ros::ok()) {
    BOOST_FOREACH(boost::shared_ptr<MultiUsbCam>& camera , cameras_) {
      camera->publish_image();
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

}

