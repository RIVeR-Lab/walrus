#ifndef MULTI_USB_CAM_H_
#define MULTI_USB_CAM_H_

#include <ros/ros.h>
#include <vector>
#include <sensor_msgs/Image.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/camera_publisher.h>
#include <boost/noncopyable.hpp>
#include "usb_cam/usb_cam.h"

namespace multi_usb_cam {

class CameraConfiguration {
public:
  CameraConfiguration(int width, int height, int fps)
    : width(width), height(height), fps(fps) {}
  int width;
  int height;
  int fps;
};

class CameraDefinition {
public:
  CameraDefinition(const std::string& name, const std::string& topic, const std::string& frame_id)
    : name(name), topic(topic), frame_id(frame_id) {}
  std::string name;
  std::string topic;
  std::string frame_id;
};

class MultiUsbCam : private boost::noncopyable {
 public:
  enum State {
    STATE_UNINITIALIZED, STATE_INACTIVE, STATE_ACTIVE
  };

  MultiUsbCam(const CameraDefinition& definition, image_transport::CameraPublisher pub);

  void activate(const CameraConfiguration& config);
  void deactivate();
  bool active() { return state_ == STATE_ACTIVE; }
  std::string topic() { return pub_.getTopic(); }
  void publish_image();

 private:
  CameraDefinition definition_;
  usb_cam::UsbCam camera_;
  image_transport::CameraPublisher pub_;
  sensor_msgs::Image img_;

  State state_;

};

class MultiUsbCamNode {
 public:
  MultiUsbCamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MultiUsbCamNode();


  void spin();

 private:
  std::vector<CameraConfiguration> camera_configs_;
  std::vector<boost::shared_ptr<MultiUsbCam> > cameras_;
  typedef std::map<std::string, int> CameraSubs;
  CameraSubs camera_subs_;


  void subscribe_callback(const image_transport::SingleSubscriberPublisher& e);

};

}


#endif
