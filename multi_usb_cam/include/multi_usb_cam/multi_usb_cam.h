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

class MultiUsbCam : private boost::noncopyable {
 public:
  const static int STATE_UNINITIALIZED = -1;
  const static int STATE_INACTIVE = 0;
  // If a camera's state is greater than zero then it is total number of active cameras

  MultiUsbCam(const std::string& device_name, image_transport::CameraPublisher pub);

  void change_state(int new_state);
  bool active() { return state_ > STATE_INACTIVE; }
  std::string topic() { return pub_.getTopic(); }
  void publish_image();

 private:
  std::string device_name_;
  usb_cam::UsbCam camera_;
  image_transport::CameraPublisher pub_;
  sensor_msgs::Image img_;

  int state_;

};

class MultiUsbCamNode {
 public:
  MultiUsbCamNode(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  ~MultiUsbCamNode();


  void spin();

 private:
  std::vector<boost::shared_ptr<MultiUsbCam> > cameras_;
  typedef std::map<std::string, int> CameraSubs;
  CameraSubs camera_subs_;


  void subscribe_callback(const image_transport::SingleSubscriberPublisher& e);

};

}


#endif
