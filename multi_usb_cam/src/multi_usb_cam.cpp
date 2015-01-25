#include <ros/ros.h>
#include "multi_usb_cam/multi_usb_cam.h"
#include <sensor_msgs/fill_image.h>
#include <image_transport/image_transport.h>
#include <boost/foreach.hpp>

namespace multi_usb_cam {


MultiUsbCam::MultiUsbCam(const std::string& device_name, image_transport::CameraPublisher pub)
  : device_name_(device_name), pub_(pub), state_(STATE_UNINITIALIZED) {
}

void MultiUsbCam::change_state(int new_state) {
  if(new_state < STATE_UNINITIALIZED)
    return;
  if(state_ == new_state)
    return;
  if(state_ == STATE_UNINITIALIZED && new_state == STATE_INACTIVE) // don't switch to inactive if the camera is uninitialized
    return;

  bool should_initialize = (state_ == STATE_UNINITIALIZED);

  bool should_stop = active();
  if(active()) { // if the camera is active then we should stop it
    camera_.shutdown();
    state_ = STATE_INACTIVE;
  }

  bool should_start =  new_state > STATE_INACTIVE; // if the camera will be active after the change then we will start

  if(should_start) {
    int width;
    int height;
    int fps;

    if(new_state == 1) {
      width = 1280;
      height = 720;
      fps = 30;
    }
    else if(new_state == 2) {
      width = 320;
      height = 240;
      fps = 10;
    }
    else
      return;

    camera_.start(device_name_, usb_cam::UsbCam::IO_METHOD_MMAP, usb_cam::UsbCam::PIXEL_FORMAT_YUYV, width, height, fps);
  }

  state_ = new_state;
}

void MultiUsbCam::publish_image() {
  if(!active())
    return;
  camera_.grab_image(&img_);
  img_.header.frame_id = "";

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

  std::vector<std::string> camera_names;
  camera_names.push_back("/dev/video0");
  camera_names.push_back("/dev/video1");

  BOOST_FOREACH(const std::string& name, camera_names) {
    image_transport::CameraPublisher pub = it.advertiseCamera(name, 1,
							      boost::bind(&MultiUsbCamNode::subscribe_callback, this, _1),
							      boost::bind(&MultiUsbCamNode::subscribe_callback, this, _1));
    cameras_.push_back(boost::shared_ptr<MultiUsbCam>(new MultiUsbCam(name, pub)));
    camera_subs_[name] = 0;
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
  int num_cameras = active_topics.size();
  ROS_INFO_STREAM("Reconfiguring cameras for " << num_cameras << " cameras");
  // first deactivate all cameras so that we don't try to subscribe to more than we should while switching
  BOOST_FOREACH(boost::shared_ptr<MultiUsbCam>& camera , cameras_) {
    camera->change_state(MultiUsbCam::STATE_INACTIVE);
  }
  BOOST_FOREACH(boost::shared_ptr<MultiUsbCam>& camera , cameras_) {
    if(std::find(active_topics.begin(), active_topics.end(), camera->topic()) != active_topics.end())
      camera->change_state(num_cameras);
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

