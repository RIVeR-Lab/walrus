#ifndef SCREEN_BASE_H
#define SCREEN_BASE_H

namespace diagnostic_peripheral_base
{

  class ScreenBase
  {
  public:
    ScreenBase(int height, int width, ros::NodeHandle& nh, ros::NodeHandle& pnh, ScreenBase* previous, ScreenBase* home)
      : height(height), width(width), nh(nh), pnh(pnh), previous(previous), home(home) active(false) {}

    void activate();
    void deactivate();

  protected:
    int height;
    int width;
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ScreenBase* previous;
    ScreenBase* home;
    bool active;
  };


}


#endif //SCREEN_BASE_H
