#ifndef WALRUS_STAIR_DETECTOR_MULTI_TIMER_H_
#define WALRUS_STAIR_DETECTOR_MULTI_TIMER_H_

#include <limits>
#if DEBUG_TIMING

#include <map>
#include <vector>
#include <boost/timer/timer.hpp>
#include <boost/foreach.hpp>
#include <algorithm>

#endif

namespace walrus_stair_detector {


struct TimerPeriod {
#if DEBUG_TIMING
  TimerPeriod(const boost::timer::cpu_times& t)
    : user(t.user/1e9), system(t.system/1e9), wall(t.wall/1e9) {}
#endif
  TimerPeriod()
    : user(std::numeric_limits<double>::quiet_NaN()),
      system(std::numeric_limits<double>::quiet_NaN()),
      wall(std::numeric_limits<double>::quiet_NaN()) {}

  double user;
  double system;
  double wall;
};

#if DEBUG_TIMING
class MultiTimer {
public:
  void end(const std::string& segment_name) {
    if(std::find(segment_order.begin(), segment_order.end(), segment_name) == segment_order.end())
      segment_order.push_back(segment_name);
    times[segment_name].push_back(timer.elapsed());
    timer.start();
  }
  void skip() {
    timer.start();
  }
  TimerPeriod getTotal() {
    return total_timer.elapsed();
  }
  TimerPeriod getSegmentTotal(const std::string& segment_name) {
    const std::vector<boost::timer::cpu_times>& segment_times = times[segment_name];
    boost::timer::cpu_times total = {0, 0, 0};
    BOOST_FOREACH(boost::timer::cpu_times time, segment_times) {
      total.wall += time.wall;
      total.user += time.user;
      total.system += time.system;
    }
    return total;
  }
  void print() {
    BOOST_FOREACH(const std::string& segment_name, segment_order) {
      const std::vector<boost::timer::cpu_times>& segment_times = times[segment_name];
      if(segment_times.size() > 1) {
	std::cout << segment_name << ": [" << segment_times.size() << " segments]" << std::endl;
	boost::timer::cpu_times total = {0, 0, 0};
	boost::timer::cpu_times average = {0, 0, 0};
	BOOST_FOREACH(boost::timer::cpu_times time, segment_times) {
	  //std::cout << "\t" << boost::timer::format(time, 3, "User: %us, System: %ss, Wall: %ws") << std::endl;
	  total.wall += time.wall;
	  total.user += time.user;
	  total.system += time.system;
	}
	average.wall += total.wall / segment_times.size();
	average.user += total.user / segment_times.size();
	average.system += total.system / segment_times.size();
	std::cout << "\tTotal: " << boost::timer::format(total, 3, "User: %us, System: %ss, Wall: %ws") << std::endl;
	std::cout << "\tAverage: " << boost::timer::format(average, 3, "User: %us, System: %ss, Wall: %ws") << std::endl;
      }
      else {
	std::cout << segment_name << ": " << boost::timer::format(segment_times[0], 3, "User: %us, System: %ss, Wall: %ws") << std::endl;
      }
    }
    std::cout << "TOTAL: " << boost::timer::format(total_timer.elapsed(), 3, "User: %us, System: %ss, Wall: %ws") << std::endl;
  }
private:
  typedef std::map<std::string, std::vector<boost::timer::cpu_times> > TimesCollection;
  boost::timer::cpu_timer timer;
  boost::timer::cpu_timer total_timer;
  std::vector<std::string> segment_order;
  TimesCollection times;
};
#else
class MultiTimer {
public:
  void end(const std::string& segment_name) {}
  void skip() {}
  TimerPeriod getSegmentTotal(const std::string& segment_name) {
    return TimerPeriod();
  }
  TimerPeriod getTotal() {
    return TimerPeriod();
  }
};
#endif

}


#endif

