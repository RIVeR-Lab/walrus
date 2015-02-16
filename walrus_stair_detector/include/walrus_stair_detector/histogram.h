#ifndef WALRUS_STAIR_DETECTOR_HISTOGRAM_H_
#define WALRUS_STAIR_DETECTOR_HISTOGRAM_H_

#include <cstddef>
#include <vector>
#include <iostream>
#include <cmath>

namespace walrus_stair_detector {

// From http://stackoverflow.com/questions/10847007/using-the-gaussian-probability-density-function-in-c
double normal_pdf(double x, double m, double s)
{
  static const double inv_sqrt_2pi = 0.3989422804014327;
  double a = (x - m) / s;
  return inv_sqrt_2pi / s * std::exp(-0.5f * a * a);
}


class Histogram {
public:
  Histogram(double min, double max, double sigma, int num_buckets) : min_(min), max_(max), sigma_(sigma), num_buckets_(num_buckets) {
    buckets.resize(num_buckets);
    bucket_size_ = (max_ - min_) / num_buckets;
  }
  void add(double value, double weight = 1.0) {
    for(int i = 0; i < num_buckets_; ++i) {
      buckets[i] += normal_pdf(valueAtIndex(i), value, sigma_) * weight;
    }
  }
  double largestBucket() {
    double max = -1;
    int max_index = -1;
    for(int i = 0; i < num_buckets_; ++i) {
      if(buckets[i] > max) {
	max = buckets[i];
	max_index = i;
      }
    }
    return valueAtIndex(max_index);
  }
  void print() {
    double max = 0;
    for(int i = 0; i < num_buckets_; ++i) {
      if(buckets[i] > max)
	max = buckets[i];
    }
    std::cerr << "Histogram:" << std::endl;
    std::cerr.precision(6);
    for(int i = 0; i < num_buckets_; ++ i) {
      std::cerr << valueAtIndex(i) << ": ";
      for(int j = 0; j < 40 * buckets[i] / max; ++j)
	std::cerr << "*";
      std::cerr << "        " << buckets[i] << std::endl;
    }
    std::cerr.unsetf ( std::ios::floatfield );
  }
private:
  double valueAtIndex(int i) {
    return min_ + i * bucket_size_ + bucket_size_ / 2;
  }

  double min_;
  double max_;
  double sigma_;
  double bucket_size_;
  int num_buckets_;
  std::vector<double> buckets;
};

}

#endif
