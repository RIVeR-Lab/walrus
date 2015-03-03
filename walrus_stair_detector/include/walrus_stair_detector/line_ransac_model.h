#ifndef WALRUS_STAIR_DETECTOR_LINE_RANSAC_MODEL_H_
#define WALRUS_STAIR_DETECTOR_LINE_RANSAC_MODEL_H_

#include <cstddef>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <limits>
#include <walrus_stair_detector/ransac.h>
#include <boost/foreach.hpp>

namespace walrus_stair_detector {

struct LineModel {
  double slope;
  double offset;
};

class LineRansacModel : public RansacModel<std::pair<double, double>, LineModel> {
public:
  typedef std::pair<double, double> DataT;
  LineRansacModel()
    : RansacModel(2), inlier_threshold_(0.2), min_inliers_(0.5),
      min_slope_(-std::numeric_limits<double>::infinity()),
      max_slope_(std::numeric_limits<double>::infinity()) {}
  virtual ~LineRansacModel() {}

  void setInlierThreshold(const double threshold) {
    inlier_threshold_ = threshold;
  }
  void setMinInliers(const double percent) {
    min_inliers_ = percent;
  }
  void setSlopeBounds(const double min_slope, const double max_slope) {
    min_slope_ = min_slope;
    max_slope_ = max_slope;
  }

  virtual bool generateInitialModel(const SequenceView<DataT>& data, LineModel* model_out) const {
    model_out->slope = (data[1].second - data[0].second) / (data[1].first - data[0].first);
    model_out->offset = data[0].second - data[0].first * model_out->slope;
    if(model_out->slope > min_slope_ && model_out->slope < max_slope_)
      return true;
    return false;
  }

  virtual bool fitsModel(const LineModel& model, const DataT& value) const {
    return std::fabs(value.second - (value.first * model.slope + model.offset)) < inlier_threshold_;
  }

  virtual bool enoughInliers(int num_inliers, int data_size) const {
    return num_inliers >= data_size * min_inliers_;
  }

  virtual bool generateCompleteModel(const SequenceView<DataT>& data, const LineModel& initial_model, LineModel* model_out) const {
    // Least squares linear regression
    double mean_x = 0;
    double mean_y = 0;
    for(size_t i = 0; i < data.size(); ++i) {
      const DataT& value = data[i];
      mean_x += value.first;
      mean_y += value.second;
    }
    mean_x /= data.size();
    mean_y /= data.size();

    double Sx = 0;
    double Sxy = 0;
    for(size_t i = 0; i < data.size(); ++i) {
      const DataT& value = data[i];
      Sx += (value.first - mean_x) * (value.first - mean_x);
      Sxy += (value.first - mean_x) * (value.second - mean_y);
    }

    model_out->slope = Sxy / Sx;
    model_out->offset = mean_y - model_out->slope * mean_x;
    if(model_out->slope > min_slope_ && model_out->slope < max_slope_)
      return true;
    return false;
  }

  virtual double getModelError(const SequenceView<DataT>& data, int num_outliers, const LineModel& model) const {
    double fit = 0;
    for(size_t i = 0; i < data.size(); ++i) {
      fit += fabs(data[i].second - (data[i].first * model.slope + model.offset));
    }
    fit /= data.size();
    fit *= pow(1.05, num_outliers);
    return fit;
  }

private:
  double inlier_threshold_;
  double min_inliers_;
  double min_slope_, max_slope_;
};

}

#endif
