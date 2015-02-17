#ifndef WALRUS_STAIR_DETECTOR_SLOPE_RANSAC_MODEL_H_
#define WALRUS_STAIR_DETECTOR_SLOPE_RANSAC_MODEL_H_

#include <cstddef>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <limits>
#include <walrus_stair_detector/ransac.h>
#include <boost/foreach.hpp>

namespace walrus_stair_detector {

struct SlopeModel {
  double slope;
  double offset;
};

class SlopeRansacModel : public RansacModel<std::pair<double, double>, SlopeModel> {
public:
  typedef std::pair<double, double> DataT;
  SlopeRansacModel()
    : RansacModel(2) {}
  virtual ~SlopeRansacModel() {}

  virtual bool generateInitialModel(const SequenceView<DataT>& data, SlopeModel* model_out) const {
    model_out->slope = (data[1].second - data[0].second) / (data[1].first - data[0].first);
    model_out->offset = data[0].second - data[0].first * model_out->slope;
    return true;
  }

  virtual bool fitsModel(const SlopeModel& model, const DataT& value) const {
    return std::fabs(value.second - (value.first * model.slope + model.offset)) < 0.2;
  }

  virtual bool enoughInliers(int num_inliers, int data_size) const {
    return num_inliers > data_size / 2;
  }

  virtual bool generateCompleteModel(const SequenceView<DataT>& data, const SlopeModel& initial_model, SlopeModel* model_out) const {
    // Least squares linear regression
    double mean_x = 0;
    double mean_y = 0;
    for(int i = 0; i < data.size(); ++i) {
      const DataT& value = data[i];
      mean_x += value.first;
      mean_y += value.second;
    }
    mean_x /= data.size();
    mean_y /= data.size();

    double Sx = 0;
    double Sxy = 0;
    for(int i = 0; i < data.size(); ++i) {
      const DataT& value = data[i];
      Sx += (value.first - mean_x) * (value.first - mean_x);
      Sxy += (value.first - mean_x) * (value.second - mean_y);
    }

    model_out->slope = Sxy / Sx;
    model_out->offset = mean_y - model_out->slope * mean_x;
    return true;
  }

  virtual double getModelError(const SequenceView<DataT>& data, int num_outliers, const SlopeModel& model) const {
    double fit = 0;
    for(int i = 0; i < data.size(); ++i) {
      fit += data[i].second - (data[i].first * model.slope + model.offset);
    }
    fit /= data.size();
    fit *= pow(1.05, num_outliers);
    return fit;
  }

};

}

#endif