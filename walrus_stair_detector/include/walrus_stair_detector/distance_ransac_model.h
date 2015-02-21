#ifndef WALRUS_STAIR_DETECTOR_DISTANCE_RANSAC_MODEL_H_
#define WALRUS_STAIR_DETECTOR_DISTANCE_RANSAC_MODEL_H_

#include <cstddef>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <limits>
#include <walrus_stair_detector/ransac.h>

namespace walrus_stair_detector {

struct DistanceModel {
  double spacing;
  double offset;
  std::multimap<int, int> groups;
};

class DistanceRansacModel : public RansacModel<double, DistanceModel> {
public:
  DistanceRansacModel(double initial_guess, double max_error)
    : RansacModel(1),
      initial_guess_(initial_guess), max_error_(max_error) {}
  virtual ~DistanceRansacModel() {}

  virtual bool generateInitialModel(const SequenceView<double>& data, DistanceModel* model_out) const {
    model_out->offset = data[0];
    model_out->spacing = initial_guess_;
    model_out->groups.insert(std::make_pair(0, 0));
    return true;
  }

  virtual bool fitsModel(const DistanceModel& model, const double& value) const {
    double offset = fmod(std::fabs(value - model.offset), model.spacing);
    if(offset < max_error_ || model.spacing - offset < max_error_)
      return true;
    return false;
  }

  virtual bool enoughInliers(int num_inliers, int data_size) const {
    return num_inliers > data_size / 2;
  }

  virtual bool generateCompleteModel(const SequenceView<double>& data, const DistanceModel& initial_model, DistanceModel* model_out) const {
    double min = std::numeric_limits<double>::infinity();
    for(size_t i = 0; i < data.size(); ++i) {
      if(data[i] < min)
	min = data[i];
    }
    std::vector<std::pair<double, double> > ls_data;
    for(size_t i = 0; i < data.size(); ++i) {
      int group = (int)round((data[i] - min) / initial_model.spacing);
      ls_data.push_back(std::make_pair(group, data[i]));
      model_out->groups.insert(std::make_pair(group, i));
    }

    double mean_x = 0;
    double mean_y = 0;
    for(size_t i = 0; i < ls_data.size(); ++i) {
      const std::pair<double, double>& value = ls_data[i];
      mean_x += value.first;
      mean_y += value.second;
    }
    mean_x /= ls_data.size();
    mean_y /= ls_data.size();

    double Sx = 0;
    double Sxy = 0;
    for(size_t i = 0; i < ls_data.size(); ++i) {
      const std::pair<double, double>& value = ls_data[i];
      Sx += (value.first - mean_x) * (value.first - mean_x);
      Sxy += (value.first - mean_x) * (value.second - mean_y);
    }

    model_out->spacing = Sxy / Sx;
    model_out->offset = mean_y - model_out->spacing * mean_x;
    return true;
  }

  virtual double getModelError(const SequenceView<double>& data, int num_outliers, const DistanceModel& model) const {
    double fit = 0;
    for(size_t i = 0; i < data.size(); ++i) {
      double offset = std::fmod(std::fabs(data[i] - model.offset), model.spacing);
      if(offset > model.spacing / 2)
	fit += model.spacing - offset;
      else
	fit += offset;
    }
    fit /= data.size();
    fit *= pow(1.05, num_outliers);
    return fit;
  }
private:
  double initial_guess_;
  double max_error_;

};

}

#endif
