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

  // TODO use least squares to compute model?
  virtual bool generateCompleteModel(const SequenceView<double>& data, const DistanceModel& initial_model, DistanceModel* model_out) const {
    double min = std::numeric_limits<double>::infinity();
    for(int i = 0; i < data.size(); ++i) {
      if(data[i] < min)
	min = data[i];
    }
    double offset_error = 0;
    for(int i = 0; i < data.size(); ++i) {
      int group = (int)round((data[i] - min) / initial_model.spacing);
      double estimated = min + group * initial_model.spacing;
      offset_error += data[i] - estimated;
      model_out->groups.insert(std::make_pair(group, i));
    }
    offset_error /= data.size();
    if(std::fabs(offset_error) > max_error_)
      return false;

    model_out->offset = min + offset_error;

    double spacing_error = 0;
    int count = 0;
    for(std::multimap<int, int>::const_iterator itr = model_out->groups.begin(); itr != model_out->groups.end(); ++itr) {
      int group = itr->first;
      double estimated = model_out->offset + group * initial_model.spacing;
      spacing_error += (data[itr->second] - estimated) * group;
      count += group;
    }
    spacing_error /= count;
    if(std::fabs(spacing_error) > max_error_)
      return false;
    model_out->spacing = initial_model.spacing + spacing_error;
    return true;
  }

  virtual double getModelError(const SequenceView<double>& data, int num_outliers, const DistanceModel& model) const {
    double fit = 0;
    for(int i = 0; i < data.size(); ++i) {
      double offset = std::fmod(std::fabs(data[i] - model.offset), model.spacing);
      if(offset > model.spacing / 2)
	fit += model.spacing - offset;
      else
	fit += offset;
    }
    fit /= data.size();
    fit *= pow(1.02, num_outliers);
    return fit;
  }
private:
  double initial_guess_;
  double max_error_;

};

}

#endif
