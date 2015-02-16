#ifndef WALRUS_STAIR_DETECTOR_PARALLEL_PLANE_RANSAC_MODEL_H_
#define WALRUS_STAIR_DETECTOR_PARALLEL_PLANE_RANSAC_MODEL_H_

#include <cstddef>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <limits>
#include <walrus_stair_detector/ransac.h>
#include <walrus_stair_detector/detected_plane.h>

namespace walrus_stair_detector {

class ParallelPlaneRansacModel : public RansacModel<DetectedPlane::Ptr, Eigen::Vector3f> {
public:
  ParallelPlaneRansacModel() : RansacModel(2) {}
  virtual ~ParallelPlaneRansacModel() {}

  virtual bool generateInitialModel(const SequenceView<DetectedPlane::Ptr>& data, Eigen::Vector3f* model_out) const {
    Eigen::Vector3f qa = data[0]->normal;
    const Eigen::Vector3f& qb =data[1]->normal;

    // TODO is the needed?
    if((-qa).dot(qb) > qa.dot(qb)) {
      qa = -qa;
    }
    *model_out = (qa + qb) / 2;
    return true;
  }

  virtual bool fitsModel(const Eigen::Vector3f& model, const DetectedPlane::Ptr& value) const {
    return value->normal.dot(model) > 0.92;
  }

  virtual bool enoughInliers(int num_inliers, int data_size) const {
    return num_inliers > data_size / 2;
  }

  virtual bool generateCompleteModel(const SequenceView<DetectedPlane::Ptr>& data, const Eigen::Vector3f& initial_model, Eigen::Vector3f* model_out) const {
    Eigen::Vector3f new_model;
    new_model << 0.0, 0.0, 0.0;
    for(int i = 0; i < data.size(); ++i) {
      new_model += data[i]->normal;
    }
    new_model /= data.size();
    new_model.normalize();
    *model_out = new_model;
    return true;
  }

  virtual double getModelError(const SequenceView<DetectedPlane::Ptr>& data, int num_outliers, const Eigen::Vector3f& model) const {
    double fit = 0;
    for(int i = 0; i < data.size(); ++i) {
      fit += M_PI/2 - model.dot(data[i]->normal);
    }
    fit /= data.size();
    return fit;
  }

};

}

#endif
