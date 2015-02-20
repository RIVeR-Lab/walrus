#ifndef WALRUS_STAIR_DETECTOR_RANSAC_H_
#define WALRUS_STAIR_DETECTOR_RANSAC_H_

#include <cstddef>
#include <vector>
#include <algorithm>
#include <assert.h>
#include <limits>
#include <walrus_stair_detector/sequence_view.h>

namespace walrus_stair_detector {

template <class DataT, class ModelT> class RansacModel {
public:
  RansacModel(int minimum_model_size) :minimum_model_size_(minimum_model_size) {}
  virtual ~RansacModel() {}

  // Get the miminum number of elements to generate a model
  int getMinimumModelSize() const { return minimum_model_size_; }
  virtual bool generateInitialModel(const SequenceView<DataT>& data, ModelT* model_out) const = 0;
  virtual bool fitsModel(const ModelT& model, const DataT& value) const = 0;
  virtual bool generateCompleteModel(const SequenceView<DataT>& data, const ModelT& initial_model, ModelT* model_out) const = 0;
  virtual bool enoughInliers(int num_inliers, int data_size) const = 0;
  virtual double getModelError(const SequenceView<DataT>& data, int num_outliers, const ModelT& model) const = 0;


private:
  int minimum_model_size_;
};

template <class DataT, class ModelT> class Ransac {
public:
  Ransac(const RansacModel<DataT, ModelT>* model)
    : model_(model), data_(NULL), max_iterations_(30) {

  }

  void setInput(std::vector<DataT>* data) {
    data_ = new ProxySequenceView<std::vector<DataT> >(data);
  }
  void setInput(std::vector<DataT>* data, const std::vector<int>* indices) {
    data_ = new IndexSequenceView<std::vector<DataT> >(data, indices);
  }

  void setMaxIterations(int num_itr) {
    max_iterations_ = num_itr;
  }

  bool estimate(std::vector<int>* inliers_out, ModelT* model_out) {
    assert(model_ != NULL);
    assert(data_ != NULL);
    assert(inliers_out != NULL);
    assert(model_out != NULL);

    double best_fit = std::numeric_limits<double>::infinity();

    for(int iteration = 0; iteration < max_iterations_; ++iteration) {
      std::vector<int> initial_indices;
      if(!selectRandomIndices(&initial_indices)) {
#if DEBUG_RANSAC
	std::cerr << "failed to select initial RANSAC indices" << std::endl;
#endif
	continue;
      }

      IndexSequenceView<SequenceView<DataT> > initial_data(data_, &initial_indices);
      ModelT initial_model;
      if(!model_->generateInitialModel(initial_data, &initial_model)) {
#if DEBUG_RANSAC
	std::cerr << "failed to generate initial RANSAC model" << std::endl;
#endif
	continue;
      }

      std::vector<int> estimate_inlier_indices;
      for(int i = 0; i < data_->size(); ++i) {
	if(std::find(initial_indices.begin(), initial_indices.end(), i) != initial_indices.end())
	  continue;
	if(model_->fitsModel(initial_model, (*data_)[i]))
	  estimate_inlier_indices.push_back(i);
      }

      if(model_->enoughInliers(estimate_inlier_indices.size(), data_->size())) {
	std::vector<int> all_inlier_indices;
	all_inlier_indices.insert(all_inlier_indices.end(), initial_indices.begin(), initial_indices.end());
	all_inlier_indices.insert(all_inlier_indices.end(), estimate_inlier_indices.begin(), estimate_inlier_indices.end());

	IndexSequenceView<SequenceView<DataT> > all_inlier_data(data_, &all_inlier_indices);
	ModelT final_model;
	if(!model_->generateCompleteModel(all_inlier_data, initial_model, &final_model)) {
#if DEBUG_RANSAC
	  std::cerr << "failed to generate complete RANSAC model" << std::endl;
#endif
	  continue;
	}

	double error = model_->getModelError(all_inlier_data, data_->size() - all_inlier_data.size(), final_model);
	if(error < best_fit) {
#if DEBUG_RANSAC
	  std::cerr << "Better fit: " << error << " (" << iteration << ")[" << all_inlier_indices.size() << "]" <<  std::endl;
#endif
	  best_fit = error;
	  *inliers_out = all_inlier_indices;
	  *model_out = final_model;
	}
	else {
#if DEBUG_RANSAC
	  std::cerr << "Worse fit: " << error << " (" << iteration << ")[" << all_inlier_indices.size() << "]" <<  std::endl;
#endif
        }
      }
      else {
#if DEBUG_RANSAC
	std::cerr << "not enough ransac inliers " << estimate_inlier_indices.size() << " / " << data_->size() << std::endl;
#endif
      }
    }

    if(best_fit == std::numeric_limits<double>::infinity()) { // never found a model
#if DEBUG_RANSAC
      std::cerr << "No valid RANSAC model found" << std::endl;
#endif
      return false;
    }

#if DEBUG_RANSAC
    std::cerr << "Found RANSAC model matching " << inliers_out->size() << "/" << data_->size() << std::endl;
#endif
    return true;
  }

private:
  bool selectRandomIndices(std::vector<int>* initial_indices) {
    std::size_t number_to_select = model_->getMinimumModelSize();
    std::size_t sample_size = data_->size();
    if(sample_size < number_to_select)
      return false;

    initial_indices->resize(number_to_select, -1);
    for(int i = 0; i < number_to_select; ++i) {
      int index;
      do {
	index = rand() % sample_size;
      } while(std::find(initial_indices->begin(), initial_indices->end(), index) != initial_indices->end());
      initial_indices->at(i) = index;
    }
    return true;
  }

private:
  const RansacModel<DataT, ModelT>* model_;
  SequenceView<DataT>* data_;

  int max_iterations_;


};


}

#endif
