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
  int getMinimumModelSize() { return minimum_model_size_; }
  virtual bool gemerateInitialModel(const SequenceView<DataT>& data, ModelT* model_out) = 0;
  virtual bool fitsModel(const ModelT& model, const DataT& value) = 0;
  virtual bool gemerateCompleteModel(const SequenceView<DataT>& data, ModelT* model_out) = 0;
  virtual bool getModelError(const SequenceView<DataT>& data, const ModelT& model_out) = 0;


private:
  int minimum_model_size_;
};

template <class DataT, class ModelT> class Ransac {
public:
  Ransac(const RansacModel<DataT, ModelT>* model)
    : model_(model), data_(NULL), max_iterations_(30) {

  }

  void setInput(const std::vector<DataT>* data) {
    data_ = new ProxySequenceView<std::vector<DataT> >(data);
  }
  void setInput(const std::vector<DataT>* data, const std::vector<int>* indices) {
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
      if(!selectRandomIndices())
	continue;

      IndexSequenceView<SequenceView<DataT> > initial_data(data_, &initial_indices_);
      if(!model_->generateInitialModel(initial_data, &initial_model_))
	continue;

      estimate_inlier_indices_.clear();
      for(int i = 0; i < data_->size(); ++i) {
	if(std::find(initial_indices_.begin(), initial_indices_.end(), i) != initial_indices_.end())
	  continue;
	if(model_->fitsModel(*model_, (*data_)[i]))
	  estimate_inlier_indices_.push_back(i);
      }

      if(model_->enoughInliers(estimate_inlier_indices_.size())) {
	all_inlier_indices_.clear();
	all_inlier_indices_.insert(all_inlier_indices_.end(), initial_indices_.begin(), initial_indices_.end());
	all_inlier_indices_.insert(all_inlier_indices_.end(), estimate_inlier_indices_.begin(), estimate_inlier_indices_.end());

	IndexSequenceView<SequenceView<DataT> > all_inlier_data(data_, &all_inlier_indices_);
	if(!model_->generateCompleteModel(all_inlier_data, &final_model_))
	  continue;

	double error = model_->getModelError(all_inlier_data, final_model_);
	if(error < best_fit) {
	  best_fit = error;
	  *inliers_out = all_inlier_indices_;
	  *model_out = final_model_;
	}
      }
    }

    if(best_fit == std::numeric_limits<double>::infinity()) // never found a model
      return false;

    return true;
  }

private:
  bool selectRandomIndices() {
    std::size_t number_to_select = model_->getMinimumModelSize();
    std::size_t sample_size = data_->size();
    if(sample_size < number_to_select)
      return false;

    initial_indices_.resize(number_to_select);
    for(int i = 0; i < number_to_select; ++i) {
      int index;
      do {
	index = rand() % sample_size;
      } while(std::find(initial_indices_.begin(), initial_indices_.end(), index) != initial_indices_.end());
      initial_indices_[i] = index;
    }
  }

private:
  const RansacModel<DataT, ModelT>* model_;
  const SequenceView<DataT>* data_;

  std::vector<int> initial_indices_;
  ModelT initial_model_;

  int max_iterations_;

  std::vector<int> estimate_inlier_indices_;
  std::vector<int> all_inlier_indices_;

  ModelT final_model_;
};


}

#endif
