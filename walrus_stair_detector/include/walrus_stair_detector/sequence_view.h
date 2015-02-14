#ifndef WALRUS_STAIR_DETECTOR_SEQUENCE_VIEW_H_
#define WALRUS_STAIR_DETECTOR_SEQUENCE_VIEW_H_

#include <cstddef>
#include <vector>

namespace walrus_stair_detector {

template <class ValueType> class SequenceView {
public:
  typedef ValueType value_type;
  virtual std::size_t size() = 0;
  virtual value_type& operator[](std::size_t index) = 0;
};

template <class ContainerType> class IndexSequenceView : public SequenceView<typename ContainerType::value_type> {
public:
  typedef typename SequenceView<typename ContainerType::value_type>::value_type value_type;

  IndexSequenceView(ContainerType* container, const std::vector<int>* indices)
    : container_(container), indices_(indices) {}

  std::size_t size() {
    return indices_->size();
  }

  value_type& operator[](std::size_t index) {
    return (*container_)[(*indices_)[index]];
  }
private:
  ContainerType* container_;
  const std::vector<int>* indices_;
};

template <class ContainerType> class ProxySequenceView : public SequenceView<typename ContainerType::value_type> {
public:
  typedef typename SequenceView<typename ContainerType::value_type>::value_type value_type;

  ProxySequenceView(ContainerType* container)
    : container_(container) {}

  std::size_t size() {
    return container_->size();
  }

  value_type& operator[](std::size_t index) {
    return (*container_)[index];
  }
private:
  ContainerType* container_;
};

}


#endif
