#ifndef CAMERA_FUSION_RING_BUFFER_H
#define CAMERA_FUSION_RING_BUFFER_H

#include <deque>

#include "data_frame.h"

template <typename E>
class RingBuffer : public std::deque<E> {
 public:
  RingBuffer(int size);
  ~RingBuffer() = default;

  void PushFront(E e);
  void PushBack(E e);

 private:
  int size_;
};

template <typename E>
RingBuffer<E>::RingBuffer(int size) : size_(size) {}

template <typename E>
void RingBuffer<E>::PushFront(E e) {
  this->push_front(e);
  if (this->size() > size_) {
    this->pop_back();
  }
}

template <typename E>
void RingBuffer<E>::PushBack(E e) {
  this->push_back(e);
  if (this->size() > size_) {
    this->pop_front();
  }
}

#endif
