// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Víctor López
 */

#ifndef ROLLING_MEAN_ACCUMLATOR_HPP
#define ROLLING_MEAN_ACCUMLATOR_HPP

#include <vector>
#include <cassert>

namespace diff_drive_controller
{

/**
 * \brief Simplification of boost::accumulators::accumulator_set<double,
 *  bacc::stats<bacc::tag::rolling_mean>> to avoid dragging boost dependencies
 *
 * Computes the mean of the last accumulated elements
 */
template<typename T>
class RollingMeanAccumulator
{
public:
  explicit RollingMeanAccumulator(size_t rolling_window_size)
  : rolling_window_size_(rolling_window_size)
  {
    buffer_.reserve(rolling_window_size);
    next_insert_ = buffer_.begin();
    sum_ = 0.0;
  }

  void accumulate(T val)
  {
    if (buffer_.size() == rolling_window_size_) {
      sum_ -= *next_insert_;
      *next_insert_ = val;
    } else {
      next_insert_ = buffer_.insert(next_insert_, val);
    }

    next_insert_++;
    if (next_insert_ == buffer_.end()) {
      next_insert_ = buffer_.begin();
    }
    sum_ += val;
  }

  T getRollingMean() const
  {
    assert(!buffer_.empty());
    return sum_ / buffer_.size();
  }

private:
  size_t rolling_window_size_;
  std::vector<T> buffer_;
  typename std::vector<T>::iterator next_insert_;
  T sum_;
};
}
#endif // ROLLING_MEAN_ACCUMLATOR_HPP
