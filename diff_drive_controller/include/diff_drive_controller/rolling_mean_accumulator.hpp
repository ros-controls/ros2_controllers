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

#ifndef DIFF_DRIVE_CONTROLLER__ROLLING_MEAN_ACCUMULATOR_HPP_
#define DIFF_DRIVE_CONTROLLER__ROLLING_MEAN_ACCUMULATOR_HPP_

#include <cassert>
#include <cstdio>
#include <vector>

namespace diff_drive_controller
{
/**
 * \brief Simplification of boost::accumulators::accumulator_set<double,
 *  bacc::stats<bacc::tag::rolling_mean>> to avoid dragging boost dependencies
 *
 * Computes the mean of the last accumulated elements
 */
template <typename T>
class RollingMeanAccumulator
{
public:
  explicit RollingMeanAccumulator(size_t rolling_window_size)
  : buffer_(rolling_window_size, 0.0), next_insert_(0), sum_(0.0), buffer_filled_(false)
  {
  }

  void accumulate(T val)
  {
    sum_ -= buffer_[next_insert_];
    sum_ += val;
    buffer_[next_insert_] = val;
    next_insert_++;
    buffer_filled_ |= next_insert_ >= buffer_.size();
    next_insert_ = next_insert_ % buffer_.size();
  }

  T getRollingMean() const
  {
    size_t valid_data_count = buffer_filled_ * buffer_.size() + !buffer_filled_ * next_insert_;
    assert(valid_data_count > 0);
    return sum_ / valid_data_count;
  }

private:
  std::vector<T> buffer_;
  size_t next_insert_;
  T sum_;
  bool buffer_filled_;
};
}  // namespace diff_drive_controller
#endif  // DIFF_DRIVE_CONTROLLER__ROLLING_MEAN_ACCUMULATOR_HPP_
