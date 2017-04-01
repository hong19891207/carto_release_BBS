/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_
#define CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_

#include <utility>
#include <vector>

#include "port.h"
#include "laser.h"

namespace carto_release {
namespace mapping_2d {

//proto::LaserFanInserterOptions CreateLaserFanInserterOptions(
//    common::LuaParameterDictionary* parameter_dictionary);

class LaserFanInserter {
 public:
  explicit LaserFanInserter();

  LaserFanInserter(const LaserFanInserter&) = delete;
  LaserFanInserter& operator=(const LaserFanInserter&) = delete;

  // Inserts 'laser_fan' into 'probability_grid'.
  void Insert(const LaserFan& laser_fan,
              ProbabilityGrid* probability_grid) const;

  const std::vector<uint16>& hit_table() const { return hit_table_; }
  const std::vector<uint16>& miss_table() const { return miss_table_; }

 private:
//  const proto::LaserFanInserterOptions options_;
  float hit_probability_ = 0.52;
  float miss_probability_ = 0.48;
  const std::vector<uint16> hit_table_;
  const std::vector<uint16> miss_table_;
};

// Iterates in row-major order through a range of xy-indices.
class XYIndexRangeIterator
    : public std::iterator<std::input_iterator_tag, Eigen::Array2i> {
 public:
  // Constructs a new iterator for the specified range.
  XYIndexRangeIterator(const Eigen::Array2i& min_xy_index,
                       const Eigen::Array2i& max_xy_index)
      : min_xy_index_(min_xy_index),
        max_xy_index_(max_xy_index),
        xy_index_(min_xy_index) {}

  // Constructs a new iterator for everything contained in 'cell_limits'.
  explicit XYIndexRangeIterator(const CellLimits& cell_limits)
      : XYIndexRangeIterator(Eigen::Array2i::Zero(),
                             Eigen::Array2i(cell_limits.num_x_cells - 1,
                                            cell_limits.num_y_cells - 1)) {}

  XYIndexRangeIterator& operator++() {
    // This is a necessary evil. Bounds checking is very expensive and needs to
    // be avoided in production. We have unit tests that exercise this check
    // in debug mode.
//    DCHECK(*this != end());
    if (xy_index_.x() < max_xy_index_.x()) {
      ++xy_index_.x();
    } else {
      xy_index_.x() = min_xy_index_.x();
      ++xy_index_.y();
    }
    return *this;
  }

  Eigen::Array2i& operator*() { return xy_index_; }

  bool operator==(const XYIndexRangeIterator& other) const {
    return (xy_index_ == other.xy_index_).all();
  }

  bool operator!=(const XYIndexRangeIterator& other) const {
    return !operator==(other);
  }

  XYIndexRangeIterator begin() {
    return XYIndexRangeIterator(min_xy_index_, max_xy_index_);
  }

  XYIndexRangeIterator end() {
    XYIndexRangeIterator it = begin();
    it.xy_index_ = Eigen::Array2i(min_xy_index_.x(), max_xy_index_.y() + 1);
    return it;
  }

 private:
  Eigen::Array2i min_xy_index_;
  Eigen::Array2i max_xy_index_;
  Eigen::Array2i xy_index_;
};

// For each ray in 'laser_fan', calls 'hit_visitor' and 'miss_visitor' on the
// appropriate cells. Hits are handled before misses.
void CastRays(const LaserFan& laser_fan, const MapLimits& limits,
              const std::function<void(const Eigen::Array2i&)>& hit_visitor,
              const std::function<void(const Eigen::Array2i&)>& miss_visitor);

}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_MAPPING_2D_LASER_FAN_INSERTER_H_
