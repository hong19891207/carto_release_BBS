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

#ifndef CARTOGRAPHER_COMMON_PORT_H_
#define CARTOGRAPHER_COMMON_PORT_H_

#include <cinttypes>
#include <cmath>
#include <cstddef>
#include <memory>
#include <type_traits>
#include <utility>
#include <string>
#include <chrono>
#include <ostream>
#include <ratio>
#include "eigen3/Eigen/Core"

// #include <boost/iostreams/device/back_inserter.hpp>
// #include <boost/iostreams/filter/gzip.hpp>
// #include <boost/iostreams/filtering_stream.hpp>

using int8 = int8_t;
using int16 = int16_t;
using int32 = int32_t;
using int64 = int64_t;
using uint8 = uint8_t;
using uint16 = uint16_t;
using uint32 = uint32_t;
using uint64 = uint64_t;

using std::string;

namespace carto_release{
namespace mapping_2d {

inline int RoundToInt(const float x) { return std::lround(x); }

inline int RoundToInt(const double x) { return std::lround(x); }

inline int64 RoundToInt64(const float x) { return std::lround(x); }

inline int64 RoundToInt64(const double x) { return std::lround(x); }

// inline void FastGzipString(const string& uncompressed, string* compressed) {
//   boost::iostreams::filtering_ostream out;
//   out.push(
//       boost::iostreams::gzip_compressor(boost::iostreams::zlib::best_speed));
//   out.push(boost::iostreams::back_inserter(*compressed));
//   boost::iostreams::write(out,
//                           reinterpret_cast<const char*>(uncompressed.data()),
//                           uncompressed.size());
// }
// 
// inline void FastGunzipString(const string& compressed, string* decompressed) {
//   boost::iostreams::filtering_ostream out;
//   out.push(boost::iostreams::gzip_decompressor());
//   out.push(boost::iostreams::back_inserter(*decompressed));
//   boost::iostreams::write(out, reinterpret_cast<const char*>(compressed.data()),
//                           compressed.size());
// }


// Implementation of c++14's std::make_unique, taken from
// https://isocpp.org/files/papers/N3656.txt
template <class T>
struct _Unique_if {
  typedef std::unique_ptr<T> _Single_object;
};

template <class T>
struct _Unique_if<T[]> {
  typedef std::unique_ptr<T[]> _Unknown_bound;
};

template <class T, size_t N>
struct _Unique_if<T[N]> {
  typedef void _Known_bound;
};

template <class T, class... Args>
typename _Unique_if<T>::_Single_object make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

template <class T>
typename _Unique_if<T>::_Unknown_bound make_unique(size_t n) {
  typedef typename std::remove_extent<T>::type U;
  return std::unique_ptr<T>(new U[n]());
}

template <class T, class... Args>
typename _Unique_if<T>::_Known_bound make_unique(Args&&...) = delete;

// Clamps 'value' to be in the range ['min', 'max'].
template <typename T>
T Clamp(const T value, const T min, const T max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

// Calculates 'base'^'exponent'.
template <typename T>
constexpr T Power(T base, int exponent) {
  return (exponent != 0) ? base * Power(base, exponent - 1) : T(1);
}

// Calculates a^2.
template <typename T>
constexpr T Pow2(T a) {
      return Power(a, 2);
}

// Calculates the real part of the square root of 'a'. This is helpful when
// rounding errors generate a small negative argument. Otherwise std::sqrt
// returns NaN if its argument is negative.
template <typename T>
constexpr T RealSqrt(T a) {
  return sqrt(std::max(T(0.), a));
}

// Converts from degrees to radians.
constexpr double DegToRad(double deg) { return M_PI * deg / 180.; }

// Converts form radians to degrees.
constexpr double RadToDeg(double rad) { return 180. * rad / M_PI; }

// Bring the 'difference' between two angles into [-pi; pi].
template <typename T>
T NormalizeAngleDifference(T difference) {
  while (difference > M_PI) {
    difference -= T(2. * M_PI);
  }
  while (difference < -M_PI) {
    difference += T(2. * M_PI);
  }
  return difference;
}

template <typename T>
T atan2(const Eigen::Matrix<T, 2, 1>& vector) {
  return std::atan2(vector.y(), vector.x());
}

inline float Odds(float probability) {
  return probability / (1.f - probability);
}

inline float ProbabilityFromOdds(const float odds) {
  return odds / (odds + 1.f);
}

constexpr float kMinProbability = 0.1f;
constexpr float kMaxProbability = 1.f - kMinProbability;

// Clamps probability to be in the range [kMinProbability, kMaxProbability].
inline float ClampProbability(const float probability) {
  return Clamp(probability, kMinProbability, kMaxProbability);
}

constexpr uint16 kUnknownProbabilityValue = 0;
constexpr uint16 kUpdateMarker = 1u << 15;

// Converts a probability to a uint16 in the [1, 32767] range.
inline uint16 ProbabilityToValue(const float probability) {
  const int value =
      RoundToInt((ClampProbability(probability) - kMinProbability) *
                         (32766.f / (kMaxProbability - kMinProbability))) + 1;
  return value;
}

extern const std::vector<float>* const kValueToProbability;

// Converts a uint16 (which may or may not have the update marker set) to a
// probability in the range [kMinProbability, kMaxProbability].
inline float ValueToProbability(const uint16 value) {
  return (*kValueToProbability)[value];
}

struct CellLimits {
  CellLimits() = default;
  CellLimits(int init_num_x_cells, int init_num_y_cells)
   : num_x_cells(init_num_x_cells), num_y_cells(init_num_y_cells){}

  int num_x_cells = 0;
  int num_y_cells = 0;
};

class MapLimits {
 public:
  MapLimits(const double resolution, const Eigen::Vector2d& max,
            const CellLimits& cell_limits)
      : resolution_(resolution), max_(max), cell_limits_(cell_limits) { }

  // Returns the cell size in meters. All cells are square and the resolution is
  // the length of one side.
  double resolution() const { return resolution_; }

  // Returns the corner of the limits, i.e., all pixels have positions with
  // smaller coordinates.
  const Eigen::Vector2d& max() const { return max_; }

  // Returns the limits of the grid in number of cells.
  const CellLimits& cell_limits() const { return cell_limits_; }

  // Returns the index of the cell containing the point ('x', 'y') which may be
  // outside the map, i.e., negative or too large indices that will return
  // false for Contains().
  Eigen::Array2i GetXYIndexOfCellContainingPoint(const double x,
                                                 const double y) const {
    // Index values are row major and the top left has Eigen::Array2i::Zero()
    // and contains (centered_max_x, centered_max_y). We need to flip and
    // rotate.
    return Eigen::Array2i(
        RoundToInt((max_.y() - y) / resolution_ - 0.5),
        RoundToInt((max_.x() - x) / resolution_ - 0.5));
  }

  // Returns the point coordinate in the map when we konw it's index.
  Eigen::Vector2f GetPointCoordinatofXYIndex(const Eigen::Array2i& xy_index) const {
    return Eigen::Vector2f(
                max_.x()-(xy_index[1])*resolution_,
                max_.y()-(xy_index[0])*resolution_);
  }


  // Returns true of the ProbabilityGrid contains 'xy_index'.
  bool Contains(const Eigen::Array2i& xy_index) const {
    return (Eigen::Array2i(0, 0) <= xy_index).all() &&
           (xy_index <
            Eigen::Array2i(cell_limits_.num_x_cells, cell_limits_.num_y_cells))
               .all();
  }

  /*
  // Computes MapLimits that contain the origin, and all laser rays (both
  // returns and missing echoes) in the 'trajectory'.
//  static MapLimits ComputeMapLimits(
//      const double resolution,
//      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
//    Eigen::AlignedBox2f bounding_box = ComputeMapBoundingBox(trajectory_nodes);
//    // Add some padding to ensure all rays are still contained in the map after
//    // discretization.
//    const float kPadding = 3.f * resolution;
//    bounding_box.min() -= kPadding * Eigen::Vector2f::Ones();
//    bounding_box.max() += kPadding * Eigen::Vector2f::Ones();
//    const Eigen::Vector2d pixel_sizes =
//        bounding_box.sizes().cast<double>() / resolution;
//    return MapLimits(resolution, bounding_box.max().cast<double>(),
//                     CellLimits(common::RoundToInt(pixel_sizes.y()),
//                                common::RoundToInt(pixel_sizes.x())));
//  }

//  static Eigen::AlignedBox2f ComputeMapBoundingBox(
//      const std::vector<mapping::TrajectoryNode>& trajectory_nodes) {
//    Eigen::AlignedBox2f bounding_box(Eigen::Vector2f::Zero());
//    for (const auto& node : trajectory_nodes) {
//      const auto& data = *node.constant_data;
//      if (!data.laser_fan_3d.returns.empty()) {
//        const sensor::LaserFan3D laser_fan_3d = sensor::TransformLaserFan3D(
//            Decompress(data.laser_fan_3d), node.pose.cast<float>());
//        bounding_box.extend(laser_fan_3d.origin.head<2>());
//        for (const Eigen::Vector3f& hit : laser_fan_3d.returns) {
//          bounding_box.extend(hit.head<2>());
//        }
//      } else {
//        const sensor::LaserFan laser_fan = sensor::TransformLaserFan(
//            data.laser_fan, transform::Project2D(node.pose).cast<float>());
//        bounding_box.extend(laser_fan.origin);
//        for (const Eigen::Vector2f& hit : laser_fan.point_cloud) {
//          bounding_box.extend(hit);
//        }
//        for (const Eigen::Vector2f& miss : laser_fan.missing_echo_point_cloud) {
//          bounding_box.extend(miss);
//        }
//      }
//    }
//    return bounding_box;
//  }
*/
 private:
  double resolution_;
  Eigen::Vector2d max_;
  CellLimits cell_limits_;
};

// Represents a 2D grid of probabilities.
class ProbabilityGrid {
 public:
  explicit ProbabilityGrid(const MapLimits& limits)
      : limits_(limits),
        cells_(limits_.cell_limits().num_x_cells *
                   limits_.cell_limits().num_y_cells,
               kUnknownProbabilityValue),
        max_x_(0),
        max_y_(0),
        min_x_(limits_.cell_limits().num_x_cells - 1),
        min_y_(limits_.cell_limits().num_y_cells - 1) {}

  // Returns the limits of this ProbabilityGrid.
  const MapLimits& limits() const { return limits_; }

//   Starts the next update sequence.
  void StartUpdate() {
    while (!update_indices_.empty()) {
//      DCHECK_GE(cells_[update_indices_.back()], mapping::kUpdateMarker);
      cells_[update_indices_.back()] -= kUpdateMarker;
      update_indices_.pop_back();
    }
  }

  // Sets the probability of the cell at 'xy_index' to the given 'probability'.
  // Only allowed if the cell was unknown before.
  void SetProbability(const Eigen::Array2i& xy_index, const float probability) {
    uint16& cell = cells_[GetIndexOfCell(xy_index)];
//    CHECK_EQ(cell, mapping::kUnknownProbabilityValue);
    cell = ProbabilityToValue(probability);
    UpdateBounds(xy_index);
  }

  // Applies the 'odds' specified when calling ComputeLookupTableToApplyOdds()
  // to the probability of the cell at 'xy_index' if the cell has not already
  // been updated. Multiple updates of the same cell will be ignored until
  // StartUpdate() is called. Returns true if the cell was updated.
  //
  // If this is the first call to ApplyOdds() for the specified cell, its value
  // will be set to probability corresponding to 'odds'.
  bool ApplyLookupTable(const Eigen::Array2i& xy_index,
                        const std::vector<uint16>& table) {
//    DCHECK_EQ(table.size(), mapping::kUpdateMarker);
    const int cell_index = GetIndexOfCell(xy_index);
    uint16& cell = cells_[cell_index];
    if (cell >= kUpdateMarker) {
      return false;
    }
    update_indices_.push_back(cell_index);
    cell = table[cell];
//    DCHECK_GE(cell, mapping::kUpdateMarker);
    UpdateBounds(xy_index);
    return true;
  }

  // Returns the probability of the cell with 'xy_index'.
  float GetProbability(const Eigen::Array2i& xy_index) const {
    if (limits_.Contains(xy_index)) {
      return ValueToProbability(cells_[GetIndexOfCell(xy_index)]);
    }
    return kMinProbability;
  }

  // Returns the probability of the cell containing the point ('x', 'y').
  float GetProbability(const double x, const double y) const {
    return GetProbability(limits_.GetXYIndexOfCellContainingPoint(x, y));
  }

  // Returns true if the probability at the specified index is known.
  bool IsKnown(const Eigen::Array2i& xy_index) const {
    return limits_.Contains(xy_index) &&
           cells_[GetIndexOfCell(xy_index)] != 0;
  }

  // Fills in 'offset' and 'limits' to define a subregion of that contains all
  // known cells.
  void ComputeCroppedLimits(Eigen::Array2i* const offset,
                            CellLimits* const limits) const {
    *offset = Eigen::Array2i(min_x_, min_y_);
    *limits = CellLimits(std::max(max_x_, min_x_) - min_x_ + 1,
                         std::max(max_y_, min_y_) - min_y_ + 1);
  }

  // Grows the map as necessary to include 'x' and 'y'. This changes the meaning
  // of these coordinates going forward. This method must be called immediately
  // after 'StartUpdate', before any calls to 'ApplyLookupTable'.
  void GrowLimits(const double x, const double y) {
//    CHECK(update_indices_.empty());
    while (!limits_.Contains(limits_.GetXYIndexOfCellContainingPoint(x, y))) {
      const int x_offset = limits_.cell_limits().num_x_cells / 2;
      const int y_offset = limits_.cell_limits().num_y_cells / 2;
      const MapLimits new_limits(
          limits_.resolution(),
          limits_.max() +
              limits_.resolution() * Eigen::Vector2d(y_offset, x_offset),
          CellLimits(2 * limits_.cell_limits().num_x_cells,
                     2 * limits_.cell_limits().num_y_cells));
      const int stride = new_limits.cell_limits().num_x_cells;
      const int offset = x_offset + stride * y_offset;
      const int new_size = new_limits.cell_limits().num_x_cells *
                           new_limits.cell_limits().num_y_cells;
      std::vector<uint16> new_cells(new_size,
                                    kUnknownProbabilityValue);
      for (int i = 0; i < limits_.cell_limits().num_y_cells; ++i) {
        for (int j = 0; j < limits_.cell_limits().num_x_cells; ++j) {
          new_cells[offset + j + i * stride] =
              cells_[j + i * limits_.cell_limits().num_x_cells];
        }
      }
      cells_ = new_cells;
      limits_ = new_limits;
      min_x_ += x_offset;
      min_y_ += y_offset;
      max_x_ += x_offset;
      max_y_ += y_offset;
    }
  }

 private:
  // Returns the index of the cell at 'xy_index'.
  int GetIndexOfCell(const Eigen::Array2i& xy_index) const {
//    CHECK(limits_.Contains(xy_index)) << xy_index;
    return limits_.cell_limits().num_x_cells * xy_index.y() + xy_index.x();
  }

  void UpdateBounds(const Eigen::Array2i& xy_index) {
    min_x_ = std::min(min_x_, xy_index.x());
    min_y_ = std::min(min_y_, xy_index.y());
    max_x_ = std::max(max_x_, xy_index.x());
    max_y_ = std::max(max_y_, xy_index.y());
  }

  MapLimits limits_;
  std::vector<uint16> cells_;  // Highest bit is update marker.
  std::vector<int> update_indices_;

  // Minimum and maximum cell coordinates of know cells to efficiently compute
  // cropping limits.
  int max_x_;
  int max_y_;
  int min_x_;
  int min_y_;
};

}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_COMMON_PORT_H_
