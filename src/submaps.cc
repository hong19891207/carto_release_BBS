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

#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>

#include "eigen3/Eigen/Geometry"
#include "submaps.h"
#include "port.h"
//#include "webp/encode.h"


namespace carto_release{
namespace mapping_2d {

namespace  {
    void WriteDebugImage(const string& filename,
                         const ProbabilityGrid& probability_grid, std::vector<uint8_t> &data, int& m, int& n) {
      constexpr int kUnknown = 128;
      const CellLimits& cell_limits =
          probability_grid.limits().cell_limits();
      const int width = cell_limits.num_x_cells;
      const int height = cell_limits.num_y_cells;
      std::vector<uint8_t> rgb;
      for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(
               probability_grid.limits().cell_limits())) {
        const uint8_t value =
            probability_grid.IsKnown(xy_index)
                ? RoundToInt(
                      (1. - probability_grid.GetProbability(xy_index)) * 255 + 0)
                : kUnknown;
        rgb.push_back(value);
      }
         data = rgb;
         m = width;
         n  = height;
/*
//      uint8_t* output = nullptr;
//      size_t output_size =
//          WebPEncodeLosslessRGB(rgb.data(), width, height, 3 * width, &output);
//      std::unique_ptr<uint8_t, void (*)(void*)> output_deleter(output, std::free);
//      std::ofstream output_file(filename, std::ios::out | std::ios::binary);
//      output_file.write(reinterpret_cast<char*>(output), output_size);
//      output_file.close();
    */
    }
}

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid) {
  Eigen::Array2i offset;
  CellLimits limits;
  probability_grid.ComputeCroppedLimits(&offset, &limits);
  const double resolution = probability_grid.limits().resolution();
  const Eigen::Vector2d max =
      probability_grid.limits().max() -
      resolution * Eigen::Vector2d(offset.y(), offset.x());
  ProbabilityGrid cropped_grid(MapLimits(resolution, max, limits));
  cropped_grid.StartUpdate();
  for (const Eigen::Array2i& xy_index : XYIndexRangeIterator(limits)) {
    if (probability_grid.IsKnown(xy_index + offset)) {
      cropped_grid.SetProbability(
          xy_index, probability_grid.GetProbability(xy_index + offset));
    }
  }
  return cropped_grid;
}

Submap::Submap(const MapLimits& limits, const Eigen::Vector2f& origin)
    : Submap_f::Submap_f(Eigen::Vector2f(0, 0)),
      probability_grid(limits) {}

Submaps::Submaps()
    : laser_fan_inserter_()
{
    map_width_ = 0;
    map_height_ = 0;
  // We always want to have at least one likelihood field which we can return,
  // and will create it at the origin in absence of a better choice.
  AddSubmap(Eigen::Vector2f::Zero());
}

void Submaps::InsertLaserFan(const LaserFan& laser_fan) {
//  ++num_laser_fans_;
//  for (const int index : insertion_indices()) {
    Submap* submap = submaps_[0].get();

    laser_fan_inserter_.Insert(laser_fan, &submap->probability_grid);
//    submap->end_laser_fan_index = num_laser_fans_;
}

const Submap* Submaps::Get() const {
    return submaps_[0].get();
}

void Submaps::Mapshow(int index, std::vector<uint8_t> & data, int & m, int &n) {
  // Crop the finished Submap before inserting a new Submap to reduce peak
  // memory usage a bit.
  Submap* submap = submaps_[0].get();
//  submap->probability_grid =   ComputeCroppedProbabilityGrid(submap->probability_grid);
//  submap->finished_probability_grid = &submap->probability_grid;
    // Output the Submap that won't be changed from now on.
    WriteDebugImage("submap" + std::to_string(index) + ".webp", submap->probability_grid, data, m, n);

}

void Submaps::AddSubmap(const Eigen::Vector2f& origin) {
  const int num_cells_per_dimension =  RoundToInt(2. * half_length_ / resolution_) + 1;
  submaps_.push_back(make_unique<Submap>(
      MapLimits(resolution_,
                origin.cast<double>() +
                    half_length_* Eigen::Vector2d::Ones(),
                CellLimits(num_cells_per_dimension, num_cells_per_dimension)),
      origin));
}

}  // namespace mapping_2d
}  // namespace cartographer
