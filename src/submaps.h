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

#ifndef CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
#define CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "port.h"
#include "laser.h"
#include "rigid_transform.h"
#include "laser_fan_inserter.h"

namespace carto_release {
namespace mapping_2d {

ProbabilityGrid ComputeCroppedProbabilityGrid(
    const ProbabilityGrid& probability_grid);

// An individual submap, which has an initial position 'origin', keeps track of
// which laser fans where inserted into it, and sets the
// 'finished_probability_grid' to be used for loop closing once the map no
// longer changes.
struct Submap_f {
  Submap_f(const Eigen::Vector2f& origin)
      : origin(origin) {}

  // Origin of this submap.
  Eigen::Vector2f origin;

  /*
  // This Submap contains LaserFans with indices in the range
  // ['begin_laser_fan_index', 'end_laser_fan_index').
  int begin_laser_fan_index;
  int end_laser_fan_index;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertLaserFan() will change the submap.
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
  */
};

struct Submap : public Submap_f{
//  Submap(const Eigen::Vector2f start);
  Submap(const MapLimits& limits, const Eigen::Vector2f& origin);
  ProbabilityGrid probability_grid;
};

/*
struct Submap {
  Submap(const Eigen::Vector3f& origin, int begin_laser_fan_index)
      : origin(origin),
        begin_laser_fan_index(begin_laser_fan_index),
        end_laser_fan_index(begin_laser_fan_index) {}

  transform::Rigid3d local_pose() const {
    return transform::Rigid3d::Translation(origin.cast<double>());
  }

  // Origin of this submap.
  Eigen::Vector3f origin;

  // This Submap contains LaserFans with indices in the range
  // ['begin_laser_fan_index', 'end_laser_fan_index').
  int begin_laser_fan_index;
  int end_laser_fan_index;

  // The 'finished_probability_grid' when this submap is finished and will not
  // change anymore. Otherwise, this is nullptr and the next call to
  // InsertLaserFan() will change the submap.
  const mapping_2d::ProbabilityGrid* finished_probability_grid = nullptr;
};
*/

// A container of Submaps. : public mapping::Submaps
class Submaps  {
 public:
  Submaps();

//  Submaps(const Submaps&) = delete; explicit
//  Submaps& operator=(const Submaps&) = delete;

  const Submap* Get() const;
//  int size() const override;
//  void SubmapToProto(
//      int index, const std::vector<mapping::TrajectoryNode>& trajectory_nodes,
//      const transform::Rigid3d& global_submap_pose,
//      mapping::proto::SubmapQuery::Response* response) override;

  // Inserts 'laser_fan' into the Submap collection.
  void InsertLaserFan(const LaserFan& laser_fan);
  void Mapshow(int index, std::vector<uint8_t> & data, int & m, int &n);

  std::vector<uint8_t>  rgb_;
  int map_width_;
  int map_height_;

 private:
//  void FinishSubmap(int index);
  void AddSubmap(const Eigen::Vector2f& origin);

//  const proto::SubmapsOptions options_;

  std::vector<std::unique_ptr<Submap>> submaps_;
  LaserFanInserter laser_fan_inserter_;

  float half_length_ = 20;
  float resolution_ = 0.05;
//  // Number of LaserFans inserted.
//  int num_laser_fans_ = 0;

//  // Number of LaserFans inserted since the last Submap was added.
//  int num_laser_fans_in_last_submap_ = 0;
};



}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_MAPPING_2D_SUBMAPS_H_
