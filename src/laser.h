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

#ifndef CARTOGRAPHER_SENSOR_LASER_H_
#define CARTOGRAPHER_SENSOR_LASER_H_

#include <tuple>
#include <unordered_set>
#include <utility>

#include "port.h"
#include "Multi-SensorsDataReceive.h"
// #include "LaserUart.h"
#include "rigid_transform.h"

namespace carto_release {
namespace mapping_2d {

// using LASER::LaserUart;

typedef std::vector<Eigen::Vector2f> PointCloud2D;

// Converts 'point_cloud' to a 2D point cloud by removing the z component.
//PointCloud2D ProjectToPointCloud2D(const PointCloud& point_cloud);

// Returns a new point cloud without points that fall outside the axis-aligned
// cuboid defined by 'min' and 'max'.

// Builds a LaserFan from 'proto' and separates any beams with ranges outside
// the range ['min_range', 'max_range']. Beams beyond 'max_range' inserted into
// the 'missing_echo_point_cloud' with length 'missing_echo_ray_length'. The
// points in both clouds are stored in scan order.
struct LaserFan {
  Eigen::Vector2f origin;
  PointCloud2D point_cloud;
  PointCloud2D missing_echo_point_cloud;
};
LaserFan ToLaserFan(const LASER_ODOM_DATA& proto, float min_range,
                    float max_range, float missing_echo_ray_length);

// Transforms 'laser_fan' according to 'transform'.
LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid2f& transform);

//LaserFan TransformLaserFan(const LaserFan& laser_fan);

// Projects 'laser_fan' into 2D and crops it according to the cuboid defined by
// 'min' and 'max'.
//LaserFan ProjectCroppedLaserFan(const LaserFan3D& laser_fan,
//                                const Eigen::Vector3f& min,
//                                const Eigen::Vector3f& max);

struct PointCloudWithIntensities {
  PointCloud2D points;
  std::vector<float> intensities;
};

// Transforms 'point_cloud_2d' according to 'transform'.
PointCloud2D TransformPointCloud2D(const PointCloud2D& point_cloud_2d,
                                   const transform::Rigid2f& transform);

// Returns a new point cloud without points that fall outside the region defined
// by 'min_z' and 'max_z'.
//PointCloud Crop(const PointCloud& point_cloud, float min_z, float max_z);

// // Converts 'point_cloud' to a proto::PointCloud.
//proto::PointCloud ToProto(const PointCloud& point_cloud);

// Converts 'proto' to a PointCloud.
//PointCloud ToPointCloud(const proto::PointCloud& proto);

// Returns a voxel filtered copy of 'point_cloud' where 'size' is the length
// a voxel edge.
PointCloud2D VoxelFiltered(const PointCloud2D& point_cloud, float size);

// Voxel filter for point clouds. For each voxel, the assembled point cloud
// contains the first point that fell into it from any of the inserted point
// clouds.
class VoxelFilter {
 public:
  // 'size' is the length of a voxel edge.
  explicit VoxelFilter(float size);

//  VoxelFilter(const VoxelFilter&) = delete;
//  VoxelFilter& operator=(const VoxelFilter&) = delete;

  // Inserts a point cloud into the voxel filter.
  void InsertPointCloud(const PointCloud2D& point_cloud);

  // Returns the filtered point cloud representing the occupied voxels.
  const PointCloud2D& point_cloud() const;

 private:
  struct IntegerPairHash {
    size_t operator()(const std::pair<int64, int64>& x) const {
      const uint64 first = x.first;
      const uint64 second = x.second;
      return first ^ (first + 0x9e3779b9u + (second << 6) + (second >> 2));
    }
  };

  const float size_;
  std::unordered_set<std::pair<int64, int64>, IntegerPairHash> voxels_;
  PointCloud2D point_cloud_;
};

class AdaptiveVoxelFilter {
 public:
  explicit AdaptiveVoxelFilter();

  AdaptiveVoxelFilter(const AdaptiveVoxelFilter&) = delete;
  AdaptiveVoxelFilter& operator=(const AdaptiveVoxelFilter&) = delete;

  PointCloud2D Filter(const PointCloud2D& point_cloud) const;

 private:
//  const proto::AdaptiveVoxelFilterOptions options_;
//  float max_length_ = 0.5;
//  int min_num_points_ = 200;
};

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds);

struct UniversalTimeScaleClock {
  using rep = int64;
  using period = std::ratio<1, 10000000>;
  using duration = std::chrono::duration<rep, period>;
  using time_point = std::chrono::time_point<UniversalTimeScaleClock>;
//  using time_now = std::chrono::system_clock::now();
  static constexpr bool is_steady = true;
};

// Represents Universal Time Scale durations and timestamps which are 64-bit
// integers representing the 100 nanosecond ticks since the Epoch which is
// January 1, 1 at the start of day in UTC.
using Duration = UniversalTimeScaleClock::duration;
using Time = UniversalTimeScaleClock::time_point;
//using Time_now = UniversalTimeScaleClock::time_now();

// Convenience functions to create common::Durations.
Duration FromSeconds(double seconds);
Duration FromMilliseconds(int64 milliseconds);

// Returns the given duration in seconds.
double ToSeconds(Duration duration);

// Creates a time from a Universal Time Scale.
Time FromUniversal(int64 ticks);

// Outputs the Universal Time Scale timestamp for a given Time.
int64 ToUniversal(Time time);

constexpr int64 kUtsEpochOffsetFromUnixEpochInSeconds =
    (719162ll * 24ll * 60ll * 60ll);

//timespec ToRos(Time time);

Time FromRos(const timespec& time);

// Takes poses as input and filters them to get fewer poses.
class MotionFilter {
 public:
  explicit MotionFilter();

  // If the accumulated motion (linear, rotational, or time) is above the
  // threshold, returns false. Otherwise the relative motion is accumulated and
  // true is returned.
  bool IsSimilar(Time time, const transform::Rigid2f& pose);

 private:
//  const proto::MotionFilterOptions options_;
  int num_total_ = 0;
  int num_different_ = 0;
  Time last_time_;
  float max_distance_meters_ = 0.06;
  double max_time_seconds_ = 5.0;

  transform::Rigid2f last_pose_;
};


}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_SENSOR_LASER_H_
