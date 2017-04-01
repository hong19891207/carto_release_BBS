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

#include "laser.h"

#include "rigid_transform.h"

namespace carto_release {
namespace mapping_2d {

namespace {
template <typename PointCloudType, typename TransformType>
PointCloudType Transform(const PointCloudType& point_cloud,
                         const TransformType& transform) {
  PointCloudType result;
  result.reserve(point_cloud.size());
  for (const auto& point : point_cloud) {
    result.emplace_back(transform * point);
  }
  return result;
}

/*
//PointCloud FilterByMaxRange(const PointCloud& point_cloud,
//                            const float max_range) {
//  PointCloud result;
//  for (const Eigen::Vector3f& point : point_cloud) {
//    if (point.norm() <= max_range) {
//      result.push_back(point);
//    }
//  }
//  return result;
//}
*/

// 0 is unknown, [1, 32767] maps to [kMinProbability, kMaxProbability].
float SlowValueToProbability(const uint16 value) {
//  CHECK_GE(value, 0);
//  CHECK_LE(value, 32767);
  if (value == kUnknownProbabilityValue) {
    // Unknown cells have kMinProbability.
    return kMinProbability;
  }
  const float kScale = (kMaxProbability - kMinProbability) / 32766.f;
  return value * kScale + (kMinProbability - kScale);
}

const std::vector<float>* PrecomputeValueToProbability() {
  std::vector<float>* result = new std::vector<float>;
  // Repeat two times, so that both values with and without the update marker
  // can be converted to a probability.
  for (int repeat = 0; repeat != 2; ++repeat) {
    for (int value = 0; value != 32768; ++value) {
      result->push_back(SlowValueToProbability(value));
    }
  }
  return result;
}

PointCloud2D AdaptivelyVoxelFiltered( const PointCloud2D& point_cloud) {
  float max_length = 0.5;
  int min_num_points = 160;
  if (point_cloud.size() <= min_num_points) {
    // 'point_cloud' is already sparse enough.
    return point_cloud;
  }
  PointCloud2D result = VoxelFiltered(point_cloud, max_length);
  if (result.size() >= 160) {
    // Filtering with 'max_length' resulted in a sufficiently dense point cloud.
    return result;
  }
  // Search for a 'low_length' that is known to result in a sufficiently
  // dense point cloud. We give up and use the full 'point_cloud' if reducing
  // the edge length by a factor of 1e-2 is not enough.
  for (float high_length = max_length;  high_length > 1e-2f * max_length; high_length /= 2.f) {
    float low_length = high_length / 2.f;
    result = VoxelFiltered(point_cloud, low_length);
    if (result.size() >= min_num_points) {
      // Binary search to find the right amount of filtering. 'low_length' gave
      // a sufficiently dense 'result', 'high_length' did not. We stop when the
      // edge length is at most 10% off.
      while ((high_length - low_length) / low_length > 1e-1f) {
        const float mid_length = (low_length + high_length) / 2.f;
        const PointCloud2D candidate = VoxelFiltered(point_cloud, mid_length);
        if (candidate.size() >= min_num_points) {
          low_length = mid_length;
          result = candidate;
        } else {
          high_length = mid_length;
        }
      }
      return result;
    }
  }
  return result;
}

}  // namespace

PointCloud2D TransformPointCloud2D(const PointCloud2D& point_cloud_2d,
                                   const transform::Rigid2f& transform) {
  return Transform(point_cloud_2d, transform);
}

LaserFan ToLaserFan(const LASER_ODOM_DATA& proto, const float min_range,
                    const float max_range,
                    const float missing_echo_ray_length) {
  //the origin of laserfan seems that relative position between lds and roboot platform!
  LaserFan laser_fan = {Eigen::Vector2f::Zero(), {}, {}};
  float angle = proto.angle_min_;
  for (int i=0; i<360; i++) {
    if (proto.Distance[i] > 0) {
      const float first_echo = proto.Distance[i];
      if (!std::isnan(first_echo) && first_echo >= min_range) {
        if (first_echo <= max_range) {
          laser_fan.point_cloud.push_back(Eigen::Rotation2Df(angle) *
                                          Eigen::Vector2f(first_echo, 0.f));
        }
        else {
          laser_fan.missing_echo_point_cloud.push_back(
              Eigen::Rotation2Df(angle) *
              Eigen::Vector2f(missing_echo_ray_length, 0.f));
        }
      }
    }
    angle += proto.angle_increment_;
  }
  return laser_fan;
}

//LaserFan TransformLaserFan(const LaserFan& laser_fan) {}

LaserFan TransformLaserFan(const LaserFan& laser_fan,
                           const transform::Rigid2f& transform) {
  return LaserFan{
      transform * laser_fan.origin,
      TransformPointCloud2D(laser_fan.point_cloud, transform),
      TransformPointCloud2D(laser_fan.missing_echo_point_cloud, transform)};
}


PointCloud2D VoxelFiltered(const PointCloud2D& point_cloud, const float size) {
  VoxelFilter voxel_filter(size);
  voxel_filter.InsertPointCloud(point_cloud);
  return voxel_filter.point_cloud();
}

VoxelFilter::VoxelFilter(const float size) : size_(size) {}

void VoxelFilter::InsertPointCloud(const PointCloud2D& point_cloud) {
  for (const Eigen::Vector2f& point : point_cloud) {
    if (voxels_
            .emplace(RoundToInt64(point.x() / size_),
                     RoundToInt64(point.y() / size_))
            .second) {
      point_cloud_.push_back(point);
    }
  }
}

const PointCloud2D& VoxelFilter::point_cloud() const { return point_cloud_; }

AdaptiveVoxelFilter::AdaptiveVoxelFilter() {}

PointCloud2D AdaptiveVoxelFilter::Filter(
    const PointCloud2D& point_cloud) const {
  return AdaptivelyVoxelFiltered(point_cloud);
}

const std::vector<float>* const kValueToProbability =
    PrecomputeValueToProbability();

std::vector<uint16> ComputeLookupTableToApplyOdds(const float odds) {
  std::vector<uint16> result;
  result.push_back(ProbabilityToValue(ProbabilityFromOdds(odds)) + kUpdateMarker);
  for (int cell = 1; cell != 32768; ++cell) {
    result.push_back(ProbabilityToValue(ProbabilityFromOdds(
                         odds * Odds((*kValueToProbability)[cell]))) + kUpdateMarker);
  }
  return result;
}

Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

//Time TimeNow() { return std::chrono::system_clock::now(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

//timespec ToRos(Time time) {
//  int64 uts_timestamp = ToUniversal(time);
//  int64 ns_since_unix_epoch =
//      (uts_timestamp - kUtsEpochOffsetFromUnixEpochInSeconds * 10000000ll) *
//      100ll;
//  timespec ros_time;
//  ros_time.fromNSec(ns_since_unix_epoch);
//  return ros_time;
//}

// TODO(pedrofernandez): Write test.
Time FromRos(const timespec& time) {
  // The epoch of the ICU Universal Time Scale is "0001-01-01 00:00:00.0 +0000",
  // exactly 719162 days before the Unix epoch.
  return FromUniversal(
      (time.tv_sec + kUtsEpochOffsetFromUnixEpochInSeconds) * 10000000ll +
      (time.tv_nsec + 50) / 100);  // + 50 to get the rounding correct.
}

//proto::MotionFilterOptions CreateMotionFilterOptions(
//    common::LuaParameterDictionary* const parameter_dictionary) {
//  proto::MotionFilterOptions options;
//  options.set_max_time_seconds(
//      parameter_dictionary->GetDouble("max_time_seconds"));
//  options.set_max_distance_meters(
//      parameter_dictionary->GetDouble("max_distance_meters"));
//  options.set_max_angle_radians(
//      parameter_dictionary->GetDouble("max_angle_radians"));
//  return options;
//}

MotionFilter::MotionFilter() {}

bool MotionFilter::IsSimilar(const Time time,
                             const transform::Rigid2f& pose) {
//  LOG_EVERY_N(INFO, 500) << "Motion filter reduced the number of scans to "  << 100. * num_different_ / num_total_ << "%.";
  ++num_total_;
  //std::cerr<<"we do motion filter"<<std::endl;
  if (num_total_ > 1 &&  (pose.translation() - last_pose_.translation()).norm() <=  max_distance_meters_ &&  time - last_time_ <= FromSeconds(max_time_seconds_))
//      &&           transform::GetAngle(pose.inverse() * last_pose_) <=   options_.max_angle_radians()     time - last_time_ <= common::FromSeconds(options_.max_time_seconds()) &&
  {
    return true;
  }
  last_time_ = time;
  last_pose_ = pose;
  ++num_different_;
  return false;
}


}  // namespace mapping_2d
}  // namespace carto_release
