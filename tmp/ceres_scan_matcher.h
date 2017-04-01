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

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_

#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
//#include "cartographer/common/lua_parameter_dictionary.h"
//#include "cartographer/kalman_filter/pose_tracker.h"
#include "port.h"
#include "laser.h"
//#include "cartographer/mapping_2d/probability_grid.h"
//#include "cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h"
//#include "cartographer/sensor/point_cloud.h"
#include "ceres/ceres.h"
#include "ceres/cubic_interpolation.h"

namespace carto_release {
namespace mapping_2d {

// Align scans with an existing map using Ceres.
class CeresScanMatcher {
 public:
  explicit CeresScanMatcher();
  virtual ~CeresScanMatcher();

  CeresScanMatcher(const CeresScanMatcher&) = delete;
  CeresScanMatcher& operator=(const CeresScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' and returns 'pose_estimate', 'covariance', and
  // the solver 'summary'.
  void Match(const transform::Rigid2f& previous_pose,
             const transform::Rigid2f& initial_pose_estimate,
             const PointCloud2D& point_cloud,
             const ProbabilityGrid& probability_grid,
             transform::Rigid2f* pose_estimate,
             ceres::Solver::Summary* summary) const;

 private:
//  const proto::CeresScanMatcherOptions options_;
  ceres::Solver::Options ceres_solver_options_;
  float occupied_space_cost_functor_weight_ = 20.0;
  float previous_pose_translation_delta_cost_functor_weight_ = 1.0;
  float initial_pose_estimate_rotation_delta_cost_functor_weight_ = 1e2;
};

// Computes the cost of inserting occupied space described by the point cloud
// into the map. The cost increases with the amount of free space that would be
// replaced by occupied space.
class OccupiedSpaceCostFunctor {
 public:
  // Creates an OccupiedSpaceCostFunctor using the specified map, resolution
  // level, and point cloud.
  OccupiedSpaceCostFunctor(const double scaling_factor,
                           const PointCloud2D& point_cloud,
                           const ProbabilityGrid& probability_grid)
      : scaling_factor_(scaling_factor),
        point_cloud_(point_cloud),
        probability_grid_(probability_grid) {}

  OccupiedSpaceCostFunctor(const OccupiedSpaceCostFunctor&) = delete;
  OccupiedSpaceCostFunctor& operator=(const OccupiedSpaceCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    Eigen::Matrix<T, 2, 1> translation(pose[0], pose[1]);
    Eigen::Rotation2D<T> rotation(pose[2]);
    Eigen::Matrix<T, 2, 2> rotation_matrix = rotation.toRotationMatrix();
    Eigen::Matrix<T, 3, 3> transform;
    transform << rotation_matrix, translation, T(0.), T(0.), T(1.);

    const GridArrayAdapter adapter(probability_grid_);
    ceres::BiCubicInterpolator<GridArrayAdapter> interpolator(adapter);
    const MapLimits& limits = probability_grid_.limits();

    for (size_t i = 0; i < point_cloud_.size(); ++i) {
      // Note that this is a 2D point. The third component is a scaling factor.
      const Eigen::Matrix<T, 3, 1> point((T(point_cloud_[i].x())),
                                         (T(point_cloud_[i].y())), T(1.));
      const Eigen::Matrix<T, 3, 1> world = transform * point;
      interpolator.Evaluate(
          (limits.max().x() - world[0]) / limits.resolution() - 0.5 +
              T(kPadding),
          (limits.max().y() - world[1]) / limits.resolution() - 0.5 +
              T(kPadding),
          &residual[i]);
      residual[i] = scaling_factor_ * (1. - residual[i]);
    }
    return true;
  }

 private:
  static constexpr int kPadding = INT_MAX / 4;
  class GridArrayAdapter {
   public:
    enum { DATA_DIMENSION = 1 };

    explicit GridArrayAdapter(const ProbabilityGrid& probability_grid)
        : probability_grid_(probability_grid) {}

    void GetValue(const int row, const int column, double* const value) const {
      if (row < kPadding || column < kPadding || row >= NumRows() - kPadding ||
          column >= NumCols() - kPadding) {
        *value = kMinProbability;
      } else {
        *value = static_cast<double>(probability_grid_.GetProbability(
            Eigen::Array2i(column - kPadding, row - kPadding)));
      }
    }

    int NumRows() const {
      return probability_grid_.limits().cell_limits().num_y_cells +
             2 * kPadding;
    }

    int NumCols() const {
      return probability_grid_.limits().cell_limits().num_x_cells +
             2 * kPadding;
    }

   private:
    const ProbabilityGrid& probability_grid_;
  };

  const double scaling_factor_;
  const PointCloud2D& point_cloud_;
  const ProbabilityGrid& probability_grid_;
};

// Computes the cost of rotating the initial pose estimate. Cost increases with
// the solution's distance from the initial estimate.
class RotationDeltaCostFunctor {
 public:
  // Constructs a new RotationDeltaCostFunctor for the given 'angle'.
  explicit RotationDeltaCostFunctor(const double scaling_factor,
                                    const double angle)
      : scaling_factor_(scaling_factor), angle_(angle) {}

  RotationDeltaCostFunctor(const RotationDeltaCostFunctor&) = delete;
  RotationDeltaCostFunctor& operator=(const RotationDeltaCostFunctor&) = delete;

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[2] - angle_);
    return true;
  }

 private:
  const double scaling_factor_;
  const double angle_;
};

// Computes the cost of translating the initial pose estimate. Cost increases
// with the solution's distance from the initial estimate.
class TranslationDeltaCostFunctor {
 public:
  // Constructs a new TranslationDeltaCostFunctor from the given
  // 'initial_pose_estimate' (x, y, theta).
  explicit TranslationDeltaCostFunctor(
      const double scaling_factor,
      const transform::Rigid2f& initial_pose_estimate)
      : scaling_factor_(scaling_factor),
        x_(initial_pose_estimate.translation().x()),
        y_(initial_pose_estimate.translation().y()) {}

  TranslationDeltaCostFunctor(const TranslationDeltaCostFunctor&) = delete;
  TranslationDeltaCostFunctor& operator=(const TranslationDeltaCostFunctor&) =
      delete;

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    residual[0] = scaling_factor_ * (pose[0] - x_);
    residual[1] = scaling_factor_ * (pose[1] - y_);
    return true;
  }

 private:
  const double scaling_factor_;
  const double x_;
  const double y_;
};

}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_CERES_SCAN_MATCHER_H_
