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

#include "real_time_correlative_scan_matcher.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

#include "eigen3/Eigen/Geometry"
#include "port.h"
#include "laser.h"
#include "rigid_transform.h"


namespace carto_release {
namespace mapping_2d {

RealTimeCorrelativeScanMatcher::RealTimeCorrelativeScanMatcher() {}

std::vector<Candidate>
RealTimeCorrelativeScanMatcher::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
//  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher::Match(
    const transform::Rigid2f& initial_pose_estimate,
    const PointCloud2D& point_cloud,
    const ProbabilityGrid& probability_grid,
    transform::Rigid2f* pose_estimate) const {
//  CHECK_NOTNULL(pose_estimate);

  const Eigen::Rotation2Df initial_rotation = initial_pose_estimate.rotation();
  const PointCloud2D rotated_point_cloud = TransformPointCloud2D(
      point_cloud, transform::Rigid2f::Rotation(initial_rotation.cast<float>().angle()));
//      Eigen::AngleAxisf( initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ());

  const SearchParameters search_parameters(
      linear_search_window_, angular_search_window_,
      rotated_point_cloud, probability_grid.limits().resolution());

  const std::vector<PointCloud2D> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan> discrete_scans = DiscretizeScans(
      probability_grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  std::vector<Candidate> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  ScoreCandidates(probability_grid, discrete_scans, search_parameters,
                  &candidates);

  const Candidate& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2f(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Df(best_candidate.orientation));
  return best_candidate.score;
}

void RealTimeCorrelativeScanMatcher::ScoreCandidates(
    const ProbabilityGrid& probability_grid,
    const std::vector<DiscreteScan>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate>* const candidates) const {
  for (Candidate& candidate : *candidates) {
    candidate.score = 0.f;
    for (const Eigen::Array2i& xy_index :
         discrete_scans[candidate.scan_index]) {
      const Eigen::Array2i proposed_xy_index(
          xy_index.x() + candidate.x_index_offset,
          xy_index.y() + candidate.y_index_offset);
      const float probability =
          probability_grid.GetProbability(proposed_xy_index);
      candidate.score += probability;
    }
    candidate.score /=
        static_cast<float>(discrete_scans[candidate.scan_index].size());
    candidate.score *=
        std::exp(-Pow2(std::hypot(candidate.x, candidate.y) *
                                   translation_delta_cost_weight_ +
                               std::abs(candidate.orientation) *
                                   rotation_delta_cost_weight_));
  }
}

SearchParameters::SearchParameters(const double linear_search_window,
                                   const double angular_search_window,
                                   const PointCloud2D& point_cloud,
                                   const double resolution)
    : resolution(resolution) {
  // We set this value to something on the order of resolution to make sure that
  // the std::acos() below is defined.
  float max_scan_range = 3.f * resolution;
  for (const Eigen::Vector2f& point : point_cloud) {
    const float range = point.norm();
    max_scan_range = std::max(range, max_scan_range);
  }
  const double kSafetyMargin = 1. - 1e-3;
  angular_perturbation_step_size =
      kSafetyMargin *
      std::acos(1. -
                Pow2(resolution) / (2. * Pow2(max_scan_range)));
  num_angular_perturbations =
      std::ceil(angular_search_window / angular_perturbation_step_size);
  num_scans = 2 * num_angular_perturbations + 1;

  const int num_linear_perturbations =
      std::ceil(linear_search_window / resolution);
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

SearchParameters::SearchParameters(const int num_linear_perturbations,
                                   const int num_angular_perturbations,
                                   const double angular_perturbation_step_size,
                                   const double resolution)
    : num_angular_perturbations(num_angular_perturbations),
      angular_perturbation_step_size(angular_perturbation_step_size),
      resolution(resolution),
      num_scans(2 * num_angular_perturbations + 1) {
  linear_bounds.reserve(num_scans);
  for (int i = 0; i != num_scans; ++i) {
    linear_bounds.push_back(
        LinearBounds{-num_linear_perturbations, num_linear_perturbations,
                     -num_linear_perturbations, num_linear_perturbations});
  }
}

void SearchParameters::ShrinkToFit(const std::vector<DiscreteScan>& scans,
                                   const CellLimits& cell_limits) {
//  CHECK_EQ(scans.size(), num_scans);
//  CHECK_EQ(linear_bounds.size(), num_scans);
  for (int i = 0; i != num_scans; ++i) {
    Eigen::Array2i min_bound = Eigen::Array2i::Zero();
    Eigen::Array2i max_bound = Eigen::Array2i::Zero();
    for (const Eigen::Array2i& xy_index : scans[i]) {
      min_bound = min_bound.min(-xy_index);
      max_bound = max_bound.max(Eigen::Array2i(cell_limits.num_x_cells - 1,
                                               cell_limits.num_y_cells - 1) -
                                xy_index);
    }
    linear_bounds[i].min_x = std::max(linear_bounds[i].min_x, min_bound.x());
    linear_bounds[i].max_x = std::min(linear_bounds[i].max_x, max_bound.x());
    linear_bounds[i].min_y = std::max(linear_bounds[i].min_y, min_bound.y());
    linear_bounds[i].max_y = std::min(linear_bounds[i].max_y, max_bound.y());
  }
}

std::vector<PointCloud2D> GenerateRotatedScans(
    const PointCloud2D& point_cloud,
    const SearchParameters& search_parameters) {
  std::vector<PointCloud2D> rotated_scans;
  rotated_scans.reserve(search_parameters.num_scans);

  double delta_theta = -search_parameters.num_angular_perturbations *
                       search_parameters.angular_perturbation_step_size;
  for (int scan_index = 0; scan_index < search_parameters.num_scans;
       ++scan_index,
           delta_theta += search_parameters.angular_perturbation_step_size) {
    rotated_scans.push_back(TransformPointCloud2D(
        point_cloud, transform::Rigid2f::Rotation(delta_theta)));
  }
  return rotated_scans;
}

std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<PointCloud2D>& scans,
    const Eigen::Translation2f& initial_translation) {
  std::vector<DiscreteScan> discrete_scans;
  discrete_scans.reserve(scans.size());
  for (const PointCloud2D& scan : scans) {
    discrete_scans.emplace_back();
    discrete_scans.back().reserve(scan.size());
    for (const Eigen::Vector2f& point : scan) {
      const Eigen::Vector2f translated_point = initial_translation * point;
      discrete_scans.back().push_back(
          map_limits.GetXYIndexOfCellContainingPoint(translated_point.x(),
                                                     translated_point.y()));
    }
  }
  return discrete_scans;
}

}  // namespace mapping_2d
}  // namespace carto_release
