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

// This is an implementation of the algorithm described in "Real-Time
// Correlative Scan Matching" by Olson.
//
// The correlative scan matching algorithm is exhaustively evaluating the scan
// matching search space. As described by the paper, the basic steps are:
//
// 1) Evaluate the probability p(z|xi, m) over the entire 3D search window using
// the low-resolution table.
// 2) Find the best voxel in the low-resolution 3D space that has not already
// been considered. Denote this value as Li. If Li < Hbest, terminate: Hbest is
// the best scan matching alignment.
// 3) Evaluate the search volume inside voxel i using the high resolution table.
// Suppose the log-likelihood of this voxel is Hi. Note that Hi <= Li since the
// low-resolution map overestimates the log likelihoods. If Hi > Hbest, set
// Hbest = Hi.
//
// This can be made even faster by transforming the scan exactly once over some
// discretized range.

#ifndef CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
#define CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_

#include <iostream>
#include <memory>
#include <vector>

#include "eigen3/Eigen/Core"
#include "port.h"
#include "laser.h"

//#include "cartographer/mapping_2d/probability_grid.h"
//#include "cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h"

namespace carto_release {
namespace mapping_2d {

typedef std::vector<Eigen::Array2i> DiscreteScan;

// Describes the search space.
struct SearchParameters {
  // Linear search window in pixel offsets; bounds are inclusive.
  struct LinearBounds {
    int min_x;
    int max_x;
    int min_y;
    int max_y;
  };

  SearchParameters(double linear_search_window, double angular_search_window,
                   const PointCloud2D& point_cloud, double resolution);

  // For testing.
  SearchParameters(int num_linear_perturbations, int num_angular_perturbations,
                   double angular_perturbation_step_size, double resolution);

  // Tightens the search window as much as possible.
  void ShrinkToFit(const std::vector<DiscreteScan>& scans,
                   const CellLimits& cell_limits);

  int num_angular_perturbations;
  double angular_perturbation_step_size;
  double resolution;
  int num_scans;
  std::vector<LinearBounds> linear_bounds;  // Per rotated scans.
};

// Generates a collection of rotated scans.
std::vector<PointCloud2D> GenerateRotatedScans(
    const PointCloud2D& point_cloud,
    const SearchParameters& search_parameters);

// Translates and discretizes the rotated scans into a vector of integer
// indices.
std::vector<DiscreteScan> DiscretizeScans(
    const MapLimits& map_limits, const std::vector<PointCloud2D>& scans,
    const Eigen::Translation2f& initial_translation);

// A possible solution.
struct Candidate {
  Candidate(const int init_scan_index, const int init_x_index_offset,
            const int init_y_index_offset,
            const SearchParameters& search_parameters)
      : scan_index(init_scan_index),
        x_index_offset(init_x_index_offset),
        y_index_offset(init_y_index_offset),
        x(-y_index_offset * search_parameters.resolution),
        y(-x_index_offset * search_parameters.resolution),
        orientation((scan_index - search_parameters.num_angular_perturbations) *
                    search_parameters.angular_perturbation_step_size) {}

  // Index into the rotated scans vector.
  int scan_index = 0;

  // Linear offset from the initial pose.
  int x_index_offset = 0;
  int y_index_offset = 0;

  // Pose of this Candidate relative to the initial pose.
  double x = 0.;
  double y = 0.;
  double orientation = 0.;

  // Score, higher is better.
  float score = 0.f;

  bool operator<(const Candidate& other) const { return score < other.score; }
  bool operator>(const Candidate& other) const { return score > other.score; }
};

// An implementation of "Real-Time Correlative Scan Matching" by Olson.
class RealTimeCorrelativeScanMatcher {
 public:
  explicit RealTimeCorrelativeScanMatcher();

  RealTimeCorrelativeScanMatcher(const RealTimeCorrelativeScanMatcher&) =
      delete;
  RealTimeCorrelativeScanMatcher& operator=(
      const RealTimeCorrelativeScanMatcher&) = delete;

  // Aligns 'point_cloud' within the 'probability_grid' given an
  // 'initial_pose_estimate' then updates 'pose_estimate' with the result and
  // returns the score.
  double Match(const transform::Rigid2f& initial_pose_estimate,
               const PointCloud2D& point_cloud,
               const ProbabilityGrid& probability_grid,
               transform::Rigid2f* pose_estimate) const;

  // Computes the score for each Candidate in a collection. The cost is computed
  // as the sum of probabilities, different from the Ceres CostFunctions:
  // http://ceres-solver.org/modeling.html
  //
  // Visible for testing.
  void ScoreCandidates(const ProbabilityGrid& probability_grid,
                       const std::vector<DiscreteScan>& discrete_scans,
                       const SearchParameters& search_parameters,
                       std::vector<Candidate>* candidates) const;

 private:
  std::vector<Candidate> GenerateExhaustiveSearchCandidates(
      const SearchParameters& search_parameters) const;

 const double linear_search_window_ = 0.08;
 const double angular_search_window_ = DegToRad(10.);
 const double translation_delta_cost_weight_ = 1e-1;
 const double rotation_delta_cost_weight_ = 1e-1;

};


}  // namespace mapping_2d
}  // namespace carto_release

#endif  // CARTOGRAPHER_MAPPING_2D_SCAN_MATCHING_REAL_TIME_CORRELATIVE_SCAN_MATCHER_H_
