//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef SCAN_MATCHER_H_
#define SCAN_MATCHER_H_

#include "eigen3/Eigen/Geometry"
//#include "../util/UtilFunctions.h"
#include "port.h"
#include "laser.h"
#include "rigid_transform.h"

//#include "../util/DrawInterface.h"
//#include "../util/HectorDebugInfoInterface.h"

namespace carto_release {
namespace mapping_2d {

//template<typename ConcreteOccGridMapUtil>
class ScanMatcher
{
public:

  ScanMatcher();
  ~ScanMatcher();
  void matchData(const transform::Rigid2f& beginEstimateWorld, const PointCloud2D& dataContainer, const ProbabilityGrid& gridMapUtil, transform::Rigid2f& endEstimateWorld, Eigen::Matrix3f& covMatrix);

private:
  bool estimateTransformationLogLh(Eigen::Vector3f& estimate, const ProbabilityGrid& gridMapUtil, const PointCloud2D& dataPoints);
  void getCompleteHessianDerivs(const Eigen::Vector3f& pose, const PointCloud2D& dataPoints,const  ProbabilityGrid& gridMapUtil, Eigen::Matrix3f& H, Eigen::Vector3f& dTr);
  void updateEstimatedPose(Eigen::Vector3f& estimate, const Eigen::Vector3f& change);
  Eigen::Affine2f getTransformForState(const Eigen::Vector3f& transVector);
  Eigen::Vector3f interpMapValueWithDerivatives(const Eigen::Vector2f& coords, const ProbabilityGrid& gridMapUtil);

  float normalize_angle_pos(float angle);
  float normalize_angle(float angle);

protected:
  Eigen::Vector3f dTr;
  Eigen::Matrix3f H;
  int maxIterations_ = 5;
};

}  // namespace mapping_2d
}  // namespace carto_release

#endif
