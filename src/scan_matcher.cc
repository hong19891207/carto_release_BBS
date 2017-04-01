/*//=================================================================================================
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
//================================================================================================= */
#include "scan_matcher.h"
#include "eigen3/Eigen/Geometry"
//#include "../scan/DataPointContainer.h"
//#include "../util/UtilFunctions.h"
#include "port.h"

//#include "../util/DrawInterface.h"
//#include "../util/HectorDebugInfoInterface.h"

namespace carto_release {
namespace mapping_2d {

  ScanMatcher::ScanMatcher() {}

  ScanMatcher::~ScanMatcher()  {}

  void ScanMatcher::matchData(const transform::Rigid2f& beginEstimateWorld, const PointCloud2D& dataContainer, const ProbabilityGrid& gridMapUtil,  transform::Rigid2f& endEstimateWorld, Eigen::Matrix3f& covMatrix)
  {
     Eigen::Vector3f estimate(beginEstimateWorld.translation().x(), beginEstimateWorld.translation().y(), beginEstimateWorld.rotation().angle());
     estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);

     for (int i = 0; i < maxIterations_; ++i) {
        estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
     }
     estimate[2] = normalize_angle(estimate[2]);
     endEstimateWorld =  transform::Rigid2f(transform::Rigid2f::Vector(estimate[0], estimate[1]),
                                                                          estimate[2] );
     covMatrix = H;
  }

  bool ScanMatcher::estimateTransformationLogLh(Eigen::Vector3f& estimate, const ProbabilityGrid& gridMapUtil, const PointCloud2D& dataPoints)
  {
      getCompleteHessianDerivs(estimate, dataPoints, gridMapUtil, H, dTr);

      if ((H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {
          Eigen::Vector3f searchDir (H.inverse() * dTr);
          if (searchDir[2] > 0.2f) {
              searchDir[2] = 0.2f;
//              std::cout << "SearchDir angle change too large\n";
          }
          else if (searchDir[2] < -0.2f) {
              searchDir[2] = -0.2f;
//              std::cout << "SearchDir angle change too large\n";
          }
          updateEstimatedPose(estimate, searchDir);
          return true;
      }
     return false;
  }

  void ScanMatcher::getCompleteHessianDerivs(const Eigen::Vector3f& pose, const PointCloud2D& dataPoints, const ProbabilityGrid& gridMapUtil, Eigen::Matrix3f& H, Eigen::Vector3f& dTr)
   {

//      const Eigen::Rotation2Df initial_rotation = pose[3];
//      const PointCloud2D rotated_point_cloud = TransformPointCloud2D(
//          dataPoints, transform::Rigid2f::Rotation(initial_rotation.cast<float>().angle()));

     int size = dataPoints.size();
//     Eigen::Affine2f transform(getTransformForState(pose));

     const PointCloud2D point_update = dataPoints;
     TransformPointCloud2D(point_update, transform::Rigid2f(transform::Rigid2f::Vector(pose[0], pose[1]),
                                                                                                                                                      pose[2]));
     float sinRot = sin(pose[2]);
     float cosRot = cos(pose[2]);

     H = Eigen::Matrix3f::Zero();
     dTr = Eigen::Vector3f::Zero();

     for (int i = 0; i < size; ++i) {
       const Eigen::Vector2f& currPoint(point_update[i]);
       Eigen::Vector3f transformedPointData(interpMapValueWithDerivatives( currPoint, gridMapUtil));

       float funVal = 1.0f - transformedPointData[0];
       dTr[0] += transformedPointData[1] * funVal;
       dTr[1] += transformedPointData[2] * funVal;
       float rotDeriv = ((-sinRot * currPoint.x() - cosRot * currPoint.y()) * transformedPointData[1] + (cosRot * currPoint.x() - sinRot * currPoint.y()) * transformedPointData[2]);
       dTr[2] += rotDeriv * funVal;

       H(0, 0) += Pow2(transformedPointData[1]);
       H(1, 1) += Pow2(transformedPointData[2]);
       H(2, 2) += Pow2(rotDeriv);

       H(0, 1) += transformedPointData[1] * transformedPointData[2];
       H(0, 2) += transformedPointData[1] * rotDeriv;
       H(1, 2) += transformedPointData[2] * rotDeriv;
     }

     H(1, 0) = H(0, 1);
     H(2, 0) = H(0, 2);
     H(2, 1) = H(1, 2);

   }

  void ScanMatcher::updateEstimatedPose(Eigen::Vector3f& estimate, const Eigen::Vector3f& change)
  {
    estimate += change;
  }

  Eigen::Affine2f ScanMatcher::getTransformForState(const Eigen::Vector3f& transVector)
  {
    return Eigen::Translation2f(transVector[0], transVector[1]) * Eigen::Rotation2Df(transVector[2]);
  }

  Eigen::Vector3f ScanMatcher::interpMapValueWithDerivatives(const Eigen::Vector2f& coords, const ProbabilityGrid& gridMapUtil)
    {
      //check if coords are within map limits.
 /*     if (concreteGridMap->pointOutOfMapBounds(coords)){
        return Eigen::Vector3f(0.0f, 0.0f, 0.0f);
      }
      //map coords are always positive, floor them by casting to int
      Eigen::Vector2i indMin(coords.cast<int>());

      //get factors for bilinear interpolation
      Eigen::Vector2f factors(coords - indMin.cast<float>());

      int sizeX = gridMapUtil.limits().cell_limits().num_x_cells;
      int index = indMin[1] * sizeX + indMin[0];

      Eigen::Vector2f tmp_point;
      tmp_point[0] = (coords[0] + 0.025)/0.05;
      tmp_point[1] = (coords[1] + 0.025)/0.05;

//      Eigen::Vector2i indMin(tmp_point.cast<int>());
//      Eigen::Vector2f factors;
//      (tmp_point - indMin.cast<float>());
*/
      // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
      // filter gridPoint with gaussian and store in cache.
      float intensities[4] = {0, 0, 0, 0};
      Eigen::Array2i index_update =  gridMapUtil.limits().GetXYIndexOfCellContainingPoint(double(coords[0] + 0.025), double(coords[1] + 0.025));
      Eigen::Vector2f factors_trans = gridMapUtil.limits().GetPointCoordinatofXYIndex(index_update);
      //diff from the origin coordinate;
      Eigen::Vector2f factors;
      factors[0] = 20*(-coords[1] + factors_trans[1]);
      factors[1] = 20*(-coords[0] + factors_trans[0]);

      // Returns the probability of the cell with 'xy_index'.
      intensities[0] = gridMapUtil.GetProbability(index_update);

      index_update[0] += 1;
      intensities[1] = gridMapUtil.GetProbability(index_update);
      index_update[1] += 1;
      intensities[3] = gridMapUtil.GetProbability(index_update);
      index_update[0] -= 1;
      intensities[2] = gridMapUtil.GetProbability(index_update);

/*      // get grid values for the 4 grid points surrounding the current coords. Check cached data first, if not contained
      // filter gridPoint with gaussian and store in cache.
      if (!cacheMethod.containsCachedData(index, intensities[0])) {
        intensities[0] = getUnfilteredGridPoint(index);
        cacheMethod.cacheData(index, intensities[0]);
      }

      ++index;

      if (!cacheMethod.containsCachedData(index, intensities[1])) {
        intensities[1] = getUnfilteredGridPoint(index);
        cacheMethod.cacheData(index, intensities[1]);
      }

      index += sizeX-1;

      if (!cacheMethod.containsCachedData(index, intensities[2])) {
        intensities[2] = getUnfilteredGridPoint(index);
        cacheMethod.cacheData(index, intensities[2]);
      }

      ++index;

      if (!cacheMethod.containsCachedData(index, intensities[3])) {
        intensities[3] = getUnfilteredGridPoint(index);
        cacheMethod.cacheData(index, intensities[3]);
      }
*/
      float dx1 = intensities[0] - intensities[1];
      float dx2 = intensities[2] - intensities[3];

      float dy1 = intensities[0] - intensities[2];
      float dy2 = intensities[1] - intensities[3];

      float xFacInv = (1.0f - factors[0]);
      float yFacInv = (1.0f - factors[1]);

      return Eigen::Vector3f(
        ((intensities[0] * xFacInv + intensities[1] * factors[0]) * (yFacInv)) +
        ((intensities[2] * xFacInv + intensities[3] * factors[0]) * (factors[1])),
        -((dx1 * yFacInv) + (dx2 * factors[1])),
        -((dy1 * xFacInv) + (dy2 * factors[0]))
      );
    }

float ScanMatcher::normalize_angle_pos(float angle)
  {
    return fmod(fmod(angle, 2.0f*M_PI) + 2.0f*M_PI, 2.0f*M_PI);
  }

float ScanMatcher::normalize_angle(float angle)
  {
    float a = normalize_angle_pos(angle);
    if (a > M_PI){
      a -= 2.0f*M_PI;
    }
    return a;
  }

}  // namespace mapping_2d
}  // namespace carto_release

