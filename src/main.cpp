/*****
     Copyright 2016 The carto_release Authors

     Licensed under the Apache License, Version 2.0 (the "License");
     you may not use this file except in compliance with the License.
     You may obtain a copy of the License at

          http://www.apache.org/licenses/LICENSE-2.0

     Unless required by applicable law or agreed to in writing, software
     distributed under the License is distributed on an "AS IS" BASIS,
     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
     See the License for the specific language governing permissions and
     limitations under the License.
**********/
/***********************
 * Step 1: I need to grab the laser data;
 *
 * Step 2: Turn the laser data to the right format that carto wanted;
 *
 *Step 3: Use the algorithm of realtime correlative scan matcher to get the right robot pose;
 *
 * Step 4: Use ceres to get more percise robot pose, but I don't realize this step;
 *
 * Step 5: Show the map!
*************************/

#include <iostream>
#include "port.h"
#include "Multi-SensorsDataReceive.h"
#include "rigid_transform.h"
#include "laser.h"
#include "submaps.h"
#include "laser_fan_inserter.h"
#include "real_time_correlative_scan_matcher.h"
#include "scan_matcher.h"

// use opencv to show the map, it is not neccessary in the real project!
#include <thread>
#include <mutex>
// #include<opencv/highgui.h>

#define pi 3.1415926
using namespace std;
namespace carto_release {
namespace mapping_2d{

// Carto that listens to all the sensor data that we are interested in and wires it up to the SLAM.
class Carto {
    public:
          struct InsertionResult {
              Time time;
              const Submaps* submaps;
/*
                 const mapping::Submap* matching_submap;
//              std::vector<const Submap*> insertion_submaps;
//              transform::Rigid3d tracking_to_tracking_2d;
//              transform::Rigid3d tracking_2d_to_map;
//              LaserFan laser_fan_in_tracking_2d;
*/
              transform::Rigid2f pose_estimate_2d;
        };
        Carto();
        ~Carto();
        void Initialize();
        std::unique_ptr<InsertionResult> AddHorizontalLaserFan(timespec timestamp,  const LASER_ODOM_DATA& laser_odom_data);
//         void show_map(std::vector<uint8_t> rgb , int width, int height);
//         void show_map_thread();

        std::thread t1;
    private:
        // Current 'pose_estimate_' and 'velocity_estimate_' at 'time_'.
        Time time_ = Time::min();
        transform::Rigid2f pose_predict_ = transform::Rigid2f::Identity();
        transform::Rigid2f pose_predict_increase_ = transform::Rigid2f::Identity();
        transform::Rigid2f pose_estimate_ = transform::Rigid2f::Identity();
        transform::Rigid2f last_pose_estimate_ = transform::Rigid2f::Identity();
        Time last_scan_match_time_ = Time::min();
        timespec timestamp;
        Submaps submaps_;
        RealTimeCorrelativeScanMatcher  real_time_correlative_scan_matcher_;
        ScanMatcher scan_matcher_;
        MotionFilter motion_filter_;
        std::vector<uint8_t>  rgb_;

        float angle_increase_;
        Eigen::Matrix3f covMatrix_;

        // Scan match 'laser_fan_in_tracking_2d' and fill in the
        // 'pose_observation' and 'covariance_observation' with the result.  kalman_filter::PoseCovariance* covariance_observation
        void ScanMatch(Time time, const transform::Rigid2f& pose_prediction,
                       const LaserFan& laser_fan,
                       transform::Rigid2f* pose_observation );
};

Carto::Carto() {
    timestamp.tv_sec = 0;
    timestamp.tv_nsec = 0;
    angle_increase_ = 0;
//     angle_tmp_ = 0;
    covMatrix_ = Eigen::Matrix3f::Zero();
}

Carto::~Carto() {
    if(t1.joinable()) {
        t1.join();
    }
}

/*
// void Carto::show_map(std::vector<uint8_t>  rgb , int width, int height) {
//     int img_len = height*width;
//     uint8_t *img_data = (uint8_t*)new uint8_t[img_len];
//     memset(img_data, 0, img_len);
//     int vector_idx = 0;
//     for(int r=0; r<height; r++){
//         for(int c = 0;c<width;c++){
//             img_data[r*width + c] = rgb[vector_idx];
//             vector_idx++;
//         }
//     }
//     cv::Mat image(width, height,  CV_8UC1, (void*)img_data );
//     cv::imshow("map",image);
//     cv::waitKey(10);
//     delete[] img_data;
// }
// void Carto::show_map_thread() {
//     while(1) {
//         submaps_.Mapshow(m_, rgb_, width_, height_);
//         if(rgb_.size()) {
//             show_map(rgb_, width_, height_);
//         }
//     }
// }
*/

void Carto::Initialize()
{
//     t1 = std::thread(&Carto::show_map_thread, this);
     LaserOdomUart* laser_odom_collect = new LaserOdomUart;

    struct timeval start, end, mid;
    int timeuse = 0;
    while (1)
    {
        //get the laser datas
	if(laser_odom_collect->_flag_certain) {
	    gettimeofday( &start, NULL );
	    LASER_ODOM_DATA laser_odom =  laser_odom_collect->getCurrentData();
	    //here, we should build the gobal map, but we haven't build the function to do this work!
	    timestamp = laser_odom.timestamp;
	    gettimeofday( &mid, NULL );
// 	    timeuse = 1000000 * ( mid.tv_sec - start.tv_sec ) + mid.tv_usec -start.tv_usec;
// 	    cout<<"first time cost  =         "<<timeuse/1000<<endl;
	    std::unique_ptr<InsertionResult> insertion_result = AddHorizontalLaserFan(timestamp,  laser_odom);
	    gettimeofday( &end, NULL );
	    timeuse = 1000000 * ( end.tv_sec - mid.tv_sec ) + end.tv_usec -mid.tv_usec;
// 	    cout<<"second time cost  =         "<<timeuse/1000<<endl;
	}
    }
}

std::unique_ptr<Carto::InsertionResult> Carto::AddHorizontalLaserFan(const timespec timestamp,  const LASER_ODOM_DATA& laser_odom_data)
{
    const Time time = FromRos(timestamp);
 //    const Rigid3d sensor_to_tracking =   LookupToTrackingTransformOrThrow(time, frame_id);
    LaserFan laser_fan = ToLaserFan(laser_odom_data, 0.16, 3., 1.);
    laser_fan =  TransformLaserFan(laser_fan,  transform::Rigid2f(transform::Rigid2f::Vector(-0.122, 0), 0));

//     pose_predict_increase_ = transform::Rigid2f(transform::Rigid2f::Vector(laser_odom_data.X - pose_predict_.translation().x(), laser_odom_data.Y - pose_predict_.translation().y()),
//                                                (laser_odom_data.Theta - pose_predict_.rotation().angle()));
    angle_increase_ = laser_odom_data.Theta - pose_predict_.rotation().angle();

    if(angle_increase_  > pi) {
            angle_increase_ = (-pi - pose_predict_.rotation().angle()) + (laser_odom_data.Theta - pi);
    }
    else if(angle_increase_  < -pi) {
           angle_increase_ = (pi - pose_predict_.rotation().angle()) + (laser_odom_data.Theta + pi);
    }
    if(abs(angle_increase_) >= pi/6) {
        angle_increase_ = 0;
    }
    pose_predict_increase_ = transform::Rigid2f(transform::Rigid2f::Vector(laser_odom_data.X - pose_predict_.translation().x(), laser_odom_data.Y - pose_predict_.translation().y()),
                                                angle_increase_ );
// //    cerr<<"theta     "<< pose_predict_increase_.rotation().angle()<<endl;
     pose_predict_ = transform::Rigid2f(transform::Rigid2f::Vector(laser_odom_data.X, laser_odom_data.Y),
                                       laser_odom_data.Theta);

    const transform::Rigid2f pose_prediction = transform::Rigid2f(transform::Rigid2f::Vector(pose_estimate_.translation().x() + pose_predict_increase_.translation().x(),
                                                                                             pose_estimate_.translation().y() + pose_predict_increase_.translation().y()),
                                                                                             pose_estimate_.rotation().angle()+pose_predict_increase_.rotation().angle());

    if (laser_fan.point_cloud.size()<50) {
        cerr<<"these are no enough point!"<<endl;
        return nullptr;
    }
    //do the scan match to get the robot pose
     ScanMatch(time, pose_prediction, laser_fan, &pose_estimate_);

     if(pose_estimate_.rotation().angle() >= pi) {
        pose_estimate_ = transform::Rigid2f(transform::Rigid2f::Vector(pose_estimate_.translation().x(), pose_estimate_.translation().y() ),
                                                                       pose_estimate_.rotation().angle() - 2*pi);
            cerr<<"theta    less "<<endl;
     }
     else if(pose_estimate_.rotation().angle() <= -pi) {
         pose_estimate_ = transform::Rigid2f(transform::Rigid2f::Vector(pose_estimate_.translation().x(), pose_estimate_.translation().y() ),
                                                                        pose_estimate_.rotation().angle() + 2*pi);
//         pose_estimate_.rotation().angle() = pose_estimate_.rotation().angle() + 2*pi;
             cerr<<"theta    more "<<endl;
     }

         cerr<<"correlative_scan_matcher     "<< pose_estimate_.translation().x()<<"      "<<pose_estimate_.translation().y()<<"       "<<pose_estimate_.rotation().angle()<<endl;
     //record lastest pose!
     last_pose_estimate_ = pose_estimate_;

     if (motion_filter_.IsSimilar(time, pose_estimate_)) {
//         cerr<<"waitting for robot moving!"<<endl;
         return nullptr;
     }

    const LaserFan laser_fan_updata = TransformLaserFan(laser_fan, pose_estimate_);
    submaps_.InsertLaserFan(laser_fan_updata);
    return make_unique<InsertionResult>(InsertionResult{
         time, &submaps_, pose_estimate_});
}

void Carto::ScanMatch(
    Time time, const transform::Rigid2f& pose_prediction,
    const LaserFan& laser_fan,
    transform::Rigid2f* pose_observation) {
//  const ProbabilityGrid& probability_grid =  submaps_.Get(submaps_.matching_index())->probability_grid;
  const ProbabilityGrid& probability_grid =  submaps_.Get()->probability_grid;
  // The online correlative scan matcher will refine the initial estimate for the Ceres scan matcher.
  transform::Rigid2f optimal_pose;
  transform::Rigid2f final_pose;
  AdaptiveVoxelFilter adaptive_voxel_filter;
  const PointCloud2D filtered_point_cloud =  adaptive_voxel_filter.Filter(laser_fan.point_cloud);
  real_time_correlative_scan_matcher_.Match(pose_prediction, filtered_point_cloud, probability_grid, &optimal_pose);
  *pose_observation = optimal_pose;
//    cerr<<"correlative_scan_matcher     "<< optimal_pose.translation().x()<<"      "<<optimal_pose.translation().y()<<"       "<<optimal_pose.rotation().angle()<<endl;
//  scan_matcher_.matchData(optimal_pose, filtered_point_cloud, probability_grid, final_pose, covMatrix_);
//  *pose_observation = final_pose;
//    cerr<<"scan_to_map scan_matcher     "<< final_pose.translation().x()<<"     "<<final_pose.translation().y()<<"     "<<final_pose.rotation().angle()<<endl;

}

void Run() {
    Carto carto;
    carto.Initialize();
}

}  // namespace mapping_2d
}  // namespace carto_release


int main(int argc, char *argv[]) {
    carto_release::mapping_2d::Run();
//     cerr<<"start"<<endl;
    return 0;
}








