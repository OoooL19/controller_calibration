//
// Created by owen on 7/13/22.
//

#ifndef LANDMARK_CALIBRATION_ROSSOLVERVISULIZER_H
#define LANDMARK_CALIBRATION_ROSSOLVERVISULIZER_H
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>

#include "utils/dataset_reader.h"
#include <boost/filesystem.hpp>
#include <cv_bridge/cv_bridge.h>

#include "solver/calibSolver.h"
#include "sim/simulator.h"
namespace calib {

  class RosSolverVisulizer {

  public:
    RosSolverVisulizer(ros::NodeHandle &nh, shared_ptr<calibSolver> sys, std::shared_ptr<simulator> sim);
    void pub_sim_info();
    void pub_op_info();

  protected:
    /// Core application of the filter system
    std::shared_ptr<calibSolver> _sys;
    std::shared_ptr<simulator> _sim;
    ///publisher on the ros
    ros::Publisher pub_points_slam;
    ros::Publisher pub_traj;
    ros::Publisher pub_init_points;
    ros::Publisher pub_init_traj;
    ros::Publisher pub_result_points;
    ros::Publisher pub_result_traj;
  };

} // calib

#endif //LANDMARK_CALIBRATION_ROSSOLVERVISULIZER_H
