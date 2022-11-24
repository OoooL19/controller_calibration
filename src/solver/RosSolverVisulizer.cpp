//
// Created by owen on 7/13/22.
//

#include "RosSolverVisulizer.h"

namespace calib {
  RosSolverVisulizer::RosSolverVisulizer(ros::NodeHandle &nh, std::shared_ptr<calibSolver> sys, std::shared_ptr<simulator> sim): _sys(sys), _sim(sim){
      pub_points_slam = nh.advertise<sensor_msgs::PointCloud2>("/landmark_calibration/points_slam", 2);
      ROS_INFO("Publishing: %s", pub_points_slam.getTopic().c_str());
      pub_traj = nh.advertise<nav_msgs::Path>("/landmark_calibration/cam_trj", 2);
      ROS_INFO("Publishing: %s", pub_traj.getTopic().c_str());
      pub_init_points = nh.advertise<sensor_msgs::PointCloud2>("/landmark_calibration/init_points_slam", 2);
      ROS_INFO("Publishing: %s", pub_init_points.getTopic().c_str());
      pub_init_traj = nh.advertise<nav_msgs::Path>("/landmark_calibration/init_cam_trj", 2);
      ROS_INFO("Publishing: %s", pub_init_traj.getTopic().c_str());
      pub_result_points = nh.advertise<sensor_msgs::PointCloud2>("/landmark_calibration/result_points_slam", 2);
      ROS_INFO("Publishing: %s", pub_result_points.getTopic().c_str());
      pub_result_traj = nh.advertise<nav_msgs::Path>("/landmark_calibration/result_cam_trj", 2);
      ROS_INFO("Publishing: %s", pub_result_traj.getTopic().c_str());
      cout << "Publisher init" << endl;
  }
  void RosSolverVisulizer::pub_op_info() {
    nav_msgs::Path arr_GT;
    arr_GT.header.stamp = ros::Time::now();
    arr_GT.header.seq = 1;
    arr_GT.header.frame_id = "global";
    for (auto pt: _sys->init_path) {
      geometry_msgs::PoseStamped pose_gt;
      pose_gt.pose.orientation.x = pt(3);
      pose_gt.pose.orientation.y = pt(4);
      pose_gt.pose.orientation.z = pt(5);
      pose_gt.pose.orientation.w = pt(6);
      pose_gt.pose.position.x = pt(0);
      pose_gt.pose.position.y = pt(1);
      pose_gt.pose.position.z = pt(2);
      arr_GT.poses.push_back(pose_gt);
    }
    pub_init_traj.publish(arr_GT);

    // Get our good features
    std::vector<Eigen::VectorXd> feats_slam = _sys->init_led;

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SLAM;
    cloud_SLAM.header.frame_id = "global";
    cloud_SLAM.header.stamp = ros::Time::now();
    cloud_SLAM.width  = 3*feats_slam.size();
    cloud_SLAM.height = 1;
    cloud_SLAM.is_bigendian = false;
    cloud_SLAM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SLAM(cloud_SLAM);
    modifier_SLAM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SLAM.resize(3*feats_slam.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SLAM(cloud_SLAM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SLAM(cloud_SLAM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SLAM(cloud_SLAM, "z");

    // Fill our iterators
    for(const auto &pt : feats_slam) {
      *out_x_SLAM = pt(0); ++out_x_SLAM;
      *out_y_SLAM = pt(1); ++out_y_SLAM;
      *out_z_SLAM = pt(2); ++out_z_SLAM;
    }

    // Publish
    pub_init_points.publish(cloud_SLAM);

    nav_msgs::Path arr_GT1;
    arr_GT1.header.stamp = ros::Time::now();
    arr_GT1.header.seq = 1;
    arr_GT1.header.frame_id = "global";
    for (auto pt: _sys->result_path) {
      geometry_msgs::PoseStamped pose_gt;
      pose_gt.pose.orientation.x = pt(3);
      pose_gt.pose.orientation.y = pt(4);
      pose_gt.pose.orientation.z = pt(5);
      pose_gt.pose.orientation.w = pt(6);
      pose_gt.pose.position.x = pt(0);
      pose_gt.pose.position.y = pt(1);
      pose_gt.pose.position.z = pt(2);
      arr_GT1.poses.push_back(pose_gt);
    }
    pub_result_traj.publish(arr_GT1);

    // Get our good features
    std::vector<Eigen::VectorXd> feats_slam1 = _sys->result_led;

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SLAM1;
    cloud_SLAM1.header.frame_id = "global";
    cloud_SLAM1.header.stamp = ros::Time::now();
    cloud_SLAM1.width  = 3*feats_slam1.size();
    cloud_SLAM1.height = 1;
    cloud_SLAM1.is_bigendian = false;
    cloud_SLAM1.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SLAM1(cloud_SLAM1);
    modifier_SLAM1.setPointCloud2FieldsByString(1,"xyz");
    modifier_SLAM1.resize(3*feats_slam1.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SLAM1(cloud_SLAM1, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SLAM1(cloud_SLAM1, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SLAM1(cloud_SLAM1, "z");

    // Fill our iterators
    for(const auto &pt : feats_slam1) {
      *out_x_SLAM1 = pt(0); ++out_x_SLAM1;
      *out_y_SLAM1 = pt(1); ++out_y_SLAM1;
      *out_z_SLAM1 = pt(2); ++out_z_SLAM1;
    }

    // Publish
    pub_result_points.publish(cloud_SLAM1);
  }
  void RosSolverVisulizer::pub_sim_info() {
//    nav_msgs::Path arr_GT;
//    arr_GT.header.stamp = ros::Time::now();
//    arr_GT.header.seq = 1;
//    arr_GT.header.frame_id = "global";
//    for (auto pt: _sim->pose_camToW) {
//      geometry_msgs::PoseStamped pose_gt;
//      pose_gt.pose.orientation.x = pt(4);
//      pose_gt.pose.orientation.y = pt(5);
//      pose_gt.pose.orientation.z = pt(6);
//      pose_gt.pose.orientation.w = pt(7);
//      pose_gt.pose.position.x = pt(1);
//      pose_gt.pose.position.y = pt(2);
//      pose_gt.pose.position.z = pt(3);
//      arr_GT.poses.push_back(pose_gt);
//    }
//    pub_result_traj.publish(arr_GT);

    nav_msgs::Path arr_GT1;
    arr_GT1.header.stamp = ros::Time::now();
    arr_GT1.header.seq = 1;
    arr_GT1.header.frame_id = "global";
    for (auto pt: _sim->traj_data_gt) {
      geometry_msgs::PoseStamped pose_gt;
      pose_gt.pose.orientation.x = pt(3);
      pose_gt.pose.orientation.y = pt(4);
      pose_gt.pose.orientation.z = pt(5);
      pose_gt.pose.orientation.w = pt(6);
      pose_gt.pose.position.x = pt(0);
      pose_gt.pose.position.y = pt(1);
      pose_gt.pose.position.z = pt(2);
      arr_GT1.poses.push_back(pose_gt);
    }
    pub_traj.publish(arr_GT1);

    // Get our good features
    std::vector<Eigen::VectorXd> feats_slam = _sim->landmark;

    // Declare message and sizes
    sensor_msgs::PointCloud2 cloud_SLAM;
    cloud_SLAM.header.frame_id = "global";
    cloud_SLAM.header.stamp = ros::Time::now();
    cloud_SLAM.width  = 3*feats_slam.size();
    cloud_SLAM.height = 1;
    cloud_SLAM.is_bigendian = false;
    cloud_SLAM.is_dense = false; // there may be invalid points

    // Setup pointcloud fields
    sensor_msgs::PointCloud2Modifier modifier_SLAM(cloud_SLAM);
    modifier_SLAM.setPointCloud2FieldsByString(1,"xyz");
    modifier_SLAM.resize(3*feats_slam.size());

    // Iterators
    sensor_msgs::PointCloud2Iterator<float> out_x_SLAM(cloud_SLAM, "x");
    sensor_msgs::PointCloud2Iterator<float> out_y_SLAM(cloud_SLAM, "y");
    sensor_msgs::PointCloud2Iterator<float> out_z_SLAM(cloud_SLAM, "z");
    // Fill our iterators
    for(const auto &pt : feats_slam) {
      *out_x_SLAM = pt(0); ++out_x_SLAM;
      *out_y_SLAM = pt(1); ++out_y_SLAM;
      *out_z_SLAM = pt(2); ++out_z_SLAM;
    }

    // Publish
    pub_points_slam.publish(cloud_SLAM);



  }
} // calib