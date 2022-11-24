//
// Created by owen on 7/26/22.
//
#include "solver/RosSolverVisulizer.h"
#include "solver/calibSolver.h"
#include "sim/simulator.h"
#include "utils/dataset_reader.h"
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <string>
#include <utils/sensor_data.h>
#include <cstdlib>
#include "imu_calibration_evaluation/evaluation/imu_evaluation.h"
#include "imu_calibration_evaluation/calibration/imu_calibration.h"

shared_ptr<calib::RosSolverVisulizer> viz;
shared_ptr<calib::calibSolver> sys;
shared_ptr<calib::simulator> sim;
shared_ptr<calib::imu_evaluation> imu_e;
shared_ptr<calib::imu_calibration> imu_c;
int main(int argc, char **argv) {


  ros::init(argc, argv, "sim_main");
  ros::NodeHandle nh("~");


  sys = std::make_shared<calib::calibSolver>();
  sim = std::make_shared<calib::simulator>();
  viz = std::make_shared<calib::RosSolverVisulizer>(nh, sys, sim);
//  imu_e = std::make_shared<calib::imu_evaluation>();
//  imu_c = std::make_shared<calib::imu_calibration>();


//  imu_c->input_imu();
//  imu_c->calibrate_imu();
//  imu_e->imu_evaluate();
  while(ros::ok()){

    viz->pub_sim_info();
//    viz->pub_op_info();
//    sleep(1);
  }
  return EXIT_SUCCESS;
}