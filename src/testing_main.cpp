//
// Created by owen on 7/13/22.
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

shared_ptr<calib::RosSolverVisulizer> viz;
shared_ptr<calib::calibSolver> sys;
shared_ptr<calib::simulator> sim;
int main(int argc, char **argv) {

  ros::init(argc, argv, "main");
  ros::NodeHandle nh("~");

  sys = std::make_shared<calib::calibSolver>();
  sim = std::make_shared<calib::simulator>();
  viz = std::make_shared<calib::RosSolverVisulizer>(nh, sys, sim);

    calib::calibSolver sim_problem;

    sim_problem.loadFile_imu("/home/owen/workspace/landmark_calibration/sim_data.txt"); //* /src/landmark_calibration/src/data /home/owen/workspace/landmark_calibration/sim_data.txt /home/owen/ceres-solver/data/problem-16-22106-pre.txt*//
    std::cout << sim_problem.num_imus() << std::endl;

  const int point_block_size = sim_problem.point_block_size();
  const int camera_block_size = sim_problem.camera_block_size();
  const int imu_block_size = sim_problem.imu_block_size();
  const int gt_block_size = sim_problem.gt_block_size();
  double *points = sim_problem.mutable_points();
  double *cameras = sim_problem.mutable_cameras();
  double *imus = sim_problem.mutable_imus();
  double *gts = sim_problem.mutable_gts();
  const double* observations = sim_problem.observations();
  ceres::Problem problem;
  for (int j = 0; j < sim_problem.num_cameras(); ++j) {
    ceres::CostFunction* cost_function = SnavelyReprojectionFisheyeError::Create(
        observations[2 * j + 0], observations[2 * j + 1]);
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);
    ceres::CostFunction* cost_function_imu = new ceres::AutoDiffCostFunction<ImuGroudTruthError,6,6,6>(new ImuGroudTruthError);
    double *camera = cameras + camera_block_size * sim_problem.camera_index()[j];
    double *point = points + point_block_size * sim_problem.point_index()[j];
    double *imu = imus + imu_block_size*j;
    double *gt = gts + gt_block_size*j;
    problem.AddResidualBlock(cost_function_imu, loss_function,imu,gt);
    problem.AddResidualBlock(cost_function, nullptr, camera,point, imu);

    problem.SetParameterBlockConstant(gt);
//    problem.SetParameterBlockConstant(imu);
  }
  /// Run the solver!
  ceres::Solver::Options options;
//  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  options.update_state_every_iteration = true;
  sim_problem.WriteToRviz(sys->init_led, sys->init_path);
  sim_problem.WriteToRviz_imu_pose(sys->init_path);
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  sim_problem.WriteToRviz(sys->result_led, sys->result_path);
  sim_problem.WriteToRviz_imu_pose(sys->result_path);
  std::cout << summary.FullReport() << "\n";
  while(ros::ok()){
    viz->pub_op_info();
    viz->pub_sim_info();
    sleep(1);
  }
  return EXIT_SUCCESS;
}