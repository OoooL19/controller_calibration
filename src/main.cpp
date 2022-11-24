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

//  google::InitGoogleLogging(argv[0]);
//  if (argc != 2) {
//    std::cerr << "usage: main text file path\n";
//    return 1;
//  }

    calib::calibSolver sim_problem;

//  if (!sim_problem.loadFile(argv[1])) {
//    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
//    return 1;
//  }
    sim_problem.loadFile("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/sim_data.txt"); //* /src/landmark_calibration/src/data /home/owen/workspace/landmark_calibration/sim_data.txt /home/owen/ceres-solver/data/problem-16-22106-pre.txt*//
//    sim_problem.Normalize();
//    sim_problem.Perturb(0.1, 0.5, 0.5);
    const int point_block_size = sim_problem.point_block_size();
    const int camera_block_size = sim_problem.camera_block_size();
    double *points = sim_problem.mutable_points();
    double *cameras = sim_problem.mutable_cameras();
//    std::cout << sim_problem.num_cameras() << std::endl;
    const double* observations = sim_problem.observations();

    ceres::Problem problem;

  for (int i = 0; i < sim_problem.num_observations(); ++i) {

      ceres::CostFunction* cost_function = SnavelyReprojectionFisheyeError19::Create(
          observations[2 * i + 0], observations[2 * i + 1]);
      ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

      double *camera = cameras + camera_block_size * sim_problem.camera_index()[i];
      double *point = points + point_block_size * sim_problem.point_index()[i];
      problem.AddResidualBlock(cost_function, loss_function, camera, point);

      ///make camera pose constant
//      problem.SetParameterBlockConstant(camera);
  }

  ///default solver is SPARSE_NORMAL_CHOLESKY
    ceres::Solver::Options options;
//    options.linear_solver_type = ceres::LinearSolverType::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
//    options.update_state_every_iteration = true;
//    options.trust_region_strategy_type = ceres::DOGLEG;
//    options.num_threads = 8;
//    options.max_num_iterations = 500;
//    options.linear_solver_ordering;
  sim_problem.WriteToRviz(sys->init_led, sys->init_path);
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
  sim_problem.WriteToRviz(sys->result_led, sys->result_path);
    std::cout << summary.FullReport() << "\n";
//  for (int j = 0; j < sys->result_led.size(); ++j) {
//    std::cout << RED << sys->result_led.at(j) << std::endl;
//  }
  while(ros::ok()){
    viz->pub_op_info();
    viz->pub_sim_info();
    sleep(1);
  }
  return EXIT_SUCCESS;
}