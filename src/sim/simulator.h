//
// Created by owen on 7/13/22.
//

#ifndef LANDMARK_CALIBRATION_SIMULATOR_H
#define LANDMARK_CALIBRATION_SIMULATOR_H
#include <Eigen/StdVector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "init/InertialInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/LandmarkRepresentation.h"

///include ceres
#include <ceres/ceres.h>
#include "ceres/rotation.h"

// libs for gtsam
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam_unstable/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/shared_ptr.hpp>

#include <sim/BsplineSE3.h>

//#include <asdl.h>

// params
#include "utils/quat_ops.h"
namespace calib {

  class simulator {
  public:
    simulator();
    void perturb();
    void make_observation(std::vector<Eigen::VectorXd> landmark,  std::vector<Eigen::VectorXd> cam_pose);
    void make_observation_cam_stay(std::vector<Eigen::VectorXd> landmark,  std::vector<Eigen::VectorXd> imu_pose);
    void load_data(std::string path_traj, std::vector<Eigen::VectorXd> &traj);
    void transToCamToW(std::vector<Eigen::VectorXd> traj);
    void transToG2Odataset(std::vector<Eigen::VectorXd> traj);
    void rifts_transToCamToW(std::vector<Eigen::VectorXd> traj);
    void transLandmarkFrame(std::vector<Eigen::VectorXd> landmark);
    void moving_landmark(std::vector<Eigen::VectorXd> landmark, std::vector<Eigen::VectorXd> traj);
    void imu_evaluate(std::vector<Eigen::VectorXd> imu_traj);
    std::vector<Eigen::VectorXd> traj_data_gt;
    std::vector<Eigen::VectorXd> imu_traj_data;
    std::vector<Eigen::VectorXd> pose_camToW;
    std::vector<Eigen::VectorXd> traj_data;
    int counter=0;
    std::vector<Eigen::VectorXd> timestamp;
    std::vector<Eigen::VectorXd> timestamp1;
    std::vector<Eigen::VectorXd> landmark_gt;
    std::vector<Eigen::VectorXd> landmark;
    std::vector<Eigen::VectorXd> i2w;
    std::vector<Eigen::VectorXf> uvs;
    std::mt19937 gen_state_perturb;
    bool ok() { return is_running; }
  protected:
    typedef std::map<double, Eigen::Matrix<double,7,1>> Vector4d_map;
    bool is_running;

  };

} // calib

#endif //LANDMARK_CALIBRATION_SIMULATOR_H
