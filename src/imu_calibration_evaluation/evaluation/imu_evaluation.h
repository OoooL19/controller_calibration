//
// Created by owen on 9/13/22.
//

#ifndef LANDMARK_CALIBRATION_IMU_EVALUATION_H
#define LANDMARK_CALIBRATION_IMU_EVALUATION_H
#include <vector>
#include <string>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>


namespace calib {
  class imu_evaluation {
  public:
    imu_evaluation();

    void load_data(std::string path_traj);

    void imu_evaluate(const std::string& imu_error_handling,std::string &save_path);

    std::vector<Eigen::VectorXd> imu_traj_data;
  };
} // calib


#endif //LANDMARK_CALIBRATION_IMU_EVALUATION_H
