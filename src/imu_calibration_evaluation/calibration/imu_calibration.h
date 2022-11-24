//
// Created by owen on 9/14/22.
//

#ifndef LANDMARK_CALIBRATION_IMU_CALIBRATION_H
#define LANDMARK_CALIBRATION_IMU_CALIBRATION_H
//imu_tk
#pragma once
#include "imu_tk/include/imu_tk/calibration.h"
#include "imu_tk/include/imu_tk/base.h"
#include "imu_tk/include/imu_tk/filters.h"
#include "imu_tk/include/imu_tk/integration.h"

#include <map>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <fstream>
#include <iostream>


namespace calib {

  class imu_calibration {
  public:
    imu_calibration();
    void load_data(std::string path_traj);
    void input_imu(std::vector<Eigen::VectorXd> imu_traj_data);
    void calibrate_imu(std::string SN_number, std::string controller_id, std::string &save_path,const std::string& imu_error_handling);
  private:
    struct ImuData
    {
      double ts;
      Eigen::Vector3d gyro;
      Eigen::Vector3d acc;
    };
  public:
    std::vector<Eigen::VectorXd> imu_traj_data;
    std::vector<ImuData> m_imuDatas;
    std::vector<ImuData> m_imuRecentDatas;

  private:
    double m_last_ts = 0;
    double m_state_duration = 0;
    int m_imuId = -1;
//    double m_t_init = 15;
    double m_t_wait = 0;
    double m_static_detect_threshold = 0;
    int m_min_intervals = 0;
    bool m_print_acc_variance = false;

  };

} // calib

#endif //LANDMARK_CALIBRATION_IMU_CALIBRATION_H
