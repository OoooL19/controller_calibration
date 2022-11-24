//
// Created by owen on 7/13/22.
//

#ifndef LANDMARK_CALIBRATION_CALIBSOLVER_H
#define LANDMARK_CALIBRATION_CALIBSOLVER_H
#include <Eigen/StdVector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <cstdio>

///include ceres
#include <ceres/ceres.h>
#include "ceres/rotation.h"

#include "init/InertialInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/LandmarkRepresentation.h"

// boost
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/shared_ptr.hpp>

// params
#include "utils/quat_ops.h"
#include "SnavelyReprojectionError.h"
#include "sim/simulator.h"

namespace calib {

  class calibSolver {
  public:
    ~calibSolver(){
      delete[] point_index_;
      delete[] camera_index_;
      delete[] observations_;
      delete[] parameters_;
    }
    /// save results to text file
    void WriteToRviz(std::vector<Eigen::VectorXd> &landmark_w, std::vector<Eigen::VectorXd> &pose) const;
    void WriteToRviz_imu_pose(std::vector<Eigen::VectorXd> &pose) const;
    void Normalize();
    void CameraToAngelAxisAndCenter(const double *camera,
                                                 double *angle_axis,
                                                 double *center) const;
    void AngleAxisAndCenterToCamera(const double *angle_axis,
                                                 const double *center,
                                                 double *camera) const;
    void Perturb(const double rotation_sigma,
                              const double translation_sigma,
                              const double point_sigma);
    int num_observations() const {return num_observations_;}
    const double* observations() const { return observations_; }
    double* mutable_cameras() { return parameters_; }
    double* mutable_points() { return parameters_ + camera_block_size() * num_cameras_; }
    double* mutable_imus() { return parameters_ + camera_block_size() * num_cameras_ + point_block_size()*num_points_; }
    double* mutable_gts() { return parameters_ + camera_block_size() * num_cameras_ + point_block_size()*num_points_ + imu_block_size()*num_imus_; }
    const int* point_index()     const { return point_index_;              }
    const int* camera_index()    const { return camera_index_;             }
    int camera_block_size() const { return 13; }

    int point_block_size() const { return 3; }
    int imu_block_size() const { return 6; }
    int gt_block_size() const { return 6; }

    int num_cameras() const { return num_cameras_; }

    int num_points() const { return num_points_; }

    int num_imus() const { return num_imus_; }
    int num_gts() const { return num_gts_; }

    bool loadFile(const char* filename);
    bool loadFile_imu(const char* filename);

    std::vector<Eigen::VectorXd> init_path;
    std::vector<Eigen::VectorXd> init_led;
    std::vector<Eigen::VectorXd> result_path;
    std::vector<Eigen::VectorXd> result_led;

  private:
    template <typename T>
    void FscanfOrDie(FILE* fptr, const char* format, T* value) {
      int num_scanned = fscanf(fptr, format, value);
      if (num_scanned != 1) {
        LOG(FATAL) << "Invalid UW data file.";
      }
    }
    double Median(std::vector<double> *data) {
      int n = data->size();
      std::vector<double>::iterator mid_point = data->begin() + n / 2;
      std::nth_element(data->begin(), mid_point, data->end());
      return *mid_point;
    }
    inline double RandDouble()
    {
      double r = static_cast<double>(rand());
      return r / RAND_MAX;
    }
    inline double RandNormal()
    {
      double x1, x2, w;
      do{
        x1 = 2.0 * RandDouble() - 1.0;
        x2 = 2.0 * RandDouble() - 1.0;
        w = x1 * x1 + x2 * x2;
      }while( w >= 1.0 || w == 0.0);

      w = sqrt((-2.0 * log(w))/w);
      return x1 * w;
    }
    void PerturbPoint3(const double sigma, double *point) {
      for (int i = 0; i < 3; ++i)
        point[i] += RandNormal() * sigma;
    }
    int num_cameras_;
    int num_points_;
    int num_imus_;
    int num_gts_;
    int num_observations_;
    int num_parameters_;

    int* point_index_;
    int* camera_index_;
    double* observations_;
    double* parameters_;
  };

} // calib

#endif //LANDMARK_CALIBRATION_CALIBSOLVER_H
