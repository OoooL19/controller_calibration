//
// Created by owen on 7/13/22.
//

#include "calibSolver.h"
typedef Eigen::Map<Eigen::VectorXd> VectorRef;
typedef Eigen::Map<const Eigen::VectorXd> ConstVectorRef;
namespace calib {

  void calibSolver::WriteToRviz(std::vector<Eigen::VectorXd> &landmark_w, std::vector<Eigen::VectorXd> &pose) const {
    std::vector<Eigen::VectorXd>landmark;
    for (int i = 0; i < num_cameras(); ++i) {
      double angleaxis[13];
        memcpy(angleaxis, parameters_ + 13 * i, 13 * sizeof(double));
      double q[4];
      double axis[3] = {angleaxis[0],angleaxis[1],angleaxis[2]};
      ceres::AngleAxisToQuaternion(axis,q);
      Eigen::Matrix<double,7,1> new_pose;
      new_pose << angleaxis[3],angleaxis[4],angleaxis[5],q[1],q[2],q[3],q[0];
      pose.emplace_back(new_pose);
    }
    const double *points = parameters_ + camera_block_size() * num_cameras_;
    for (int i = 0; i < num_points(); ++i) {
      const double *point = points + i * point_block_size();
      Eigen::Matrix<double,3,1> p_led;
      p_led << point[0],point[1],point[2];
      landmark_w.emplace_back(p_led);
      printf("%.16g %.16g %.16g\n", p_led[0], p_led[1], p_led[2]);
    }
  }
  void calibSolver::WriteToRviz_imu_pose(std::vector<Eigen::VectorXd> &pose) const {
    std::vector<Eigen::VectorXd>landmark;
    for (int i = 0; i < num_imus(); ++i) {
      double angleaxis[6];
      memcpy(angleaxis, parameters_ + camera_block_size() * num_cameras_ + point_block_size()*num_points_ + imu_block_size()*i, 6 * sizeof(double));
      double q[4];
      double axis[3] = {angleaxis[0],angleaxis[1],angleaxis[2]};
      ceres::AngleAxisToQuaternion(axis,q);
      Eigen::Matrix<double,7,1> new_pose;
      new_pose << angleaxis[3],angleaxis[4],angleaxis[5],q[1],q[2],q[3],q[0];
      pose.emplace_back(new_pose);
    }
  }
  bool calibSolver::loadFile(const char *filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == nullptr) {
      return false;
    }

    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 19 * num_cameras_ + 3 * num_points_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }

    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    fclose(fptr);
    return true;
  }
  bool calibSolver::loadFile_imu(const char *filename) {
    FILE* fptr = fopen(filename, "r");
    if (fptr == nullptr) {
      return false;
    }
    FscanfOrDie(fptr, "%d", &num_cameras_);
    FscanfOrDie(fptr, "%d", &num_points_);
    FscanfOrDie(fptr, "%d", &num_observations_);
    FscanfOrDie(fptr, "%d", &num_imus_);
    FscanfOrDie(fptr, "%d", &num_gts_);

    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[2 * num_observations_];

    num_parameters_ = 13 * num_cameras_ + 3 * num_points_ + 6 * num_imus_ + 6 * num_gts_;
    parameters_ = new double[num_parameters_];

    for (int i = 0; i < num_observations_; ++i) {
      FscanfOrDie(fptr, "%d", camera_index_ + i);
      FscanfOrDie(fptr, "%d", point_index_ + i);
      for (int j = 0; j < 2; ++j) {
        FscanfOrDie(fptr, "%lf", observations_ + 2 * i + j);
      }
    }
    for (int i = 0; i < num_parameters_; ++i) {
      FscanfOrDie(fptr, "%lf", parameters_ + i);
    }
    fclose(fptr);
    return true;
  }
  void calibSolver::CameraToAngelAxisAndCenter(const double *camera,
                                              double *angle_axis,
                                              double *center) const {
    VectorRef angle_axis_ref(angle_axis, 3);
//    if (use_quaternions_) {
//      ceres::QuaternionToAngleAxis(camera, angle_axis);
//    } else {
      angle_axis_ref = ConstVectorRef(camera, 3);
//    }

    // c = -R't
    Eigen::VectorXd inverse_rotation = -angle_axis_ref;
    ceres::AngleAxisRotatePoint(inverse_rotation.data(),
                         camera + camera_block_size() - 10,
                         center);
    VectorRef(center, 3) *= -1.0;
  }

  void calibSolver::AngleAxisAndCenterToCamera(const double *angle_axis,
                                              const double *center,
                                              double *camera) const {
    ConstVectorRef angle_axis_ref(angle_axis, 3);
//    if (use_quaternions_) {
//      AngleAxisToQuaternion(angle_axis, camera);
//    } else {
      VectorRef(camera, 3) = angle_axis_ref;
//    }

    // t = -R * c
    ceres::AngleAxisRotatePoint(angle_axis, center, camera + camera_block_size() - 10);
    VectorRef(camera + camera_block_size() - 10, 3) *= -1.0;
  }
  void calibSolver::Normalize() {
    // Compute the marginal median of the geometry
    std::vector<double> tmp(num_points_);
    Eigen::Vector3d median;
    double *points = mutable_points();
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < num_points_; ++j) {
        tmp[j] = points[3 * j + i];
      }
      median(i) = Median(&tmp);
    }

    for (int i = 0; i < num_points_; ++i) {
      VectorRef point(points + 3 * i, 3);
      tmp[i] = (point - median).lpNorm<1>();
    }

    const double median_absolute_deviation = Median(&tmp);

    // Scale so that the median absolute deviation of the resulting
    // reconstruction is 100

    const double scale = 100.0 / median_absolute_deviation;

    // X = scale * (X - median)
    for (int i = 0; i < num_points_; ++i) {
      VectorRef point(points + 3 * i, 3);
      point = scale * (point - median);
    }

    double *cameras = mutable_cameras();
    double angle_axis[3];
    double center[3];
    for (int i = 0; i < num_cameras_; ++i) {
      double *camera = cameras + camera_block_size() * i;
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      // center = scale * (center - median)
      VectorRef(center, 3) = scale * (VectorRef(center, 3) - median);
      AngleAxisAndCenterToCamera(angle_axis, center, camera);
    }
  }
  void calibSolver::Perturb(const double rotation_sigma,
                           const double translation_sigma,
                           const double point_sigma) {
    assert(point_sigma >= 0.0);
    assert(rotation_sigma >= 0.0);
    assert(translation_sigma >= 0.0);

    double *points = mutable_points();
    if (point_sigma > 0) {
      for (int i = 0; i < num_points_; ++i) {
        PerturbPoint3(point_sigma, points + 3 * i);
      }
    }

    for (int i = 0; i < num_cameras_; ++i) {
      double *camera = mutable_cameras() + camera_block_size() * i;

      double angle_axis[3];
      double center[3];
      // Perturb in the rotation of the camera in the angle-axis
      // representation
      CameraToAngelAxisAndCenter(camera, angle_axis, center);
      if (rotation_sigma > 0.0) {
        PerturbPoint3(rotation_sigma, angle_axis);
      }
      AngleAxisAndCenterToCamera(angle_axis, center, camera);

      if (translation_sigma > 0.0)
        PerturbPoint3(translation_sigma, camera + camera_block_size() - 6);
    }
  }
} // calib