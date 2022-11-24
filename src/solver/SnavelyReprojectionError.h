//
// Created by owen on 7/25/22.
//

#ifndef LANDMARK_CALIBRATION_SNAVELYREPROJECTIONERROR_H
#define LANDMARK_CALIBRATION_SNAVELYREPROJECTIONERROR_H

#include <iostream>
#include "ceres/ceres.h"
#include "ceres/rotation.h"

class SnavelyReprojectionError {
public:
  SnavelyReprojectionError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template <typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const {
    // camera[0,1,2] are the angle-axis rotation.
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    // Apply second and fourth order radial distortion.
    const T& l1 = camera[7];
    const T& l2 = camera[8];
    T r2 = xp * xp + yp * yp;
    T distortion = 1.0 + r2 * (l1 + l2 * r2);

    // Compute final projected point position.
    const T& focal = camera[6];
    T predicted_x = focal * distortion * xp;
    T predicted_y = focal * distortion * yp;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
        new SnavelyReprojectionError(observed_x, observed_y)));
  }

  double observed_x;
  double observed_y;
};
class SnavelyReprojectionFisheyeError19{
public:
  SnavelyReprojectionFisheyeError19(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template<typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const{

    T cam_temp[3] = {camera[13],camera[14],camera[15]};
    T p_temp[3];
    ceres::AngleAxisRotatePoint(cam_temp,point,p_temp);
    p_temp[0] += camera[16];
    p_temp[1] += camera[17];
    p_temp[2] += camera[18];

    T p[3];
    ceres::AngleAxisRotatePoint(camera, p_temp, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];
//    std::cout << RED << p[0] << " "<< p[1] << " " << p[2]<< std::endl;
    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    const T& fx = camera[6];
    const T& fy = camera[6];
    const T& cx = camera[7];
    const T& cy = camera[8];

    const T& k1 = camera[9];
    const T& k2 = camera[10];
    const T& k3 = camera[11];
    const T& k4 = camera[12];

    T r2=xp*xp+yp*yp;
    using std::sqrt;
    using std::atan;
    T r=sqrt(r2);

    T theta = atan(r);
    T theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta;
    T theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;
    T theta_d = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;

    T inv_r = r > T(1e-8) ? T(1.0)/r : T(1);
    T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);

    T xd=xp*cdist;
    T yd=yp*cdist;
//    T xd = (xp/r)*theta_d;
//    T yd = (yp/r)*theta_d;

    T predicted_x = fx*xd+cx;
    T predicted_y = fy*yd+cy;
//    residuals[0] = fx*xd+cx;
//    residuals[1] = fy*yd+cy;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
//    T x = (observed_x - cx)/fx;
//    T y = (observed_y - cy)/fy;
//    T x_n = x / cdist;
//    T y_n = y / cdist;
//    std::cout << RED << abs(x_n*p[2]) - abs(p[0]) << " " << abs(y_n*p[2])- abs(p[1]) <<std::endl;
    return true;
  }
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionFisheyeError19, 2, 19, 3>(
        new SnavelyReprojectionFisheyeError19(observed_x, observed_y)));
  }
private:
  double observed_x;
  double observed_y;

};
class SnavelyReprojectionFisheyeError{
public:
  SnavelyReprojectionFisheyeError(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template<typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  const T* const imu,
                  T* residuals) const{
    ///imu to device transformation
    Eigen::Matrix<double,3,3> R_ItoD = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> p_IinD;
    p_IinD <<  0.005585638, -0.007119839, 0.019845198;
    ///Rotation error
//  std::cout<< imu[0] << std::endl;
//  std::cout<< gt[0] << std::endl;
    T imu_r[3] = {imu[0],imu[1],imu[2]};
    T q_imu[4];
    ceres::AngleAxisToQuaternion(imu_r,q_imu);
    Eigen::Quaternion<T> q_{q_imu[3], q_imu[0], q_imu[1], q_imu[2]};
    Eigen::Matrix<T,3,3> imu_R = q_.toRotationMatrix();

    Eigen::Matrix<T,3,3> R_DtoG = imu_R * R_ItoD.transpose();
    Eigen::Quaternion<T> q_DG(R_DtoG);
    T q_DtoG[4] = {q_DG.w(),q_DG.x(),q_DG.y(),q_DG.z()};
    Eigen::Matrix<T,3,1> p_IinG_temp = R_DtoG*p_IinD;
    T cam_temp[3];
    ceres::QuaternionToAngleAxis(q_DtoG,cam_temp);
    T p_temp[3];
    ceres::AngleAxisRotatePoint(cam_temp,point,p_temp);
    p_temp[0] += imu[3]-p_IinG_temp[0];
    p_temp[1] += imu[4]-p_IinG_temp[1];
    p_temp[2] += imu[5]-p_IinG_temp[2];

    T p[3];
    ceres::AngleAxisRotatePoint(camera, p_temp, p);
    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    const T& fx = camera[6];
    const T& fy = camera[6];
    const T& cx = camera[7];
    const T& cy = camera[8];

    const T& k1 = camera[9];
    const T& k2 = camera[10];
    const T& k3 = camera[11];
    const T& k4 = camera[12];

    T r2=xp*xp+yp*yp;
    using std::sqrt;
    using std::atan;
    T r=sqrt(r2);

    T theta = atan(r);
    T theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta;
    T theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;
    T theta_d = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;

    T inv_r = r > T(1e-8) ? T(1.0)/r : T(1);
    T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);

    T xd=xp*cdist;
    T yd=yp*cdist;
//    T xd = (xp/r)*theta_d;
//    T yd = (yp/r)*theta_d;

    T predicted_x = fx*xd+cx;
    T predicted_y = fy*yd+cy;

    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionFisheyeError, 2, 13, 3, 6>(
        new SnavelyReprojectionFisheyeError(observed_x, observed_y)));
  }
private:
  double observed_x;
  double observed_y;
};
class SnavelyReprojectionFisheyeError13{
public:
  SnavelyReprojectionFisheyeError13(double observed_x, double observed_y)
      : observed_x(observed_x), observed_y(observed_y) {}

  template<typename T>
  bool operator()(const T* const camera,
                  const T* const point,
                  T* residuals) const{
    T p[3];
    ceres::AngleAxisRotatePoint(camera, point, p);

    // camera[3,4,5] are the translation.
    p[0] += camera[3];
    p[1] += camera[4];
    p[2] += camera[5];

    // Compute the center of distortion. The sign change comes from
    // the camera model that Noah Snavely's Bundler assumes, whereby
    // the camera coordinate system has a negative z axis.
    T xp = p[0] / p[2];
    T yp = p[1] / p[2];

    const T& fx = camera[6];
    const T& fy = camera[6];
    const T& cx = camera[7];
    const T& cy = camera[8];

    const T& k1 = camera[9];
    const T& k2 = camera[10];
    const T& k3 = camera[11];
    const T& k4 = camera[12];

    T r2=xp*xp+yp*yp;
    using std::sqrt;
    using std::atan;
    T r=sqrt(r2);

    T theta = atan(r);
    T theta2 = theta*theta, theta3 = theta2*theta, theta4 = theta2*theta2, theta5 = theta4*theta;
    T theta6 = theta3*theta3, theta7 = theta6*theta, theta8 = theta4*theta4, theta9 = theta8*theta;
    T theta_d = theta + k1*theta3 + k2*theta5 + k3*theta7 + k4*theta9;

    T inv_r = r > T(1e-8) ? T(1.0)/r : T(1);
    T cdist = r > T(1e-8) ? theta_d * inv_r : T(1);

    T xd=xp*cdist;
    T yd=yp*cdist;
//    T xd = (xp/r)*theta_d;
//    T yd = (yp/r)*theta_d;

    T predicted_x = fx*xd+cx;
    T predicted_y = fy*yd+cy;
//    residuals[0] = fx*xd+cx;
//    residuals[1] = fy*yd+cy;
    // The error is the difference between the predicted and observed position.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;
    return true;
  }
  static ceres::CostFunction* Create(const double observed_x,
                                     const double observed_y) {
    return (new ceres::AutoDiffCostFunction<SnavelyReprojectionFisheyeError13, 2, 13, 3>(
        new SnavelyReprojectionFisheyeError13(observed_x, observed_y)));
  }
private:
  double observed_x;
  double observed_y;

};
class ImuGroudTruthError {
  // (u, v): the position of the observation with respect to the image
  // center point.
public:
  template <typename T>
  bool operator()(const T* const imu,
                  const T* const gt,
                  T* residuals) const {

  ///Rotation error
//  std::cout<< imu[0] << std::endl;
//  std::cout<< gt[0] << std::endl;
  T imu_r[3] = {imu[0],imu[1],imu[2]};
  T q_imu[4];
  ceres::AngleAxisToQuaternion(imu_r,q_imu);
  Eigen::Quaternion<T> q_{q_imu[3], q_imu[0], q_imu[1], q_imu[2]};
  Eigen::Matrix<T,3,3> imu_R = q_.toRotationMatrix();

  T gt_r[3] = {gt[0], gt[1], gt[2]};
  T q_gt[4];
  ceres::AngleAxisToQuaternion(gt_r,q_gt);
  Eigen::Quaternion<T> q_GT = {q_gt[3], q_gt[0], q_gt[1], q_gt[2]};
  Eigen::Matrix<T,3,3> gt_R = q_GT.toRotationMatrix();
  Eigen::Matrix<T,3,3> theta = gt_R * imu_R.transpose();
  Eigen::Quaternion<T> q_theta(theta);
  Eigen::Matrix<T,3,1> theta_log = -q_theta.vec();
//  T q_T[4] = {q_theta.w(),q_theta.x(),q_theta.y(),q_theta.z()};
//  T axis_T[3];
//  ceres::QuaternionToAngleAxis(q_T,axis_T);
  residuals[0] = theta_log[0];
  residuals[1] = theta_log[1];
  residuals[2] = theta_log[2];

  ///Position error

  residuals[3] = gt[3]- imu[3];
  residuals[4] = gt[4]- imu[4];
  residuals[5] = gt[5]- imu[5];
    return true;
  }

  // Factory to hide the construction of the CostFunction object from
  // the client code.
//  static ceres::CostFunction* Create(const ceres::Vector observed_x,
//                                     const ceres::Vector observed_y) {
//    return (new ceres::AutoDiffCostFunction<
//        ImuGroudTruthError, 6, 6>(
//        new ImuGroudTruthError));
//  }

};

#endif //LANDMARK_CALIBRATION_SNAVELYREPROJECTIONERROR_H
