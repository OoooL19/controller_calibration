//
// Created by owen on 7/13/22.
//

#include "simulator.h"

namespace calib {
  simulator::simulator() {
    printf("=======================================\n");
    printf("Testing Graph STARTING\n");
    printf("=======================================\n");
    is_running = true;

//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/i2w.txt", i2w);
//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/i2w.txt", traj_data);

//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/rifts.txt", landmark);
//    landmark_gt = landmark;
//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/rifts_cam_pose.txt", traj_data);
//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/timestamp.txt", timestamp);
//    transLandmarkFrame(landmark_gt);
//    rifts_transToCamToW(traj_data);
//    transToG2Odataset(traj_data);
//
//    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/rifts.txt", landmark_gt);
    load_data("/home/owen/workspace/landmark_calibration/src/landmark_calibration/src/data/i2w.txt", traj_data_gt);
//    for (int j = 0; j < traj_data_gt.size(); ++j) {
//      traj_data_gt.at(j)[1] += .5;
//      traj_data_gt.at(j)[3] += 0.08;
//    }
//    traj_data = traj_data_gt;
//    std::normal_distribution<double> w(0, 1);
//    double noise = 0.05;
//    for (size_t i = 0; i < traj_data_gt.size(); i++) {
//
//      // Perturb the ground truth
//      traj_data.at(i)(1) += noise * w(gen_state_perturb);
//      traj_data.at(i)(2) += noise * w(gen_state_perturb);
//      traj_data.at(i)(3) += noise * w(gen_state_perturb);
//
//      Eigen::Vector3d w_vec;
//      w_vec << noise * w(gen_state_perturb), noise * w(gen_state_perturb),
//          noise * w(gen_state_perturb);
//      Eigen::Quaterniond q(`ov_core`::exp_so3(w_vec) * ov_core::quat_2_Rot(traj_data.at(i).block(4, 0, 4, 1)));
//      Eigen::Matrix<double,4,1> q_temp;
//      q_temp << q.x(),q.y(),q.z(),q.w();
//      traj_data.at(i).block(4, 0, 4, 1) = q_temp;
//    }
//    rifts_transToCamToW(traj_data_gt);

//    moving_landmark(landmark_gt,traj_data);
//    make_observation_cam_stay(landmark,traj_data);
//    make_observation(landmark_gt,traj_data_gt);
//    perturb();
    ///IMU reading
//    load_data("/media/owen/T7/手柄数据/record_2021_12_31_08.39.17/imu_1.txt",imu_traj_data);
//    imu_evaluate(imu_traj_data);
  }
  void simulator::perturb() {
    FILE *fptr1 = fopen("cam_pose.txt", "w");
    FILE *fptr2 = fopen("point.txt", "w");
    ///add noise to the camera pose
    std::normal_distribution<double> w(0, 1);
    double noise = 10;
    for (size_t i = 0; i < traj_data_gt.size(); i++) {

      // Perturb the ground truth
      traj_data.at(i)(1) += noise * w(gen_state_perturb);
      traj_data.at(i)(2) += noise * w(gen_state_perturb);
      traj_data.at(i)(3) += noise * w(gen_state_perturb);

      Eigen::Vector3d w_vec;
      w_vec << noise * w(gen_state_perturb), noise * w(gen_state_perturb),
          noise * w(gen_state_perturb);
      traj_data.at(i).block(4, 0, 4, 1) = ov_core::rot_2_quat(
          ov_core::exp_so3(w_vec) * ov_core::quat_2_Rot(traj_data.at(i).block(4, 0, 4, 1)));
    }
    for (int j = 0; j < landmark_gt.size(); ++j) {
      landmark.at(j)[0] += noise * w(gen_state_perturb);
      landmark.at(j)[1] += noise * w(gen_state_perturb);
      landmark.at(j)[2] += noise * w(gen_state_perturb);
      fprintf(fptr2, "%.16g\n", landmark.at(j)[0]);
      fprintf(fptr2, "%.16g\n", landmark.at(j)[1]);
      fprintf(fptr2, "%.16g\n", landmark.at(j)[2]);
    }
    Eigen::Matrix<double,9,1> cam_pose;

    for (int j = 0; j < traj_data.size(); ++j) {
      cam_pose << ov_core::log_so3(ov_core::quat_2_Rot(traj_data.at(j).block(4,0,4,1))), traj_data.at(j)(1), traj_data.at(j)(2), traj_data.at(j)(3),
          -0.28340811, 458.654,457.296;
      for (int k = 0; k < 9; ++k) {
        fprintf(fptr1, "%.16g \n", cam_pose(k));
      }
    }
  }
  void simulator::moving_landmark(std::vector<Eigen::VectorXd> landmark_gt, std::vector<Eigen::VectorXd> traj){
    Eigen::Matrix3d R_ItoD = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> p_IinD;
    p_IinD <<  0.005585638, -0.007119839, 0.019845198;
    Eigen::Isometry3d T_ItoD;
    T_ItoD.linear() = R_ItoD;
    T_ItoD.translation() = p_IinD;
    Eigen::Isometry3d T_DtoI = T_ItoD.inverse();
    for (int j = 0; j < traj.size(); ++j) {
        Eigen::Matrix<double, 4, 1> q_IctoG = traj.at(j).block(4,0,4,1);
        Eigen::Matrix<double, 3, 1> P_IctoG = traj.at(j).block(1,0,3,1);
        Eigen::Matrix3d R_IctoG = ov_core::quat_2_Rot(q_IctoG);
        Eigen::Isometry3d T_IctoG;
        T_IctoG.linear() = R_IctoG;
        T_IctoG.translation() = P_IctoG;
        Eigen::Isometry3d T_DtoG = T_IctoG * T_ItoD.inverse();
        Eigen::Matrix3d R = T_DtoG.linear();
        Eigen::Vector3d t = T_DtoG.translation();
        Eigen::Matrix<double, 45,1> landmark_temp;
      for (int k = 0; k < landmark_gt.size(); ++k) {
        Eigen::Matrix<double,3,1> p_LinD = landmark_gt.at(k).block(0,0,3,1);
        Eigen::Matrix<double,3, 1> p_LinG = R*p_LinD+t;
        landmark_temp.block(3*k,0,3,1) = p_LinG;
      }
      landmark.emplace_back(landmark_temp);
    }
  }
  void simulator::make_observation_cam_stay(std::vector<Eigen::VectorXd> landmark, std::vector<Eigen::VectorXd> imu_pose){
    Eigen::Matrix3d R_ItoD = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> p_IinD;
    p_IinD <<  0.005585638, -0.007119839, 0.019845198;
//    p_IinD << -0.000295, -0.0190975, -0.068692;
    Eigen::Isometry3d T_ItoD;
    T_ItoD.linear() = R_ItoD;
    T_ItoD.translation() = p_IinD;
    Vector4d_map trajectory_points;
    for (int j = 0; j < imu_pose.size(); ++j) {
      trajectory_points.insert({imu_pose.at(j)[0],imu_pose.at(j).block(1,0,7,1)});
    }
    Eigen::Matrix3d R_GtoC = Eigen::Matrix3d::Identity();
    Eigen::Vector3d P_CinG;
    P_CinG.setZero();
    FILE *fptr = fopen("sim_data.txt", "w");
    int cam_index = imu_pose.size();
    int point_index = landmark_gt.size();
    int temp = 0;
    int test = 0;
    int counter = 0;
    Eigen::Matrix<double,8,1> cam_d;
    cam_d << 191.67230216563604, 191.6996360134994, 232.9522079795456, 327.8509945573558, 0.311107805332893,
        -0.0555464401505884,-0.05907950993375161,0.025473056841047295;
    for (int j = 0; j < cam_index; ++j) {
      for (int k = 0; k < point_index; ++k) {
        Eigen::Vector3d p_FinG = landmark.at(j).block(3*k,0,3,1);
        Eigen::Vector3d p_FinC = R_GtoC * (p_FinG - P_CinG);
        Eigen::Vector2f uv_norm;
        uv_norm << p_FinC(0) / p_FinC(2), p_FinC(1) / p_FinC(2);

        // Distort the normalized coordinates (false=radtan, true=fisheye)
        Eigen::Vector2f uv_dist;

        double r = sqrt(uv_norm(0) * uv_norm(0) + uv_norm(1) * uv_norm(1));
        double theta = std::atan(r);
        double theta_d =
            theta + cam_d(4) * std::pow(theta, 3) + cam_d(5) * std::pow(theta, 5) + cam_d(6) * std::pow(theta, 7) +
            cam_d(7) * std::pow(theta, 9);

        // Handle when r is small (meaning our xy is near the camera center)
        double inv_r = r > 1e-8 ? 1.0 / r : 1;
        double cdist = r > 1e-8 ? theta_d * inv_r : 1;

        // Calculate distorted coordinates for fisheye
        double x1 = uv_norm(0) * cdist;
        double y1 = uv_norm(1) * cdist;
        uv_dist(0) = cam_d(0) * x1 + cam_d(2);
        uv_dist(1) = cam_d(1) * y1 + cam_d(3);
        if (abs(uv_dist(0))<=640 && abs(uv_dist(1))<=480) {
//          printf("%d %d %f %f \n", counter, k, uv_dist(0), uv_dist(1));
          fprintf(fptr, "%d %d %f %f\n", counter, k, uv_dist(0), uv_dist(1));
//          uvs.emplace_back(uv_dist);
          temp = j;
        }
      }
      if (test != temp) {
        test = temp;
//        std::cout << counter << std::endl;
        counter++;
        timestamp.emplace_back(imu_pose.at(test).block(0, 0, 1, 1));
      }
    }
    for (int j = 0; j < timestamp.size(); ++j) {
      Eigen::Quaterniond q_GtoC(R_GtoC);
      double q_c[4] = {q_GtoC.w(),q_GtoC.x(),q_GtoC.y(),q_GtoC.z()};
      double axis_c[3];
      ceres::QuaternionToAngleAxis(q_c,axis_c);
      Eigen::Matrix<double,3,1> p_IcinG = trajectory_points[timestamp.at(j)[0]].block(0, 0, 3, 1);
      Eigen::Matrix<double,4,1> q_ItoG = trajectory_points[timestamp.at(j)[0]].block(3, 0, 4, 1);
      Eigen::Quaterniond q_IctoG;
      q_IctoG.x() = q_ItoG[0];
      q_IctoG.y() = q_ItoG[1];
      q_IctoG.z() = q_ItoG[2];
      q_IctoG.w() = q_ItoG[3];
      Eigen::Matrix<double, 3, 3> R_IctoG = q_IctoG.toRotationMatrix();
      Eigen::Isometry3d T_ItoG;
      T_ItoG.linear() = R_IctoG;
      T_ItoG.translation() = p_IcinG;
      Eigen::Isometry3d T_DtoG = T_ItoG * T_ItoD.inverse();
      Eigen::Matrix3d R_d = T_DtoG.linear();
      Eigen::Vector3d t_d = T_DtoG.translation();
      double axis[3];
      Eigen::Quaterniond q_DtoG(R_d);
      double quaternion2[4] = {q_DtoG.w(),q_DtoG.x(),q_DtoG.y(),q_DtoG.z()};
      ceres::QuaternionToAngleAxis(quaternion2,axis);
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n",
              axis_c[0],axis_c[1],axis_c[2], P_CinG[0], P_CinG[1], P_CinG[2], 191.67230216563604,232.9522079795456, 327.8509945573558,0.311107805332893,
              -0.0555464401505884,-0.05907950993375161,0.025473056841047295);
    }
    ///add noise to led position groundtruth
    std::normal_distribution<double> w(0, 1);
    double noise = 0.01;
    for (int i = 0; i < landmark_gt.size(); ++i) {
      landmark_gt.at(i)[0] += noise * w(gen_state_perturb);
      landmark_gt.at(i)[1] += noise * w(gen_state_perturb);
      landmark_gt.at(i)[2] += noise * w(gen_state_perturb);
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n", landmark_gt.at(i)[0], landmark_gt.at(i)[1], landmark_gt.at(i)[2]);
    }
    fclose(fptr);
  }
  void simulator::make_observation(std::vector<Eigen::VectorXd> landmark, std::vector<Eigen::VectorXd> cam_pose) {
    Vector4d_map trajectory_points;
    for (int j = 0; j < cam_pose.size(); ++j) {
      trajectory_points.insert({cam_pose.at(j)[0],cam_pose.at(j).block(1,0,7,1)});
    }
    FILE *fptr = fopen("sim_data.txt", "w");
    int temp;
    int test;
    int counter = 0;
    int cam_index = cam_pose.size();
    int point_index = landmark.size();
    Eigen::Matrix<double,8,1> cam_d;
    cam_d << 240.79420471191406, 240.79420471191406, 325.0962829589844, 237.8402557373047, 0.20703743398189545,
        -0.15954411029815674, 0.060133516788482666, -0.010181999765336514;
    fprintf(fptr, "%d %d\n", cam_index, point_index);
    for (int j = 0; j < cam_index; ++j) {
      for (int k = 0; k < point_index; ++k) {
        Eigen::Matrix<double, 3, 3> R_GtoC = ov_core::quat_2_Rot(cam_pose.at(j).block(4, 0, 4, 1)).transpose();
        Eigen::Matrix<double,3,1> p_CinG = cam_pose.at(j).block(1,0,3,1);
        Eigen::Vector3d p_FinG = landmark.at(k);
        Eigen::Vector3d p_FinC = R_GtoC*(p_FinG - p_CinG);
        Eigen::Vector2f uv_norm;
        uv_norm << p_FinC(0)/p_FinC(2),p_FinC(1)/p_FinC(2);

        // Distort the normalized coordinates (false=radtan, true=fisheye)
        Eigen::Vector2f uv_dist;

        double r = sqrt(uv_norm(0)*uv_norm(0)+uv_norm(1)*uv_norm(1));
        double theta = std::atan(r);
        double theta_d = theta+cam_d(4)*std::pow(theta,3)+cam_d(5)*std::pow(theta,5)+cam_d(6)*std::pow(theta,7)+cam_d(7)*std::pow(theta,9);

        // Handle when r is small (meaning our xy is near the camera center)
        double inv_r = r > 1e-8 ? 1.0/r : 1;
        double cdist = r > 1e-8 ? theta_d * inv_r : 1;

        // Calculate distorted coordinates for fisheye
        double x1 = uv_norm(0)*cdist;
        double y1 = uv_norm(1)*cdist;
        uv_dist(0) = cam_d(0)*x1 + cam_d(2);
        uv_dist(1) = cam_d(1)*y1 + cam_d(3);
        if (abs(uv_dist(0))<=640 && abs(uv_dist(1))<=480) {
//          printf("%d %d %f %f \n", j, k, uv_dist(0), uv_dist(1));
          fprintf(fptr, "%d %d %f %f\n", j, k, uv_dist(0), uv_dist(1));
//          uvs.emplace_back(uv_dist);
          temp = j;
        }
      }
      if (test != temp || test == 0) {
        test = temp;
        std::cout << counter << std::endl;
        counter++;
        timestamp.emplace_back(cam_pose.at(test).block(0, 0, 1, 1));
      }
    }
    std::cout << timestamp.size() << std::endl;
    for (int j = 0; j < timestamp.size(); ++j) {
//      std::cout<< RED << "debug" << std::endl;
      Eigen::Matrix<double,3,1> p_CinG = trajectory_points[timestamp.at(j)[0]].block(0, 0, 3, 1);
      Eigen::Matrix<double,4,1> q_CtoG = trajectory_points[timestamp.at(j)[0]].block(3, 0, 4, 1);
      double q[4] = {q_CtoG[3],q_CtoG[0],q_CtoG[1],q_CtoG[2]};
      double axis_angle[3];
      ceres::QuaternionToAngleAxis(q,axis_angle);
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n", axis_angle[0],axis_angle[1],axis_angle[2],
              p_CinG[0], p_CinG[1], p_CinG[2], 240.79420471191406, 325.0962829589844, 237.8402557373047,0.20703743398189545,
              -0.15954411029815674, 0.060133516788482666, -0.010181999765336514);
//      std::cout<< RED << "debug2" << std::endl;
    }
    for (int i = 0; i < landmark.size(); ++i) {
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n", landmark.at(i)[0], landmark.at(i)[1], landmark.at(i)[2]);
    }
    fclose(fptr);
//    is_running = false;
  }
  void simulator::rifts_transToCamToW(std::vector<Eigen::VectorXd> traj){
    FILE *fptr = fopen("camPose.txt", "w");
    Eigen::Matrix3d R_ItoD = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> p_IinD;
    p_IinD <<  0.005585638, -0.007119839, 0.019845198;
//    p_IinD << -0.000295, -0.0190975, -0.068692;
    Eigen::Isometry3d T_ItoD;
    T_ItoD.linear() = R_ItoD;
    T_ItoD.translation() = p_IinD;
    for (int j = 0; j < traj.size(); ++j) {
      Eigen::Matrix<double, 4, 1> q = traj.at(j).block(4, 0, 4, 1);
      Eigen::Quaterniond q_CtoG;
      q_CtoG.x() = q[0];
      q_CtoG.y() = q[1];
      q_CtoG.z() = q[2];
      q_CtoG.w() = q[3];
      Eigen::Matrix<double, 3, 3> R_CtoG = ov_core::quat_2_Rot(q);
//      std::cout << R_CtoG.transpose()*R_CtoG << std::endl;
      Eigen::Matrix<double,3, 1> p_CinG = traj.at(j).block(1, 0, 3, 1);
//      printf("%.16g %.16g %.16g %.16g %.16g %.16g %.16g\n", p_CinG[0], p_CinG[1], p_CinG[2],q_CtoG.x(), q_CtoG.y(),q_CtoG.z(),q_CtoG.w());
      Eigen::Isometry3d T_CtoG;
      T_CtoG.linear() = R_CtoG;
      T_CtoG.translation() = p_CinG;
      Eigen::Isometry3d T_GtoC = T_CtoG.inverse();
      Eigen::Matrix3d R = T_GtoC.linear();
      Eigen::Vector3d t = T_GtoC.translation();
      Eigen::Quaterniond q_GtoC(R);
      double quaternion[4] = {q_GtoC.w(),q_GtoC.x(), q_GtoC.y(),q_GtoC.z()};
      double axis_angle[3];
      ceres::QuaternionToAngleAxis(quaternion,axis_angle);
      Eigen::Matrix<double, 4,1> q_ItoG = traj.at(j).block(10,0,4,1);
      Eigen::Quaterniond q_IctoG;
      q_IctoG.x() = q_ItoG[0];
      q_IctoG.y() = q_ItoG[1];
      q_IctoG.z() = q_ItoG[2];
      q_IctoG.w() = q_ItoG[3];
      Eigen::Matrix<double, 3, 3> R_IctoG = q_IctoG.toRotationMatrix();
      Eigen::Matrix<double,3,1> p_IinG = traj.at(j).block(7,0,3,1);
      Eigen::Isometry3d T_ItoG;
      T_ItoG.linear() = R_IctoG;
      T_ItoG.translation() = p_IinG;
      Eigen::Isometry3d T_DtoG = T_ItoG * T_ItoD.inverse();
      Eigen::Matrix3d R_d = T_CtoG.linear();
      Eigen::Vector3d t_d = T_CtoG.translation();
      double axis[3];
      Eigen::Quaterniond q_DtoG(R_d);
      double quaternion2[4] = {q_DtoG.w(),q_DtoG.x(),q_DtoG.y(),q_DtoG.z()};
      ceres::QuaternionToAngleAxis(quaternion2,axis);

      ///cam2
//      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n",
//              axis_angle[0],axis_angle[1],axis_angle[2], t[0], t[1], t[2], 191.67230216563604,232.9522079795456, 327.8509945573558,0.311107805332893,
//              -0.0555464401505884,-0.05907950993375161,0.025473056841047295,
//              axis[0],axis[1],axis[2],t_d[0], t_d[1], t_d[2]);
      ///cam3
//      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n",
//              axis_angle[0],axis_angle[1],axis_angle[2], t[0], t[1], t[2], 190.05692288380095,231.9306188458731, 319.36058469052216,0.305583880028298,
//              -0.03157661938019855,
//              -0.08238701261728001,
//              0.03235131726796304,
//              axis[0],axis[1],axis[2],t_d[0], t_d[1], t_d[2]);
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n",
              axis[0],axis[1],axis[2],t_d[0], t_d[1], t_d[2]);
    }
    ///add noise to led position groundtruth
//    std::normal_distribution<double> w(0, 1);
//    double noise = 0.01;
//    for (int i = 0; i < landmark.size(); ++i) {
//      landmark.at(i)[0] += noise * w(gen_state_perturb);
//      landmark.at(i)[1] += noise * w(gen_state_perturb);
//      landmark.at(i)[2] += noise * w(gen_state_perturb);
//      fprintf(fptr, "%.16g\n%.16g\n%.16g\n", landmark.at(i)[0], landmark.at(i)[1], landmark.at(i)[2]);
//    }
    fclose(fptr);

  }
  void simulator::transToCamToW(std::vector<Eigen::VectorXd> traj){
    Vector4d_map trajectory_points;
    FILE *fptr = fopen("camPose.txt", "w");
    Eigen::Matrix<double, 3, 3> R_C0toI, R_C1toI, R_C2toI;
    R_C0toI << 0.40098079603352366, -0.903283976143996, -0.15261853309465917,
        -0.750008152966651, -0.22803638484446043, -0.6208760661332144,
        0.5260248365676523,0.3634245141084953,-0.7689085555841781;

    R_C1toI << -0.388767901563122,-0.9097232202887576,0.14581916385349014,
        -0.7436080526358758, 0.2163738669319456, -0.6326368946641698,
        0.5439730049685911, -0.3543812056284815, -0.7605967334991641;

    R_C2toI << 0.9999577114515948,0.006936389653247441,0.006038361289886366,
                0.008627237983452943,-0.9349452900574964,-0.35468701042480094,
                0.003185290138392905,0.3547241056058698,-0.9349655944626263;
    Eigen::Matrix<double, 3,1> p_C0inI, p_C1inI, p_C2inI;
    p_C0inI << -0.04637468085763562, -0.044869103651471925, 0.005013785249861916;
    p_C1inI << 0.04338028385469968, -0.04432796250252792, 0.0046953045314795;
    p_C2inI << 0.055056824653019504,-0.009432827022165375,-0.0014056119466753086;
//    std::cout << traj.at(0).block(1,0,3,1);
    for (int j = 0; j < traj.size(); ++j) {
      trajectory_points.insert({traj.at(j)[0],traj.at(j).block(1,0,7,1)});
    }
//    std::cout << std::fixed << timestamp.at(0)[0] << std::endl << std::fixed
//    <<trajectory_points[timestamp.at(0)[0]].block(0, 0, 3, 1) << std::endl;
//    std::cout << std::fixed <<traj.at(0).block(1,0,7,1) << std::endl;
//    Eigen::Matrix3d I_3 = Eigen::Matrix3d::Identity();
//    Eigen::Quaterniond p(I_3);
//    printf("%.16g %.16g %.16g %.16g\n",p.w(),p.x(), p.y(),p.z());
    for (int k = 0; k < timestamp.size(); ++k) {
//      std::cout << std::fixed  << timestamp.at(k)[0] << std::endl;
      Eigen::Matrix<double, 4, 1> q = trajectory_points[timestamp.at(k)[0]].block(3, 0, 4, 1);
      Eigen::Quaterniond q_ItoG;
      q_ItoG.x() = q[0];
      q_ItoG.y() = q[1];
      q_ItoG.z() = q[2];
      q_ItoG.w() = q[3];
      Eigen::Matrix<double, 3, 3> R_ItoG = q_ItoG.toRotationMatrix();
      Eigen::Matrix<double,3, 1> p_IinG = trajectory_points[timestamp.at(k)[0]].block(0, 0, 3, 1);
      Eigen::Matrix<double,3,1> p_CinG = p_IinG + R_ItoG*p_C1inI;
      Eigen::Matrix<double, 3, 3> R_CtoG = R_ItoG*R_C1toI;
      Eigen::Isometry3d T_CtoG;
      T_CtoG.linear() = R_CtoG;
      T_CtoG.translation() = p_CinG;
      Eigen::Isometry3d T_GtoC = T_CtoG.inverse();
      Eigen::Matrix3d R = T_GtoC.linear();
      Eigen::Vector3d t = T_GtoC.translation();
      Eigen::Quaterniond q_GtoC(R);
      double quaternion[4] = {q_GtoC.w(),q_GtoC.x(), q_GtoC.y(),q_GtoC.z()};
      double axis_angle[3];
      ceres::QuaternionToAngleAxis(quaternion,axis_angle);
//      printf("%.16g %.16g %.16g %.16g %.16g %.16g %.16g\n", p_IinG[0], p_IinG[1], p_IinG[2],q_ItoG.x(), q_ItoG.y(),q_ItoG.z(),q_ItoG.w());
//      pose << timestamp.at(k)[0],p_CinG, ov_core::rot_2_quat(R_CtoG);
        ///CAM1
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n", axis_angle[0],axis_angle[1],axis_angle[2],
              t[0], t[1], t[2], 240.79420471191406, 325.0962829589844, 237.8402557373047,0.20703743398189545,
              -0.15954411029815674, 0.060133516788482666, -0.010181999765336514);
        ///CAM0
//      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n", axis_angle[0],axis_angle[1],axis_angle[2],
//              t[0], t[1], t[2], 237.1248016357422, 319.2764587402344, 237.0658721923828,0.20682667195796967,
//              -0.15467560291290283, 0.05298982560634613, -0.007736388128250837);
      ///cam2
//      fprintf(fptr, "%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n%.16g\n", axis_angle[0],axis_angle[1],axis_angle[2],
//              t[0], t[1], t[2], 191.67230216563604,232.9522079795456, 327.8509945573558,0.311107805332893,
//              -0.0555464401505884,-0.05907950993375161,0.025473056841047295);
//      fprintf(fptr, "%.16g %.16g %.16g %.16g %.16g %.16g %.16g %.16g\n",timestamp.at(k)[0],
//              p_CinG[0], p_CinG[1], p_CinG[2],q_CtoG.x(), q_CtoG.y(),q_CtoG.z(),q_CtoG.w());
//      pose_camToW.emplace_back(pose);
    }
    for (int i = 0; i < landmark.size(); ++i) {
      fprintf(fptr, "%.16g\n%.16g\n%.16g\n", landmark.at(i)[0], landmark.at(i)[1], landmark.at(i)[2]);
    }
    fclose(fptr);
  }
  void simulator::transLandmarkFrame(std::vector<Eigen::VectorXd> landmark_gt){
    Eigen::Matrix3d R_ItoD = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 3, 1> p_IinD;
    p_IinD <<  0.005585638, -0.007119839, 0.019845198;
    Eigen::Isometry3d T_ItoD;
    T_ItoD.linear() = R_ItoD;
    T_ItoD.translation() = p_IinD;
    Eigen::Isometry3d T_DtoI = T_ItoD.inverse();
//    std::cout << T_DtoI.translation()  << std::endl;
    Eigen::Quaterniond q_IctoG;
    q_IctoG.x() = 0.430324377699253;
    q_IctoG.y() = -0.013279077126003068;
    q_IctoG.z() = -0.009885902115560438;
    q_IctoG.w() = 0.9025225011088609;
    Eigen::Matrix3d R_IctoG = q_IctoG.toRotationMatrix();
    Eigen::Matrix<double, 3,1> p_IinG;
    p_IinG << -0.7791137311291185,0.37286963310742466, -0.24995748986339272;
    Eigen::Isometry3d T_ItoG;
    T_ItoG.linear() = R_IctoG;
    T_ItoG.translation() = p_IinG;
    Eigen::Isometry3d T_DtoG = T_ItoG * T_ItoD.inverse();
    Eigen::Matrix3d R = T_DtoG.linear();
    Eigen::Vector3d t = T_DtoG.translation();
//    Eigen::Matrix<double, 3, 3> R_DtoG = R_IctoG*R_ItoD.transpose();
    std::normal_distribution<double> w(0, 1);
    double noise = 0.01;
    for (int j = 0; j < landmark_gt.size(); ++j) {
      Eigen::Matrix<double,3,1> p_LinD = landmark_gt.at(j).block(0,0,3,1);
      p_LinD[0] += noise * w(gen_state_perturb);
      p_LinD[1] += noise * w(gen_state_perturb);
      p_LinD[2] += noise * w(gen_state_perturb);

//      std::cout << p_LinD << std::endl;
//      Eigen::Matrix<double,3, 1> p_LinG = p_IinG+R_IctoG*p_IinD+R_DtoG*p_LinD;
      Eigen::Matrix<double,3, 1> p_LinG = R*p_LinD+t;
      landmark.emplace_back(p_LinG);
//      printf("%.16g\n%.16g\n%.16g\n", landmark.at(j)[0], landmark.at(j)[1], landmark.at(j)[2]);
    }
  }
  void simulator::transToG2Odataset(std::vector<Eigen::VectorXd> traj) {
    FILE *fptr = fopen("dataset.g2o", "w");
    for (int j = 0; j < traj.size(); ++j) {
      Eigen::Matrix<double, 4, 1> q = traj.at(j).block(3, 0, 4, 1);
      Eigen::Matrix<double,3,1> t = traj.at(j).block(0,0,3,1);
      fprintf(fptr, "VERTEX_SE3:QUAT %d %.16g %.16g %.16g %.16g %.16g %.16g %.16g\n", j, t[0],t[1],t[2],q[0],q[1],q[2],q[3]);
    }
    for (int i = 0; i < traj.size(); ++i) {
      ///first SE3
      Eigen::Matrix<double, 4, 1> q_i = traj.at(i).block(3, 0, 4, 1);
      Eigen::Quaterniond q_1;
      q_1.x() = q_i[0];
      q_1.y() = q_i[1];
      q_1.z() = q_i[2];
      q_1.w() = q_i[3];
      Eigen::Matrix<double,3,1> t_i = traj.at(i).block(0,0,3,1);
      Eigen::Matrix3d R_i =  q_1.toRotationMatrix();
      Eigen::Isometry3d T_i;
      T_i.linear() = R_i;
      T_i.translation() = t_i;
      ///second SE3
      Eigen::Matrix<double, 4, 1> q_j = traj.at(i+1).block(3, 0, 4, 1);
      Eigen::Quaterniond q_2;
      q_2.x() = q_j[0];
      q_2.y() = q_j[1];
      q_2.z() = q_j[2];
      q_2.w() = q_j[3];
      Eigen::Matrix<double,3,1> t_j = traj.at(i+1).block(0,0,3,1);
      Eigen::Matrix3d R_j =  q_2.toRotationMatrix();
      Eigen::Isometry3d T_j;
      T_j.linear() = R_j;
      T_j.translation() = t_j;
      ///transpose between first and second
      Eigen::Isometry3d T_ij = T_i.inverse()*T_j;
      Eigen::Quaterniond q_ij(T_ij.linear());
      Eigen::Vector3d t_ij = T_ij.translation();
      fprintf(fptr, "EDGE_SE3:QUAT %d %d %.16g %.16g %.16g %.16g %.16g %.16g %.16g 100.000000 0.000000 0.000000 0.000000 0.000000 0.000000   100.000000 0.000000 0.000000 0.000000 0.000000   100.000000 0.000000 0.000000 0.000000   400.000000 0.000000 0.000000   400.000000 0.000000   400.000000\n", i, i+1, t_ij[0],t_ij[1],t_ij[2],q_ij.x(),q_ij.y(),q_ij.z(),q_ij.w());
      if (i+1 >= traj.size())
        continue;
    }
  }
  void simulator::load_data(std::string path_traj, std::vector<Eigen::VectorXd> &traj) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
      printf(RED "ERROR: Unable to open simulation trajectory file...\n" RESET);
      printf(RED "ERROR: %s\n" RESET, path_traj.c_str());
      exit(EXIT_FAILURE);
    }

    // Debug print
    std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
    printf("[SIM]: loaded trajectory %s\n", base_filename.c_str());

    // Loop through each line of this file
    std::string current_line;
    while (getline(file, current_line)) {

      // Skip if we start with a comment
      if (!current_line.find("#"))
        continue;

      // Loop variables
      int i = 0;
      std::istringstream s(current_line);

      std::string field;
      Eigen::Matrix<double, 15, 1> data;

      // Loop through this line (timestamp(s) tx ty tz qx qy qz qw)
      while (getline(s, field, ' ')) {
        // Skip if empty
//        std::cout << field.c_str() << std::endl;
        if (field.empty() || i >= data.rows())
          continue;
        // save the data to our vector
        data(i) = atof(field.c_str());

        i++;
      }

      // Only a valid line if we have all the parameters
      if (i > 0) {
        traj.emplace_back(data);
        //             cout << setprecision(15) << data.transpose() << endl;
      }

    }

    // Finally close the file
    file.close();

    // Error if we don't have any data
    if (traj.empty()) {
      printf(RED "ERROR: Could not parse any data from the file!!\n" RESET);
      printf(RED "ERROR: %s\n" RESET, path_traj.c_str());
      exit(EXIT_FAILURE);
    }
  }
  void simulator::imu_evaluate(std::vector<Eigen::VectorXd> imu_traj){
    long double store = 0;
    double counter = 0;
    long double acc_norm = 0;
    Eigen::Matrix<double, 3, 1> angle_average;
    ///IMU frequency
    for (int j = 0; j < imu_traj.size()-1; ++j) {
      double *timestamp_k = imu_traj.at(j).data();
      double *timestamp_k1 = imu_traj.at(j+1).data();
//      store = store + timestamp_k1[0] - timestamp_k[0];

      if((timestamp_k1[0] - timestamp_k[0]) < 3971000 || (timestamp_k1[0] - timestamp_k[0]) > 3974000) {
//        std::cout << std::fixed << timestamp_k1[0] - timestamp_k[0] << std::endl;
        counter++;
      }
    }
    for (int i = 0; i < imu_traj.size(); ++i) {
      Eigen::Matrix<double, 3, 1> angle_vel = imu_traj.at(i).block(1, 0, 3, 1);
      Eigen::Matrix<double, 3, 1> acc = imu_traj.at(i).block(4,0,3,1);
//      std::cout << imu_traj.at(i).block(4, 0, 3, 1).norm() << std::endl;
      acc_norm += acc.norm();
      angle_average += angle_vel;
    }
    std::cout << RED << "acc norm average: " << acc_norm / imu_traj.size() << std::endl;
    std::cout << RED << "angle vel average: " << std::endl << angle_average / imu_traj.size() << std::endl;

//    std::cout << std::fixed << store / imu_traj.size() << std::endl;
//      std::cout << RED << std::fixed<< counter/imu_traj.size() << std::endl;
  }
} // calib