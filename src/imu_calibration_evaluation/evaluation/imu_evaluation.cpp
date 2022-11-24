//
// Created by owen on 9/13/22.
//
#include <iomanip>
#include <time.h>
#include "imu_evaluation.h"
using json = nlohmann::json;

namespace calib {
  imu_evaluation::imu_evaluation() {}

  std::string GetTimeStamp() {
    std::time_t t = std::time(0);
    std::tm *now = std::localtime(&t);
    char str[150];
    snprintf(str, sizeof(str), "%4d-%02d-%02d %02d:%02d:%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
             now->tm_hour, now->tm_min, now->tm_sec);
    return std::string(str);
  }

  void imu_evaluation::imu_evaluate(const std::string& imu_error_handling,std::string &save_path) {
    // Debug print
    std::string base_filename = imu_error_handling.substr(imu_error_handling.find_last_of("/\\") + 1);
    printf("[SIM]: loaded trajectory %s\n", base_filename.c_str());
    
    std::ifstream ifs(imu_error_handling);
    json jf = json::parse(ifs);
    double imu_frequency = jf["imu_frequency"];
    double imu_frequency_threshold =  1/ imu_frequency * 1.5;
    double gravity_local = jf["gravity_local"];
    double nsToms = pow(10, -6);
    long double store = jf["store"];
    double drop_frame_counter = jf["drop_frame_counter"];
    double drop_frame_continuation = 0;
    double counter = 0;
    double roll_back_count = jf["roll_back_count"];
    long double acc_norm = jf["acc_norm"];
    double acc_bias = jf["acc_bias"];
    double gyro_bias = jf["gyro_bias"];
    std::vector< int > line_num;
    std::vector< int > nan_line_num;
    Eigen::Matrix<double, 3, 1> angle_all;
    angle_all.setZero();
    for (int j = 0; j < imu_traj_data.size()-1; ++j) {
      double *timestamp_k = imu_traj_data.at(j).data();
      double *timestamp_k1 = imu_traj_data.at(j + 1).data();
      ///IMU drop frame continuously counter
      if ((timestamp_k1[0] - timestamp_k[0]) > imu_frequency_threshold) {
          drop_frame_counter++;
          counter++;
      }
      else {
        if (drop_frame_counter!=0){
          if (drop_frame_counter > 1)
            drop_frame_continuation++;
          drop_frame_counter = 0;
        }
      }
      store = store + timestamp_k1[0] - timestamp_k[0];
    }

    ///IMU frequency
    std::cout  << "IMU frequency is: "  << std::fixed <<(imu_traj_data.size()/ store )  << " HZ" << std::endl;
    ///Check drop frame
    std::cout << "IMU continuously drop frame count: " << std::fixed << drop_frame_continuation << std::endl;
    std::cout << "IMU drop frame ratio: " << (counter / imu_traj_data.size()) * 100 << "%" << std::endl;

    ///check timeframe roll back
    for (int i = 0; i < imu_traj_data.size() - 1; ++i) {
      for (int j = i+1; j < imu_traj_data.size(); ++j) {
        double *timestamp_k = imu_traj_data.at(i).data();
        double *timestamp_k1 = imu_traj_data.at(j).data();
        if (timestamp_k[0] == timestamp_k1[0]){
          roll_back_count++;
          line_num.push_back(j);
        }
        if (imu_traj_data.at(i).block(1,0,6,1) == imu_traj_data.at(j).block(1,0,6,1)){
          nan_line_num.push_back(j);
        }
      }
    }
    if (roll_back_count != 0) {
      std::cout  << "IMU timeframe roll back ratio: "  << roll_back_count/imu_traj_data.size()*100<< "%" << std::endl;
      for (int i = 0; i < line_num.size(); ++i) {
        std::cout  << "IMU timeframe roll back at line "  << line_num[i] << std::endl;
      }
    }
    else{
      std::cout  << "IMU timeframe roll back: "  << 0 << std::endl;
    }
    /// data is nan or inaccurate
    for (int i = 0; i < nan_line_num.size(); ++i) {
//      std::cout << YELLOW << "IMU datas are nan or inaccurate at line " << RED << nan_line_num[i] << std::endl;
    }
    ///Testing acc and gyro
    for (int i = 0; i < imu_traj_data.size(); ++i) {
      Eigen::Matrix<double, 3, 1> angle_vel = imu_traj_data.at(i).block(1, 0, 3, 1);
      Eigen::Matrix<double, 3, 1> acc = imu_traj_data.at(i).block(4, 0, 3, 1);
//      std::cout << imu_traj.at(i).block(4, 0, 3, 1).norm() << std::endl;
      acc_norm += acc.norm();
      angle_all += angle_vel;
    }
    double acc_norm_average = acc_norm / imu_traj_data.size();
    long double acc_temp = 0;
    std::cout << "acc norm average: " << acc_norm / imu_traj_data.size() << std::endl;
    Eigen::Matrix<double, 3, 1> angle_average = angle_all / imu_traj_data.size();
    Eigen::Matrix<double, 3, 1> angle_temp;
    angle_temp.setZero();
    for (int i = 0; i < imu_traj_data.size(); ++i) {
      angle_temp += (imu_traj_data.at(i).block(1, 0, 3, 1) - angle_average).cwiseProduct (imu_traj_data.at(i).block(1, 0, 3, 1) - angle_average);
      acc_temp += (imu_traj_data.at(i).block(4, 0, 3, 1).norm() - acc_norm_average)*(imu_traj_data.at(i).block(4, 0, 3, 1).norm() - acc_norm_average);
    }
    Eigen::Matrix<double, 3, 1> angle_variance = angle_temp / imu_traj_data.size();
    double acc_variance = acc_temp / imu_traj_data.size();
    std::cout << "angle vel variance: " << std::endl << angle_variance << std::endl;
    std::cout << "acc vel variance: " << std::endl << acc_variance << std::endl;
    double acc_static_norm = acc_norm_average-acc_variance;
    std::cout << "acc vel static: " << std::endl << acc_static_norm << std::endl;

    json root;
    root["version"] = 0;
    root["DeviceType"] = "controller";
    root["IMU frequency"] = imu_traj_data.size()/ store;
    root["IMU continuously drop frame count"] = drop_frame_continuation;
    root["IMU drop frame ratio"] = (counter / imu_traj_data.size()) * 100;
    root["IMU timeframe roll back ratio"] = roll_back_count/imu_traj_data.size()*100;
    {
#ifdef WIN32
      struct tm time_s;
          auto now = time(nullptr);
          localtime_s(&time_s, &now);
          char buf[100] = { 0 };
          strftime(buf, sizeof(buf), "%y-%m-%d %H:%M:%S", &time_s);
          root["Timestamp"] = buf;
#else
      root["Timestamp"] = GetTimeStamp();
#endif

    }
    std::string fileName = "imu_evaluation_params.json";
    std::string file_save_path;
#ifdef WIN32
    file_save_path = save_path + "\\" + fileName;
#else
    file_save_path = save_path + "/" + fileName;
#endif


    std::ofstream file(file_save_path.c_str());
    file << std::setw(4) << root << std::endl;
    file.flush();
    file.close();
    printf("write to file:%s.\n", file_save_path.c_str());

  }

  void imu_evaluation::load_data(std::string path_traj) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
      printf("ERROR: Unable to open simulation trajectory file...\n" );
      printf("ERROR: %s\n", path_traj.c_str());
      exit(EXIT_FAILURE);
    }

    // Debug print
    std::string base_filename = path_traj.substr(path_traj.find_last_of("/\\") + 1);
    printf("[SIM]: loaded trajectory %s\n", base_filename.c_str());

    std::ifstream imu_data_ifs;
    imu_data_ifs.open(path_traj.c_str());
    if (imu_data_ifs) {
        Eigen::Matrix<double, 7, 1> data;
        uint64_t timestamp;
        while (imu_data_ifs >> timestamp >> data[1] >> data[2] >> data[3] >>
                            data[4] >> data[5] >> data[6]) {
            if (int64_t(timestamp) < 0) {
                continue;
            }
            // s
            data[0] = static_cast<double>(timestamp * 1e-9);
            imu_traj_data.push_back(data);
        }
    }

    imu_data_ifs.close();

    // Error if we don't have any data
    if (imu_traj_data.empty()) {
      printf("ERROR: Could not parse any data from the file!!\n");
      printf("ERROR: %s\n", path_traj.c_str());
      exit(EXIT_FAILURE);
    }
  }
}