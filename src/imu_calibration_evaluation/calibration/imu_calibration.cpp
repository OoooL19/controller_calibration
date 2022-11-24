//
// Created by owen on 9/14/22.
//

#include <iomanip>
#include <time.h>
#include "imu_calibration.h"
#include "imu_tk/include/imu_tk/calibration.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

namespace calib {
  imu_calibration::imu_calibration() {
    printf("=======================================\n");
    printf("IMU CALIBRATION STARTING\n");
    printf("=======================================\n");
  }

  std::string GetTimeStampString() {
      std::time_t t = std::time(0);
      std::tm *now = std::localtime(&t);
      char str[150];
      snprintf(str, sizeof(str), "%4d-%02d-%02d %02d:%02d:%02d", now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
               now->tm_hour, now->tm_min, now->tm_sec);
      return std::string(str);
  }

 void imu_calibration::calibrate_imu(std::string SN_number, std::string controller_id, std::string &save_path, const std::string& imu_error_handling) {
   std::ifstream ifs(imu_error_handling);
   json jf = json::parse(ifs);
   double m_t_init = jf["m_t_init"];
    std::cout << m_imuDatas.size() << std::endl;
    std::vector< imu_tk::TriadData > acc_data, gyro_data;
    for (auto& data : m_imuDatas) {
      acc_data.emplace_back(data.ts, data.acc);
      gyro_data.emplace_back(data.ts, data.gyro);
    }

    imu_tk::CalibratedTriad init_acc_calib, init_gyro_calib;
    init_acc_calib.setBias(Eigen::Vector3d(0, 0, 0));
    init_gyro_calib.setScale(Eigen::Vector3d(1.0, 1.0, 1.0));

    imu_tk::MultiPosCalibration mp_calib;

    mp_calib.setInitStaticIntervalDuration(m_t_init);
    mp_calib.setInitAccCalibration(init_acc_calib);
    mp_calib.setInitGyroCalibration(init_gyro_calib);
    mp_calib.setGravityMagnitude(9.80);
    mp_calib.enableVerboseOutput(false);
    mp_calib.enableAccUseMeans(false);
    //mp_calib.setGyroDataPeriod(0.01);
    mp_calib.calibrateAccGyro(acc_data, gyro_data);

    std::cout << "Acc calib: " << mp_calib.getAccCalib() << std::endl;
    std::cout << "Gyro calib: " << mp_calib.getGyroCalib() << std::endl;
    printf("start save json file\n");
    auto toJson = [](const imu_tk::CalibratedTriad_<double>& calib) {
          json js_calib;
          Eigen::Vector3d bias = calib.getBiasVector();
          Eigen::Matrix3d rectification = calib.getMisalignmentMatrix() * calib.getScaleMatrix();
          js_calib["offset"] = { bias[0], bias[1], bias[2] };
          js_calib["rectification_matrix"] =
                  {
                          rectification(0,0),rectification(0,1),rectification(0,2),
                          rectification(1,0),rectification(1,1),rectification(1,2),
                          rectification(2,0),rectification(2,1),rectification(2,2)
                  };
          return js_calib;
      };

      json root;
      root["version"] = 0;
      root["SerialNumber"] = SN_number;
      root["DeviceType"] = "controller";
      root["Hand"] = controller_id;
   //      root["ImuCalibration"]["acc"] = toJson(mp_calib.getAccCalib());
//      root["ImuCalibration"]["gyro"] = toJson(mp_calib.getGyroCalib());
   Eigen::Matrix3d acc_mis_Mat = mp_calib.getAccCalib().getMisalignmentMatrix();
   Eigen::Matrix3d acc_sc_Mat = mp_calib.getAccCalib().getScaleMatrix();
   Eigen::Matrix3d gyro_sc_Mat = mp_calib.getGyroCalib().getScaleMatrix();
   Eigen::Matrix3d gyro_mis_Mat = mp_calib.getGyroCalib().getMisalignmentMatrix();

   json acc;
   json gyro;
   acc["Misalignment Matrix"]=
       {
           acc_mis_Mat(0,0),acc_mis_Mat(0,1),acc_mis_Mat(0,2),
           acc_mis_Mat(1,0),acc_mis_Mat(1,1),acc_mis_Mat(1,2),
           acc_mis_Mat(2,0),acc_mis_Mat(2,1),acc_mis_Mat(2,2)
       };
   acc["Scale Matrix"]=
       {
           acc_sc_Mat(0,0),
           acc_sc_Mat(1,1),
           acc_sc_Mat(2,2)
       };
   gyro["Misalignment Matrix"]=
       {
           gyro_mis_Mat(0,0),gyro_mis_Mat(0,1),gyro_mis_Mat(0,2),
           gyro_mis_Mat(1,0),gyro_mis_Mat(1,1),gyro_mis_Mat(1,2),
           gyro_mis_Mat(2,0),gyro_mis_Mat(2,1),gyro_mis_Mat(2,2)
       };
   gyro["Scale Matrix"]=
       {
           gyro_sc_Mat(0,0),
           gyro_sc_Mat(1,1),
           gyro_sc_Mat(2,2)
       };
   Eigen::Vector3d acc_bias = mp_calib.getAccCalib().getBiasVector();
   Eigen::Vector3d gyro_bias = mp_calib.getGyroCalib().getBiasVector();
   acc["Bias Vector"]={acc_bias[0], acc_bias[1], acc_bias[2]};
   gyro["Bias Vector"]={gyro_bias[0], gyro_bias[1], gyro_bias[2]};

   root["Acc calib"] = acc;
   root["Gyro calib"] = gyro;
      {
#ifdef WIN32
          struct tm time_s;
          auto now = time(nullptr);
          localtime_s(&time_s, &now);
          char buf[100] = { 0 };
          strftime(buf, sizeof(buf), "%y-%m-%d %H:%M:%S", &time_s);
          root["Timestamp"] = buf;
#else
          root["Timestamp"] = GetTimeStampString();
#endif

      }
      std::string fileName = "imu_calibration_params.json";
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


  void imu_calibration::load_data(std::string path_traj) {

    // Try to open our groundtruth file
    std::ifstream file;
    file.open(path_traj);
    if (!file) {
      printf("ERROR: Unable to open simulation trajectory file...\n");
      printf("ERROR: %s\n", path_traj.c_str());
      exit(EXIT_FAILURE);
    }

    printf("[SIM]: loaded trajectory %s\n", path_traj.c_str());
    // Loop through each line of this file

    std::ifstream imu_data_ifs;
    imu_data_ifs.open(path_traj.c_str());
    if (imu_data_ifs) {
        ImuData imu_data;
        uint64_t timestamp;
        double last_time_stamp = -1.;
        double max_gap =6.;
        while (imu_data_ifs >> timestamp >> imu_data.gyro.x() >> imu_data.gyro.y() >> imu_data.gyro.z() >>
        imu_data.acc.x() >> imu_data.acc.y() >> imu_data.acc.z()) {
            if (int64_t(timestamp) < 0) {
                continue;
            }
            imu_data.ts = static_cast<double>(timestamp * 1e-9);

            if(last_time_stamp < 0.){
              // first time stamp
              last_time_stamp = imu_data.ts;
            }
            // for time stamp miss value
            if(imu_data.ts < last_time_stamp || abs(imu_data.ts - last_time_stamp) > max_gap){
              std::cout<<"Has time stamp roll back or large time gap!! time stamp now: "<<imu_data.ts;
              std::cout<<" , timestamp last:"<< last_time_stamp<<", time gap: "<<abs(imu_data.ts - last_time_stamp) <<std::endl;
              // continue ;
              //throw std::runtime_error(" 时间戳错误, 请重新录制数据!! time stamp now: "+ std::to_string(imu_data.ts) +
              //                         " , timestamp last: " + std::to_string(last_time_stamp) +  ", time gap: " + std::to_string(imu_data.ts - last_time_stamp));
              throw std::runtime_error(" Has time stamp roll back or large time gap!! time stamp now: "+ std::to_string(imu_data.ts) +
                                       " , timestamp last: " + std::to_string(last_time_stamp) +  ", time gap: " + std::to_string(imu_data.ts - last_time_stamp) + " please repeat datalogger");
            }
            last_time_stamp = imu_data.ts;
            m_imuDatas.push_back(imu_data);
        }
    }

    imu_data_ifs.close();
    // Error if we don't have any data
    if (m_imuDatas.empty()) {
      printf("ERROR: Could not parse any data from the file!!\n" );
      printf("ERROR: %s\n", path_traj.c_str());
      exit(EXIT_FAILURE);
    }
  }
} // calib