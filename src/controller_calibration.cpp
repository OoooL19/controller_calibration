#include <iostream>
#include "imu_calibration_evaluation/calibration/imu_calibration.h"
#include "imu_calibration_evaluation/evaluation/imu_evaluation.h"

int main(int argc, char **argv) {
    if (argc != 5) {
        std::cerr << std::endl << "Usage: ./cotroller_calibration  SN_number  left(right) data_path config_path" << std::endl;
        return 1;
    }

    std::string SN_number = argv[1];
    std::string controller_id = argv[2];
    std::string data_path = argv[3];
    std::string config_path = argv[4];

    std::cout << "SN_number: " << SN_number << std::endl;
    std::cout << "controller_id: " << controller_id << std::endl;
    std::cout << "data_path: " << data_path << std::endl;
    std::cout << "config_path: " << config_path << std::endl;

    calib::imu_calibration imuCalibration;
    std::string imu_file = (controller_id == "left") ? "imu_1.txt" : "imu_2.txt";
    std::string imu_error_json = "imu_evaluation.json";
    std::string imu_full_path;
    std::string imu_error_handling;

    // create folder
#ifdef WIN32
  std::string folderPath = data_path + "\\factory" + "\\" + controller_id;

  std::string command;
  command = "mkdir -p " + folderPath;
  system(command.c_str());
#else
  std::string folderPath = data_path + "/factory" + "/" + controller_id;

  std::string command;
  command = "mkdir -p " + folderPath;
  system(command.c_str());
#endif

#ifdef WIN32
    imu_full_path = data_path + "\\" + imu_file;
    imu_error_handling = config_path + "\\" + imu_error_json;
#else
    imu_full_path = data_path + "/" + imu_file;
    imu_error_handling = config_path + "/" + imu_error_json;
#endif

    // generate result path
    std::string save_path;
#ifdef WIN32
    save_path = data_path + "\\factory" + "\\" + controller_id;
#else
    save_path = data_path + "/factory" + "/" + controller_id;
#endif


    // IMU evaluation
//    calib::imu_evaluation imuEvaluation;
//    imuEvaluation.load_data(imu_full_path);
//    imuEvaluation.imu_evaluate(imu_error_handling,save_path);
//    std::cout << "end imu evaluation" << std::endl;

    // IMU calibration
    imuCalibration.load_data(imu_full_path);
    imuCalibration.calibrate_imu(SN_number, controller_id, save_path, imu_error_handling);
    std::cout << "end imu calibration" << std::endl;

    return 0;
}