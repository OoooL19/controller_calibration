cmake_minimum_required(VERSION 3.10)
project(controller_calibration)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmakeModules/)

# Find catkin (the ROS build system)
find_package(catkin REQUIRED COMPONENTS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge pcl_ros ov_core ov_eval ov_msckf velodyne_msgs)

# Include libraries
find_package(GTSAM REQUIRED)
#find_package(PCL 1.8 REQUIRED)
find_Package(Ceres REQUIRED)
find_package(OpenMP 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV QUIET)
find_package(X11 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)
find_package(MKL REQUIRED) # intel MKL library
find_package(nlohmann_json 3.2.0 REQUIRED)

# check if we have our python libs files
# sudo apt-get install python-matplotlib python-numpy python2.7-dev
find_package(PythonLibs 2.7)
if(PYTHONLIBS_FOUND AND NOT DISABLE_MATPLOTLIB)
    add_definitions(-DHAVE_PYTHONLIBS=1)
    message(STATUS "PYTHON VERSION: " ${PYTHONLIBS_VERSION_STRING})
    message(STATUS "PYTHON INCLUDE: " ${PYTHON_INCLUDE_DIRS})
    message(STATUS "PYTHON LIBRARIES: " ${PYTHON_LIBRARIES})
endif()

# check the eigen version
set(EIGEN_VER_H "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h")
if (EXISTS ${EIGEN_VER_H})
    file(READ "${EIGEN_VER_H}" STR_EIGEN_VERSION)
    # Extract the Eigen version from the Macros.h file, lines "#define EIGEN_WORLD_VERSION  XX", etc...
    string(REGEX MATCH "EIGEN_WORLD_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${GTSAM_EIGEN_VERSION_WORLD}")
    string(REGEX MATCH "EIGEN_MAJOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${GTSAM_EIGEN_VERSION_MAJOR}")
    string(REGEX MATCH "EIGEN_MINOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${STR_EIGEN_VERSION}")
    string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${GTSAM_EIGEN_VERSION_MINOR}")
    set(GTSAM_EIGEN_VERSION "${GTSAM_EIGEN_VERSION_WORLD}.${GTSAM_EIGEN_VERSION_MAJOR}.${GTSAM_EIGEN_VERSION_MINOR}")
    message(STATUS "Found Eigen version: ${GTSAM_EIGEN_VERSION}")
else()
    message(WARNING "Cannot determine Eigen version, missing file: `${EIGEN_VER_H}`")
endif ()


# display the version of other libs message to user
message(WARNING "OPENCV VERSION: ${OpenCV_VERSION}")
message(WARNING "BOOST VERSION: ${Boost_VERSION}")
message(WARNING "OPENMP VERSION: ${OpenMP_CXX_VERSION}")
message(WARNING "PCL VERSION: " ${PCL_VERSION})
message(WARNING "GTSAM VERSION: " ${GTSAM_VERSION_STRING})
message(WARNING "GTSAM EIGEN VERSION: " ${GTSAM_EIGEN_VERSION})

catkin_package(
        CATKIN_DEPENDS roscpp rosbag tf std_msgs geometry_msgs sensor_msgs nav_msgs visualization_msgs cv_bridge pcl_ros ov_core ov_eval ov_msckf velodyne_msgs
        INCLUDE_DIRS src
        LIBRARIES landmark_calibration_lib
)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations (-fopenmp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Enable debug flags (use if you want to debug in gdb)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")

# SSE3 is needed by libviso2 (its a cpu arch attribute)
# If we are on ARM we need to find an alternative
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")

#set(CMAKE_CXX_FLAGS "-march=native -msse4.2 -mfpmath=sse -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


# Include our header files
include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        /usr/local/lib/python2.7/dist-packages/numpy/core/include
        /usr/local/lib/python2.7/site-packages/numpy/core/include
        ${catkin_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${X11_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
        ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
        ${MLK_INCLUDE_DIRS}
)

set(GTSAM_LIBRARIES gtsam gtsam_unstable tbb tbbmalloc ${GTSAM_LIBS}  ${SUITESPARSE_LIBRARIES})   # TODO: automatic search libs
find_package(GTSAMCMakeTools)
include(GtsamMakeConfigFile)
include(GtsamBuildTypes)
include(GtsamTesting)
# for unittest scripts
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${GTSAM_DIR}/../GTSAMCMakeTools")


# Set link libraries used by all binaries
list(APPEND thirdparty_libraries
        ${OpenMP_CXX_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${PYTHON_LIBRARIES}
        ${catkin_LIBRARIES}
        ${GTSAM_LIBRARIES}
        ${PCL_LIBRARIES}
        ${MKL_LIBRARIES}
        ${CERES_LIBRARIES}
        )
add_definitions(${PCL_DEFINITIONS})

add_library(landmark_calibration_lib
        src/main.cpp
        src/solver/RosSolverVisulizer.cpp
        src/solver/RosSolverVisulizer.h
        src/solver/calibSolver.cpp
        src/solver/calibSolver.h
        src/sim/simulator.cpp
        src/sim/simulator.h
        src/solver/SnavelyReprojectionError.h
        src/imu_calibration_evaluation/evaluation/imu_evaluation.cpp
        src/imu_calibration_evaluation/evaluation/imu_evaluation.h
        src/imu_calibration_evaluation/calibration/imu_calibration.cpp
        src/imu_calibration_evaluation/calibration/imu_calibration.h)
target_link_libraries(landmark_calibration_lib ${thirdparty_libraries} nlohmann_json::nlohmann_json)
add_subdirectory(src/imu_calibration_evaluation/calibration/imu_tk)

set(LINK_LIBS imu_tk)
#set(CMAKE_CXX_STANDARD 14)
if (catkin_FOUND)
add_executable(main src/main.cpp)
target_link_libraries(main landmark_calibration_lib ${LINK_LIBS})

add_executable(sim_main src/sim_main.cpp)
target_link_libraries(sim_main landmark_calibration_lib ${LINK_LIBS})

add_executable(testing_main src/testing_main.cpp)
target_link_libraries(testing_main landmark_calibration_lib ${LINK_LIBS})

add_executable(HandEyeCalib src/HandEyeCalib.cpp)
target_link_libraries(HandEyeCalib landmark_calibration_lib ${LINK_LIBS})

add_executable(controller_calibration src/controller_calibration.cpp)
target_link_libraries(controller_calibration landmark_calibration_lib ${LINK_LIBS})
endif()