project(imu_tk)

cmake_minimum_required (VERSION 3.10) 

set(Boost_USE_STATIC_LIBS   ON)
find_package(Boost REQUIRED)  
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)

aux_source_directory(src IMU_TK_CORE_SRC )
set( IMU_TK_SRC ${IMU_TK_CORE_SRC})

add_library(imu_tk ${IMU_TK_SRC})
target_include_directories(imu_tk 
    PUBLIC 
    ${PROJECT_SOURCE_DIR}/include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIRS}
)
target_link_libraries(imu_tk ${CERES_LIBRARIES})