# landmark_calibration
For Pimax-VR controller LED position calibration

## Dependencies

- OpenVINS - https://docs.openvins.com/gs-installing.html
- ceres-solver - http://ceres-solver.org/installation.html
- Eigen3 - https://eigen.tuxfamily.org/dox/GettingStarted.html
- ROS Kinetic or Melodic - https://www.ros.org/ (For ubuntu user)

For ceres error can be solved by using branch 1.14.0:
```cmd
tar zxf ceres-solver-1.14.0.tar.gz
mkdir build
cd build
cmake ..
make -j3
make test
make install
```

## Dataset format
This datasets format was modified base on [BAL](http://grail.cs.washington.edu/projects/bal/) dataset
```cmd
<num_cameras> <num_points> <num_observations>
<camera_index_1> <point_index_1> <x_1> <y_1>
...
<camera_index_num_observations> <point_index_num_observations> <x_num_observations> <y_num_observations>
<camera_1>
R
t
...
<fx>
<cx>
<cy>
<k1>
...
<k4>
R
t
<camera_num_cameras>
<point_1>
...
<point_num_points>
```
Where, there camera and point indices start from 0. Each camera is a set of 19 parameters - R_GtoC,t,fx,cx,cy,k1 to k4,R_DtoG, P_DinG. The rotation R is specified as a Rodrigues' vector.

This dataset format is only for fisheye model, other camera model may vary. fx and fy are normally same, so we only use fx in our case. 

## Running example
```cmd
./main /path-to-dataset-text-file
```

![alt text](img/Screenshot%20from%202022-08-16%2017-58-20.png)

Although now the results are pretty close to the ground truth(Yellow dots are optimized, and blue dots are ground truth), there still have some we can do to improve the results. Such as add constrain to the CAMERA pose and get more accurate results on DEVICE-TO-GLOBAL pose.