# Multi_Car_Parking
Multi Car and High Density Parking Coodination
## Update log
### 03/25/2019
1. Built the plotting node which can subscribe to the car state and input topic and plot the car box and trajectory
2. Small adjustion to the slot width parameters

### 03/09/2019
1. Set up the ROS Package file structure, msg and srv
2. Package `parking` is the main package for the future design of parking algorithm. The msg, srv are written here.
3. A service is designed to let package `parking` get the desired parking maneuver from H-OBCA. The testing client node is successfully implemented.
4. Reorganized the H-OBCA code structure and make `main.jl` as a function. Fixed some bugs in the code.
5. `run_HOBCA` is finished to iterate all possible parameters and generate the maneuver dictionary. 
6. Made the H-OBCA as a ros package and define a server node to provide parking module with requested maneuver.

### 03/04/2019
Reorganized the files and archived MATLAB code into a seperate folder

## Reference
1. Zhang, Xiaojing, Alexander Liniger, and Francesco Borrelli. "Optimization-based collision avoidance." arXiv preprint arXiv:1711.03449 (2017). (Github repo: <https://github.com/XiaojingGeorgeZhang/H-OBCA>)