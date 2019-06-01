# Multi_Car_Parking
Multi Car and High Density Parking Coodination

## Update log
###06/02/2019
1. **[large-map branch]** Added circular motion to turning part, but the collision avoidance method is not modified.
2. **[large-map branch]** Combined the spot allocation funtion into the initialization function of Vehicle Class.
3. To-do: Clean the code, test a best circular motion, and fix the collision detection while turning.

### 05/14/2019
1. **[large-map branch]** Fixed some bugs in allocation and now 44 cars can park. The hold simulation need about 30mins on the Thinkpad Desktop.

### 05/13/2019
1. Instead of parallelly making collision free decision, now the vehicles have their priorities. The first car in the queue has the highest priority. The collision check only exists when considering other higher-priority counterparts.
2. **[large-map branch]** Constructed a larger map and rewrote the dynamics and maneuver offsets.
3. **[large-map branch]** The larger map works and the corresponding maneuvers are also shifted.

### 05/12/2019
Back to ROS
1. Changed the initialization as randomized. The starting time, the lane and end pose are all chosen randomly. The end spot is chosen from the farest end to nearest.
2. The random parameters can be saved and reloaded to recover the last experiment.

### 04/25/2019
1. In MATLAB, implemented the algorithm in reference[2] about intersection conflict resolution. 
2. Three cars use double integrator model on pre-planned path. The safety constraints are formulated as QCQP and then relaxed as SDP. For rank-1 constraint, randomization technique is adopted.
3. This technique in intersection scenario can be adopted to parking lot.

### 04/02/2019
1. Fixed the bug in cost writting function when writing the parking trajectory to cost map. Now all the grids that the car body covers will be registered in the cost map. The collision is therefore totally avoided.
2. The cost map is now a 2-tuple (cost, car_num) which marks the cost and the producer of it at the same time. Cars will not identify the cost produced by itself as collision.
3. Modified the class of some functions.
4. **Now, the naive Centralized Critical-region Regulation approach is finished. Some drawbacks of this method include: 1) The input is instantly changable constant velocity / zero, which is unrealistic and uncomfortable driving; 2) The efficiency is bad. The next car will need to wait until all process of the previous car is finished. 3) The completeness is not gauranteed if there is some strange incoming order or target perference.**


### 03/31/2019
1. Set up roslaunch and parameter service. Now H-OBCA, plot and control can be started by a single roslaunch.
2. Edited the cost map update: In the loop of every car, the map is firstly reset, then loop around every other car to collect the location and maenuver data
3. Edited the collision detection: If the car is going straight, check the cost in front of the path; if the car is in the middle of the parking maneuver, check the collision in the rest of the path.
4. The algorithm works when there is only 5 cars. When added the number of cars to 12, a dead lock happens between cars. Therefore, **the time information should be considered in the collision avoidance, or the cost map should include producer name of the corresponding cost**.

### 03/30/2019
1. H-OBCA can now save and load from file
2. The cost map can now be visualized
3. Fixed some bug about multiple cars. Abd added a lot functions for multiple cars. Now the simulation can be carried out for multiple cars.
4. The problem still exist: 1) The maneuver need to be specified from the begininng; 2) The collision avoidance is not always reliable 

### 03/26/2019
1. Added the "Go straight" function and the maneuver client, now single car can go along straight lane and park according to the specificed slot
2. Added the cost map class, and define a naive cost map for the second car to identify the obstacle
3. Added the second car to show up after some delay and avoid the first car by looking up the cost map
4. There are a lot of bugs when two car is acting, need to fix next time. The current version is just a hard-coded demo (The plot function is also ugly changed to make a temperary 2-car demo)

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
2. Katriniok, Alexander, Peter Kleibaum, and Martina Joševski. "Automation of Road Intersections Using Distributed Model Predictive Control." Control Strategies for Advanced Driver Assistance Systems and Autonomous Driving Functions. Springer, Cham, 2019. 175-199.