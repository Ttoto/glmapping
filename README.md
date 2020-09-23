# glmapping(Global-Local Mapping Kits)
### Introduction
This mapping kit is a 3D occupancy voxel map designed for the MAV/mobile robot navigation application. Currently, most of the navigation strategies are a combination of global planning and local planning algorithms. Global planning focuses on finding the least cost path from the current position to the destination. And local planning is to replan the trajectory to avoid obstacles. This mapping kit processes the perception information separately. The globalmap, on the cartesian coordinate system, is a probability ESDF(Euclidean Signed Distance Field) map. While the localmap, on cylindrical coordinates system, has an excellent dynamic performance.

### Demo Video

### Usage
Clone this repository to catkin src folder say: ~/catkin_ws/src
````
cd ~/catkin_ws/src
git clone https://github.com/Ttoto/glmapping.git
````
Compile
````
cd ~/catkin_ws/
catkin_make
````

### Maintainer
[Shengyang Chen](https://www.polyu.edu.hk/researchgrp/cywen/index.php/en/people/researchstudent.html)(Dept.ME,PolyU): shengyang.chen@connect.polyu.hk <br />

