

Racecar Eklavya
===


## Table of Contents
 - [About the Project](#about-the-project)
    - [Tech Stack](#tech-stack)
    - [File Structure](#file-structure)
    - [Project Flow](#project-flow)
    - [ODG-PF Algorithm Flowchart](#ODG-PF-Algorithm-Flowchart) 
 - [Usage and Guide](#usage-and-guide)
    - [Prerequisites and Installations](#prerequisites-and-installations)
    - [Usage](#usage)
 - [Results and Demo](#results-and-demo)
 - [Future Work](#future-work)
 - [Contributors](#contributors)
 - [Acknowledgements and Resources](#acknowledgements-and-resources)
 - [Project Timeline](#project-timeline)



## About the Project

- Obstacle Avoidance Racecar is an autonomous robot designed in Solidworks and simulated and tested in ROS, Gazebo, RViz, etc. It's main objective is to avoid obstacle using [ODG-PF](https://www.hindawi.com/journals/jat/2018/5041401/) algorithm and line following through [OpenCV](https://opencv.org/) and [PID](https://en.wikipedia.org/wiki/PID_controller).
- More about the project [here](https://github.com/sameergupta4873/racecar_eklavya/).

 ### Tech Stack
   *  [Solidworks](https://www.solidworks.com/)
   *  [ROS Noetic](http://wiki.ros.org/noetic)
   *  [GAZEBO Sim](https://classic.gazebosim.org/)
   *  [Python3](https://www.python.org/about/)
   *  [RVIZ](http://wiki.ros.org/rviz)

 ### File Structure 


     📦racecar_eklavya
      ┣ 📂assets                           #contains gifs, videos and images of the results
      ┣ 📂cfg                              #config files for the sensors
      ┣ 📂include                          #include files for the plugins
      ┃ ┗ 📂vitarana_drone
      ┃ ┃ ┣ 📜gazebo_edrone_propulsion.h   #propulsion plugin include
      ┃ ┃ ┣ 📜gazebo_ros_gps.h             #gps plugin include
      ┣ 📂launch                           #launch files
      ┃ ┗ 📜drone.launch
      ┣ 📂models                           #files and meshes used to render the model
      ┃ ┗ 📂edrone
      ┃ ┃ ┣ 📂materials
      ┃ ┃ ┣ 📂meshes
      ┃ ┃ ┣ 📜model.config
      ┃ ┃ ┗ 📜model.sdf
      ┣ 📂msg                              #contains custom messages which are used to control drone functions
      ┣ 📂scripts                          #python programs used to run the drone
      ┃ ┣ 📂__pycache__
      ┃ ┣ 📜control.py                     #brain of the drone, this file needs to be executed
      ┃ ┣ 📜pid.py                         #contains the math needed to stabilise the drone
      ┣ 📂src                              #contains custom plugins used with the drone
      ┃ ┣ 📜gazebo_edrone_propulsion.cpp
      ┃ ┗ 📜gazebo_ros_gps.cpp
      ┣ 📂worlds                           #world files
      ┃ ┣ 📜drone.world
      ┣ 📜CMakeLists.txt
      ┣ 📜README.md
      ┗ 📜package.xml
 ### Project Flow
---
![Project flow](https://user-images.githubusercontent.com/95731926/193292146-253c363b-d5e7-4f59-85fa-5d25564f0838.png)
 
 ### ODG-PF Algorithm Flowchart
---
![2](https://user-images.githubusercontent.com/95731926/193303926-14bc111d-c998-436c-acae-effe77a4ccc0.png)




## Usage and Guide

### Prerequisites and Installations
___

 ####  - Tested upon [UBUNTU 20.04](https://ubuntu.com/server/docs/installation)
 ####  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
 ####  - [Gazebo](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install#InstallGazebousingUbuntupackages)

### Usage
___
1. git clone the main repo in the /src of your catkin_ws using command

```
cd ~/catkin_ws/src
```

```
git clone https://github.com/sameergupta4873/racecar_eklavya.git
```

2. git clone the [ira_laser_tools](https://github.com/sameergupta4873/ira_laser_tools.git) branch repo in the /src of your catkin_ws using

```
git clone https://github.com/sameergupta4873/ira_laser_tools.git
```

3. cd into your catkin_ws and run command 

```
cd ~/catkin_ws
```

```
catkin_make
```

```
source devel/setup.bash
```

```
roslaunch racecar_eklavya world.launch
```

4. open new terminal again cd into catkin_ws again run command 

```
source devel/setup.bash
```

```
roslaunch ira_laser_tools laserscan_multi_merger.launch
```

5. finally to avoid obstacles again open a new terminal cd into catkin_ws and run command

```
cd ~/catkin_ws
```

```
source devel/setup.bash
```

```
rosrun racecar_eklavya obstacle_avoidance.py
```

6. robot should start moving and avoiding obstacles

Results and Demo
---

Future Work
---
- [x] Desigining of Robot in Solidworks
- [x] Application of Differential Drive 
- [x] Calculating Min-Angle for Obstacle Avoidance using ODG-PF
- [x] Acheiving Min-Angle Using IMU
- [x] Line Following using OpenCv and PID 
- [ ] Acheveing Line Following and Obstacle Avoidance simultaneously

Contributors
---
- [Sameer Gupta](https://github.com/sameergupta4873)
- [Yash Rajput](https://github.com/Yashrajput9232)

Acknowledgements and Resources
---
 - [SRA-VJTI](https://github.com/SRA-VJTI)  Eklavya 2022
 - [the Basic Science Research Program through the National Research Foundation of Korea (NRF)](https://www.nrf.re.kr/eng/index) for ODG-PF resource paper .
 - [IRALABS](https://github.com/iralabdisco) for template for Laser Data Merging .
 - Our Mentors [Toshan Luktuke](https://github.com/toshan-luktuke) , [Mark Koothor](https://github.com/marck3131) and [Aryaman Shardul](https://github.com/Aryaman22102002) for guiding and helping whenever we were stuck .


Project Timeline
---
```mermaid
gantt
        SW & URDF :a, 2022-08-17, 1w
        Spawn & Drive : b,after a,1w
        Lidar-Merge & IMU: c, after b, 9d
        Obstacle Avoidance : d , after c , 9d
        Line Following : e , 2022-09-17 , 1w

```

