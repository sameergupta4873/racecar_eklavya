# Obstacle Avoidance Racecar

# Usage : -

1 ) git clone the main repo in the /src of your catkin_ws using command

```cd ~/catkin_ws```

```cd src```


```git clone https://github.com/sameergupta4873/racecar_eklavya.git```

2 ) git clone the ira_laser_tools branch repo in the /src of your catkin_ws using

```git clone -b ira_laser_tools https://github.com/sameergupta4873/racecar_eklavya.git```

3 ) cd into your catkin_ws and run command 

```cd ~/catkin_ws```

```catkin_make```

```source devel/setup.bash```

```roslaunch racecar_eklavya world.launch```

4) open new terminal again cd into catkin_ws again run command 

```source devel/setup.bash```

```roslaunch ira_laser_tools laserscan_multi_merger.launch```

5 ) finally to avoid obstacles again open a new terminal cd into catkin_ws and run command

```cd ~/catkin_ws```

```source devel/setup.bash```

```rosrun racecar_eklavya obstacle_avoidance.py```

6 ) robot should start moving and avoiding obstacles
