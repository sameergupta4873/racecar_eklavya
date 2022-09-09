
# A Real-Time Obstacle Avoidance Method for Autonomous Vehicles 
[Click for Research Paper](https://www.hindawi.com/journals/jat/2018/5041401/#abstract)

Autonomous vehicle systems need path planning, indoor and outdoor 
localization, obstacle avoidance, object detection, and 
classification of cars, humans, pets, and traffic signs,
 signals, etc.

 ![Types of Path Planning](https://www.researchgate.net/profile/Maram-Alajlan/publication/324482833/figure/fig1/AS:696133626122241@1542982598420/Path-Planning-Categories.png)

 We would be focusing on the Path Planning based on algorithm 


## Abbreviations used in this document 
1. ODG-PF: Obstacle Dependent Gaussian Potential Field
1. GIS: Geographic Information System
1. PFM: Potential Field Method 
1. VFH: Vector Field Histogram 
1. DWA: Dynamics Window Approach 
1. MPC: Model Predictive Control 
1. ND: Nearness Diagram 
1. FGM: Follow the Gap Method 
1. AFPFM: Advanced Fuzzy Potential Field Method
1. LRF: Laser Range Finder
1. IMU: Inertial Measurement Unit

## Sensors used in our vehicle
1. Sonar Sensors
2. Laser Range Finder 
3. Stereo Vision Sensors
4. LIDAR
5. 3D Depth Sensors 
1. Inertial Measurement Unit

## Working of LIDAR

![LIDAR](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.001a.jpg)

In LiDAR, laser light is sent from a source (transmitter) and reflected from objects in the scene. The reflected light is detected by the system receiver and the time of flight (TOF) is used to develop a distance map of the objects in the scene
![lidar](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.001b.jpg)


## Algorithms for our vehicle

Potential field-based methods use an artificial potential field consisting of an attractive field and a repulsive field. The attractive field attracts the vehicle toward the goal while the repulsive field repels the vehicle from obstacles


1. Obstacle Dependent Gaussian Potential Field
2. Potential Field Method
3. Follow the Gap Method
 4. Advanced Fuzzy Potential Field Method 

## The Conventional Potential Field Method

The following equations depict conventional potential field equations:

_f_<sub>total</sub> = _f_<sub>attractive</sub> + _f_<sub>repulsive</sub> ..............(1)

_f_<sub>attractive</sub> = k<sub>attractive</sub>$(r_ g - r ) / |r_g - r|$........(2)


$r_g$ is the position vector of the goal point and $r$ is the position vector of the vehicle

for d<sub>i</sub> < d<sub>max</sub>


$f repulsive = \left( -k_r\sum_{i=1}^n(1/d_i -1/dmax)
s_i \right) $


$k_r$ = proportionality constant for repulsive nature 



for any other case 

$frepulsive = 0$




![visual representation](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.002.jpg)

### PROS of PFM
1. Paths are generated in real time 
2. Smooth paths are generated
3.  Planning can be coupled directly to a control algorithm
### CONS of PFM
1. The vehicle will get stuck in local minima 
![local minima](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.003.jpg)
2. Due to the previous limitation it is used only in local path planning 
3. Backtracking or randomwalk is used to escape local minima 

## The Follow the Gap Method
FGM is not based on an artificial potential field, but its guild angle decision is somewhat similar. Besides, its obstacle detecting and gap finding methods are somewhat like our ODG-PF.

The main idea of FGM is to find the maximum gap in front of the vehicle and calculate the middle angle of the gap (θ<sub>gap</sub>) first. From the received data, we know where the obstacles are and we should enlarge the obstacles to a degree with regard to the width of the vehicle. By enlarging the obstacles, we can easily know whether a gap is good for the vehicle to go through or not. This method also attains the angle toward the goal for which the direction decision is carried out using the weighted average of the angle of maximum gap and the angle toward the goal.


![image](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.004.jpg)
in this figure we can see that the sensor has plotted the data (blue sqaures) and we have enlarged the object by 1°-2° to ensure that our vehicle passes through it smoothly 

looking at this example 
![image](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.001a.jpg)
![iamge](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.001b.jpg)
the maximum gap is in -14° ~ 18° = θ<sub>gap</sub>
 
 
  range the medium angle = 2° = θ<sub>goal</sub>

Guide angle is calculated by θ

$θ =(h* θgap + θgoal)/(h+1)$

where h = α/d<sub>min</sub>

d<sub>min</sub>  is the minimum distance from all of the distance data from the range sensors

α is a value that we should decide to avoid an obstacle and travel efficiently toward the goal, can be calculated by  hit and trial

For small values of α the vehicle will move in a more straight path towards the goal but is likely to collide with the obstacles 

For large  values of α the vehicle will move safely around the obstacles and to our goal but the path will not be efficient 

If our vehicle is close to the obstacle d<sub>min</sub> will be small and $h*θgap$ will become greater than $θgoal$, hence it will travel more towards $θgap$ 

If our vehicle is far from obstacle d<sub>min</sub> will be large $h*θgap$ will be relatively small so it will travel more towards $θgoal$

## The Advanced Fuzzy Potential Field Method

$f repulsive = l*(m/n)$

$l=\left( -k_r\sum_{i=1}^n(E+d_i)/dmax \right) $


$m=\left( \sum_{p=1}^nw_p(dmax/(E+d_p))(1/d_p - 1/dmax)s_p \right)$

$n = \left( \sum_{p=1}^nw_p \right)$

$dmax $ is the max range of the sensor 

$E$ is the safety term considering the vehicle's outer hull

$k_r$ is the repulsive field coefficient with basically the same meaning in that of PFM

$w_p$ is a weight that in TS fuzzy is the minimum value of three membership functions U<sub>d<sub>p</sub></sub> , U<sub>Φ<sub>p</sub></sub>, and u<sub>ψ<sub>p</sub></sb>   

[Click for more info](https://scholar.google.com/scholar_lookup?title=Dynamic%20motion%20planning%20for%20mobile%20robots%20using%20potential%20field%20method&author=S.%20S.%20Ge%20and%20Y.%20J.%20Cui)

## The Obstacle-Dependent Gaussian Potential Field


 The main idea behind this method is that, after receiving distance data from the range sensor(s) (as shown in Figure 1(b)), we consider only the objects that are within the threshold range (2 m, for example), enlarge the obstacles with regard to the vehicle’s width, and construct a Gaussian (repulsive) potential field from them. Next, we calculate the attractive field from the yaw angle information from an IMU. The total field is made of these two fields, and, from it, we choose the angle with the minimum total field value.
![Flow chart of ODFPF](https://static-02.hindawi.com/articles/jat/volume-2018/5041401/figures/5041401.fig.005.jpg)

Flow chart