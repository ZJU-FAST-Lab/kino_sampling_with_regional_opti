# kRRT* with Fast Regional Optimization and Post Refining

##  About
Planning global kinodynamic trajectories for multirotor flight. 
Suits Long range and trapped starts & goals.

__Authors__: [Hongkai YE](https://kyleyehh.github.io/) and [Fei GAO](https://ustfei.com/) from the [ZJU Fast Lab](http://www.kivact.com/).

__Related Paper__:

[Efﬁcient Sampling-based Multirotors Kinodynamic Planning with Fast Regional Optimization and Post Reﬁning](https://github.com/ZJU-FAST-Lab/kino_sampling_with_regional_opti/blob/main/misc/draft.pdf)

<p align="center">
  <img src="misc/392_2x.gif" width = "480" height = "203"/>
</p>

<p align="center">
  <img src="misc/trap3.gif" width = "480" height = "209"/>
</p>

For full videos, please watch:
[[Bilibili]](https://www.bilibili.com/video/BV1sq4y1D73J/), 
[[YouTube]](https://www.youtube.com/watch?v=1VMAB_p2uqs&t=1s)

### Methods
Front-end: 

1.Kinodynamic RRT* integrated with fast regional optimization. 
<p align="center">
  <img src="misc/front-end-vis.gif" width = "900" height = "258"/>
</p>

2.Bidirectional search.
<p align="center">
  <img src="misc/bitree_growing_Trim_Slomo.gif" width = "684" height = "320"/>
</p>

Back-end: Quadratic programming incorporating obstacles.
<p align="center">
  <img src="misc/back-end-vis.gif" width = "708" height = "263"/>
</p>


## Run The Simulation
The repo has been tested on Ubuntu 16.04 and 18.04 with ros-desktop-full installation.
By default, we use 16.04 with ROS-kinetic. If use 18.04 or above, pleause modify the last code lines in so3_control_nodelet.cpp according to the comments:

### 1. Prerequisites
The __uav_simulator__ depends on the C++ linear algebra library [Armadillo](http://arma.sourceforge.net/). You can install it by:
```
~$ sudo apt-get install libarmadillo-dev
``` 
### 2. Build on ROS
We recommand create a new catkin workspace:
```
~$ mkdir -p krrt_with_ro_ws/src
```
Change directory to _~/krrt_with_ro_ws/src_ and clone the repo:
```
~$ cd krrt_with_ro_ws/src
~/krrt_with_ro_ws/src$ git clone git@github.com:ZJU-FAST-Lab/kino_sampling_with_regional_opti.git
```
Change directory to _~/krrt_with_ro_ws_ and make:
```
~/krrt_with_ro_ws/src$ cd ..
~/krrt_with_ro_ws$ catkin_make
```

### 3. Run 
In directory _~/krrt_with_ro_ws_, set up the environment and launch the simulator:
```
~/krrt_with_ro_ws$ source devel/setup.bash
~/krrt_with_ro_ws$ roslaunch state_machine rviz.launch
```

Open another terminal, set up the environment and launch the planner:
```
~/krrt_with_ro_ws$ source devel/setup.bash
~/krrt_with_ro_ws$ roslaunch state_machine planning.launch
```
If everything goes well, you should be able to navigate the drone as the gif shows below. (Click 3D Nav Goal in the Rviz panel or press g in keyboard to selecet goal. Click down both left and right mouse buttons and drag to change the goal altitude.)

<p align="center">
  <img src="misc/fly_sim_3x.gif" width = "803" height = "400"/>
</p>
