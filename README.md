# AMP FINAL PROJECT
Here is my final project for ASEN 5254 Algorithmic Motion Planning, taught in fall 2024 by Morteza Lahijanian. In this project I implement two task and motion planners, both using RRT for motion planning. One task planner follows a queue structure, while the other follows an automaton to ensure that the robot respects environmental conditions. 

## Code
This repositiory is adapted from an OMPL demo repo made by TA Yusif Razzaq. The run.sh script and CMakeLists.txt are his, with some modifications. visualize3d.py was AI generated to aid in making plots. Source files are src/3dGeo.cpp and src/3dGeoSeq.cpp.

## Configuration
This code has only been tested on my Ubuntu 22 machine. It only needs the OMPL library for motion planning, and matplotlib for plotting. I've used Python 3, and version 3.9.3. OMPL version is 1.6.0, installed from the command line. Run these commands if either is not installed.
```
apt-get install libompl-dev ompl-demos
```
```
pip3 install matplotlib==3.9.3
```

## Running code
To launch the consecutive planner: 
```
bash run.sh 3dGeoSeq
```
To launch the LTL planner:
```
bash run.sh 3dGeo
```