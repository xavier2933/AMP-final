# AMP FINAL PROJECT
Here is my final project for ASEN 5254 Algorithmic Motion Planning, taught in fall 2024 by Morteza Lahijanian. In this project I implement two task and motion planners, both using RRT for motion planning. One task planner follows a queue structure, while the other follows an automaton to ensure that the robot respects environmental conditions. 

## Code
This repositiory is adapted from an OMPL demo repo made by TA Yusif Razzaq. The run.sh script and CMakeLists.txt are his, with some modifications. visualize3d.py was AI generated to aid in making plots. Source files are `src/planLTL.cpp` and `src/planConsecutive.cpp`.

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
bash run.sh planConsecutive
```
To launch the LTL planner:
```
bash run.sh planLTL
```

### Plotting hospital location
Due to limited knowledge of matplotlib, I was unable to pass a parameter to my plotting script that enables\disables the plotting of the hospital. If you wish to see the hospital on the plot, uncomment lines 170, 178, and 186 of `visualize3d.py`. It is pretty obvious when the sub visits the hospital though, the coordinates are `(-9.0, 0.0, 0.0)`.
