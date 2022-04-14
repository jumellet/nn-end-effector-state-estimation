nn-end-effector-state-estimation

# Start Simulation

The simulation is setup with  intelligent quad  tutorial, using Ardupilot and Gazebo. Follows the different commands needed to run the simulation as well as the measurements.

## Installation

TODO: Add all dependencies.

Check out tutorials of https://github.com/Intelligent-Quads/iq_tutorials

Then add the plugin https://github.com/pal-robotics/gazebo_ros_link_attacher

## Run Simulation

Need three command terminals. The first for the simulation environment.

```
roslaunch iq_sim droneWithArm.launch
```
The second one for the software simulation of Ardupilot in the loop.
```
./startsitl.sh
```
The last one to connect mavros.
```
roslaunch iq_sim apm.launch
```

# Make dataset from simulation

The measurement is done through simulated IMU using Mavros. Ardupilot limit the rate to 50Hz.
```
python3 listen_rostopic.py
```
There is then a need to move the drone in the simulation, to perturbate the end effector while measuring its ground truth.

Then data can be plotted to ensure correctness of measurments.

```
python3 plot-scatter.py
```

# Train NN

TODO: Add NN training
