# Imitation Learning with AutoRally
This repository contains code for imitation learning on the [AutoRally](http://autorally.github.io) simulator with [imlearn: A Python Framework for Imitation Learning](https://github.com/ACDSlab/imitation_learning).

This code runs a simulated version of the experiments described in:

**Agile Off-Road Autonomous Driving Using End-to-End Deep Imitation Learning.** Y. Pan, C. Cheng, K. Saigol, K. Lee,  X. Yan, E. Theodorou and B. Boots.  Robotics: Science and Systems (2018).

However, we have not yet released the code for the Differential Dynamic Programming (MPC-DDP) controller or the Sparse Spectrum Gaussian Process (SSGP) model used for those experiments.  Instead, the code here takes advantage of the [excellent MPPI controller provided as part of the AutoRally software package](https://github.com/AutoRally/autorally/wiki/Model-Predictive-Path-Integral-Controller-(MPPI)).

## Setup
First, visit the [AutoRally website](https://autorally.github.io) to install and set up the AutoRally software.  Then:

1. Clone this repository into the `src` subdirectory of your catkin workspace, making sure to pass the `--recursive` flag to pull in the imlearn library.

   `git clone --recursive https://github.com/ACDSlab/imitation_learning_autorally.git`
   
2. Change to the catkin workspace and install dependencies.

	`rosdep install --from-path src --ignore-src -y`

3. Compile the workspace.

	`catkin_make -DCMAKE_BUILD_TYPE=Release`
    
4. Follow the instruction 4 and 6 in the [AutoRally Setup Instructions](https://github.com/AutoRally/autorally).

5. Make `models`, `data`, and `logs` folders in the home repository

6. In every terminal, don't forget to
	```
	source devel/setup.bash
	source src/autorally_private/autorally_private_sandbox/setupEnvLocal.sh
	```
	at catkin workspace.

7. Launch the AutoRally simulation environment, the camera processing node, and the keyboard node.

	```
    roslaunch autorally_gazebo autoRallyTrackGazeboSim.launch
    roslaunch autorally_core stateEstimator.launch
    roslaunch imitation_learning_autorally single_proc_gazebo.launch
    roslaunch imitation_learning_autorally keyboard.launch
    ```

## How to Use
After Setting up, check the onboard-camera view
```
rosrun image_view image_view image:=/left_camera/cropped_image_color
```
and adjust image processing parameters in `single_proc_gazebo.launch` file.

To run the experiments, first run the expert(MPPI controller)
```
roslaunch imitation_learning_autorally mppi.launch
```
You can read the control rostopic `/mppi_controller/chassisCommand` but the vehicle will not move until you run
```
roslaunch imitation_learning_autorally autorally_mppi.launch
```
If you are asked to `switch to MANUAL mode`, make the [keyboard](http://wiki.ros.org/keyboard) window focused and press `m` on your keyboard.

If you are asked to `switch to AUTONOMOUS mode`, make the keyboard window focused and press `a` on your keyboard.

If you are asked to `Hit RED/GREEN on the runstop box` and if you are using a USB gamepad, press any of the buttons by the right stick (normally labelled X, Y, A, B or square, triangle, X, circle) to toggle the published value. If you are not using a gamepad, follow [the instruction 6 in the AutoRally Setup Instructions](https://github.com/AutoRally/autorally) to toggle the `runstopMotionEnabled` parameter in the `/chassisState` topic.


## Acknowledgements
We thank our colleagues on the [AutoRally](https://autorally.github.io) team for the AutoRally software package.  This software would not be possible without their effort.

## Software Authors
Keuntaek Lee, Kamil Saigol, Gabriel Nakajima An, Yunpeng Pan, Xinyan Yan

## Citation
If you use this software in your work, please cite:

**Agile Off-Road Autonomous Driving Using End-to-End Deep Imitation Learning.** Y. Pan, C. Cheng, K. Saigol, K. Lee,  X. Yan, E. Theodorou and B. Boots.  Robotics: Science and Systems (2018).
