# agvs_sim
Agvs robot sim packages

## agvs_control
This package contains the configuration files for the Gazebo controllers used by the model.

## agvs_gazebo
This package contains the configuration files to launch the Gazebo environment along with the simulated robot.

## agvs_robot_control
This package contains the simulated controlled that interfaces with Gazebo controllers.

## agvs_sim_bringup
This package contains all the launch and config files to launch all the needed packages to load the simulated environment.


## Guide

- Spawn a gazebo robot: 
    ```$ roslaunch agvs_gazebo agvs_sim_complete.launch```

- Control robot without joystick: 
    ```$ rosrun rqt_gui rqt_gui``` & add publisher (```agvs_robot_control/command```)
