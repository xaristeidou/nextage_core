Caution! This branch is to use only with the SSD disk marked with "SSD 44" installed on robot side!

# NextageA Hardware Interface

This package exposes NextageA joints via `ros_control` as position joint interfaces.

## Dependencies
 * Ubuntu 18.04
 * ROS Melodic
 * [nextage_description](https://github.com/ipab-slmc/nextage)
 * Binary dependencies via `rosdep`

## Usage

 1. After first time the QNX is started, start the hardware interface:
    ```
    roslaunch nextage_hardware_interface nextage_hardware_interface.launch
    ```
 1. Enable the servos by calling the servo service, e.g.:
    ```
    rosservice call /nextage/nextage_hw/servo "data: true"
    ```
The hardware interface node will be started and a four trajectory controllers will be loaded (left arm, right arm, torso, head). These work well with MoveIt (e.g. [nextage_moveit_config](https://github.com/ipab-slmc/nextage)) or using [rqt joint trajectory controller](http://wiki.ros.org/rqt_joint_trajectory_controller).

## Configuration

 * Controllers are specified in the `config/controllers.yaml` file and started in the `launch` file.
 * The `config/hardware.yaml` specifies the `ros_control` hardware interface to start, the controller rate (100Hz), the joints to control (all 15 joints of the NextageA), and input/output ports.
   * Each input port will expose a topic of type `std_msgs/Bool` with the specified name in the namespace `nextage/nextage_hw/dio/inputs/<name>`.
   * Each output port port will expose a topic of type `std_msgs/Bool` with the specified name in the namespace `nextage/nextage_hw/dio/outputs/<name>`. Each output port will also expose a service of type `std_srvs/Bool` with the specified name in the namespace `nextage/nextage_hw/dio/outputs/<name>`.
   * IO interfaces get updated at 1Hz.
 * The files in `config/multiplot/` are plot layouts for [rqt_multiplot](http://wiki.ros.org/rqt_multiplot) to inspect the controller signals.

## Safety interface
The NextageA's estop/reset/power controller was replaced by a software safety interface in Arduino. To trigger, disable and reset a hardware estop, enter the below commands, where ```ns``` is the robot namespace.
```
rosservice call /ns/nextage_hw/estop "data: true"
rosservice call /ns/nextage_hw/estop "data: false"
rosservice call /ns/nextage_hw/reset
```

For all Nextage robots, there are now also available services to software estop the robot. This is useful if you want to continously stop and start the robot during use without triggering the full emergency estop. The commands are:
```
rosservice call /nextage/nextage_hw/soft_estop "data: true"
rosservice call /nextage/nextage_hw/soft_estop "data: false"
rosservice call /nextage/nextage_hw/reset
```

If a Nextage robot has an Arduino software safety interface, the start up procedure can be automated by using these services.
- ```src/nextage_safety``` configures the (hardware) estop/reset/power services and publishes diagnostics
- ```src/nextage_startup``` uses these services to power on the robot and reset it after a given time delay
- ```src/nextage_runbridge``` runs a script that is needed in the old API for the robot to work. Disregard for the new API.

For an example on how to implement these scripts using ```robot_upstart```, see ```chonk_bringup/scripts/install```.
