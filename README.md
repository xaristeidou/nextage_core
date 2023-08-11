# Nextage Core

This repo contains all the basic code to operate a Nextage robot.

Currently a work in progress...

# Install

1. Change directory: `cd /path/to/catkin_ws/src`
2. Clone the repository (make sure you include the `--recursive` flag)
  - (ssh) `git clone --recursive git@github.com:ipab-slmc/nextage_core.git`
  - (https) `git clone --recursive https://github.com/ipab-slmc/nextage_core.git`
3. Install dependencies: `rosdep update ; rosdep install --from-paths ./ -iry`
4. Build catkin workspace: `catkin build -s`

## Packages (Some to be added still)

### nextage_description

### nextage_gazebo

### nextage_moveit_config

### nextage_hardware_interface

### nextage_bringup


See the IPAB Wiki for more details, including guides and documentation for the Gazebo simulator: https://github.com/ipab-slmc/nextage_wiki/wiki
