cmake_minimum_required(VERSION 3.0.2)
project(nextage_core)

find_package(catkin REQUIRED COMPONENTS
  nextage_bringup
  nextage_description
  nextage_hardware_interface
)

catkin_package()

