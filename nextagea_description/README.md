## NextageA Description

This package contains the xacro and mesh files needed to create a vanilla NextageA robot model.

The vanilla model is composed of a robot (torso), base, and hand tools. When instantiating the robot, you can specify a namespace and if you want to include the gazebo ros control plugin. When instantiating a tools (camera), you need to specify which hand it will be attached to and if you want to use cables.

An example formulation is given below.

```
<xacro:nextagea_robot namespace=/nextagea/ nextagea_gazebo_ros_control=true"/>
<xacro:nextagea_base/>
<xacro:nextagea_tools prefix="L" use_cables="true"/>
<xacro:nextagea_tools prefix="R" use_cables="false"/>
```
