## NextageA Description

This package contains the xacro and mesh files needed to create a vanilla NextageA robot model.

To add sensors and actuators to your project, simply include the desired xacro file in the main NextageA description file ```nextagea.urdf.xacro```. See the [Nextage Wiki](https://github.com/ipab-slmc/nextagea_wiki/wiki/Nextage) for more detail about the extensions.

All current options are presented below.
```
<xacro:include filename="$(find nextage_extensions)/urdf/intel_realsense.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/intel_realsense_azure_topics.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/imu.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/hexe.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/robotiq_140_gripper.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/robotiq_85_gripper.xacro" />
<xacro:include filename="$(find nextage_extensions)/urdf/robolimb.xacro" />
```
