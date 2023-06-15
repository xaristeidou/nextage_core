#include <unistd.h>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <typeinfo>
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <combined_robot_hw/combined_robot_hw.h>

struct ThreadArgs
{
	controller_manager::ControllerManager* cm;
	combined_robot_hw::CombinedRobotHW* hw;
	double loop_hz;
};

void* update_loop(void* args)
{
	ThreadArgs* arg = reinterpret_cast<ThreadArgs*>(args);
	controller_manager::ControllerManager* cm = arg->cm;
	combined_robot_hw::CombinedRobotHW* hw = arg->hw;
	ros::Rate rate(arg->loop_hz); // update rate
	ros::Duration period = rate.expectedCycleTime();

	// Start control loop
	ROS_INFO_STREAM("Start control loop " << arg->loop_hz << "Hz");
	while(ros::ok()) {
		ros::Time now = ros::Time::now();
		hw->read(now, period);
		cm->update(now, period);
		hw->write(now, period);
		rate.sleep();
	}
	return nullptr;
}

int main(int argc, char** argv)
{
	// For display during read
	std::cout << std::fixed << std::setprecision(2) << std::setfill(' ');

	// Init ROS node
	ROS_INFO("Initialize node");
	ros::init(argc, argv, "NextageAHardwareInterface_node");
	ros::AsyncSpinner spinner(2);
	ros::NodeHandle nh;

	// Init Combined Robot HW
	ROS_INFO("Initialize combined robot hw");
	combined_robot_hw::CombinedRobotHW hw;
	bool init_success = hw.init(nh, nh);

	// Init Controller manager
	ROS_INFO("Initialize controller manager");
	controller_manager::ControllerManager cm(&hw, nh);

	// Set update rate
	double loop_hz;
	nh.param("nextage_hw/loop_hz", loop_hz, 0.1);

	ThreadArgs args;
	args.hw = &hw;
	args.cm = &cm;
	args.loop_hz = loop_hz;

	pthread_t tid;
	pthread_create(&tid, NULL, update_loop, &args);

	ros::Rate rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
