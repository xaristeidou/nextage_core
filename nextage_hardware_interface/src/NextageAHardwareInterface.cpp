#include <nextage_hardware_interface/NextageAHardwareInterface.h>
#include <string>

namespace nextage_hardware_interface
{

// Generate error message
std::runtime_error error_msg(std::string header, CORBA::Exception& e)
{
	return std::runtime_error(header + std::string(e._name())
		+ " " + std::string(e._rep_id()));
}

// string format of CORBA completion status
std::ostream& operator<<(std::ostream& os, const CORBA::CompletionStatus& status)
{
	switch (status)
	{
	case CORBA::CompletionStatus::COMPLETED_YES:
		os << "YES";
		break;
	case CORBA::CompletionStatus::COMPLETED_NO:
		os << "NO";
		break;
	case CORBA::CompletionStatus::COMPLETED_MAYBE:
		os << "MAYBE";
		break;
	}
	return os;
}

// Constructor and Destructor
NextageAHardwareInterface::NextageAHardwareInterface() { timer = 0; }

/*o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o
    INITIALIZATION
  o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o*/

// Main initialization function
bool NextageAHardwareInterface::init(
	ros::NodeHandle& root_nh,
	ros::NodeHandle& robot_hw_nh)
{
	initModel(robot_hw_nh);
	initCORBA();

	for(int tries = 0; ; tries++)
	{
		try
		{
			initAPI();
			break;
		}
		catch (std::runtime_error& e)
		{
			if (tries < 30)
			{
				ROS_INFO_STREAM("Connecting to QNX failed. Retrying...");
				ros::Duration(1.0).sleep();
				continue;
			}
			else
			{
				throw e;
			}

		}
	}
	initHWInterface();
	read(ros::Time::now(), ros::Duration());
	setCommandToActual();
	initialised = true;
	initHWPublisher(robot_hw_nh);
	ROS_INFO("Hardware Interface Initialization Complete.");

	// Diagnostics
	robot_status.name = "NextageA";
    dia_soft_estop.key = "soft estop";
	dia_soft_reset.key = "soft reset";
	soft_reset = true;
	fake_estop = false;

	switchServo(true);
	return true;
}

NextageAHardwareInterface::~NextageAHardwareInterface()
{
	if (initialised)
		switchServo(false);
}

// Initialize Robot model from hardware.yaml file
void NextageAHardwareInterface::initModel(
	ros::NodeHandle& robot_hw_nh)
{
	ROS_INFO("  - Initialize Model");

	// Get joint names and num of joints
	robot_hw_nh.getParam("joints", joint_name);
	num_joints = joint_name.size();

	// Resize vectors
	joint_position_state.resize(num_joints);
	joint_position_state_stopped.resize(num_joints);
	joint_velocity_state.resize(num_joints);
	joint_effort_state.resize(num_joints);
	joint_position_command.resize(num_joints);
	joint_position_state_last.resize(num_joints);

	// Initialize all values to 0
	for( int i = 0; i < num_joints; i++ )
	{
		joint_position_state[i] = 0;
		joint_position_state_stopped[i] = 0;
		joint_velocity_state[i] = 0;
		joint_effort_state[i] = 0;
		joint_position_command[i] = 0;
		joint_position_state_last[i] = 0;
		joint_limits_interface::getJointLimits(joint_name[i],
			robot_hw_nh, joint_limits);
	}
}

// Initialize CORBA service
void NextageAHardwareInterface::initCORBA()
{
	ROS_INFO("  - Initialize CORBA");

	// Initialize CORBA references
	char buf[CORBA_ARGNUM][CORBA_BUFSIZE];
	strcpy(buf[0], CORBA_OPT1);
	strcpy(buf[1], CORBA_ARG1);
	strcpy(buf[2], CORBA_OPT2);
	strcpy(buf[3], CORBA_ARG2);
	char *argv[CORBA_ARGNUM];
	int argc = CORBA_ARGNUM;
	for(int idx = 0; idx<CORBA_ARGNUM; idx++) { argv[idx] = buf[idx]; }
	try
	{
		orb = ::CORBA::ORB_init(argc, reinterpret_cast<char **>(argv));
		obj = orb->resolve_initial_references("NameService");
	}
	catch (CORBA::Exception& e)
	{
		throw(error_msg("", e));
	}

	rootContext = CosNaming::NamingContext::_narrow(obj);
	ROS_INFO("GetOrb&Context: Success");
}

CORBA::Object_ptr NextageAHardwareInterface::getObject(const std::string& objname)
{
	CosNaming::Name name;
	name.length(1);
	name[0].id   = objname.c_str();
	name[0].kind = (const char*)"";
	return rootContext->resolve(name);
}

// Initialize Nextage API
bool NextageAHardwareInterface::initAPI()
{
	ROS_INFO("  - Initialize API");

	// get services
	Joint = jointAnglePlugin::_narrow(getObject("joint"));
	Status = statusPlugin::_narrow(getObject("status"));
	Servo = servoPlugin::_narrow(getObject("servo"));
	Dio = dioPlugin::_narrow(getObject("dio"));
	ROS_INFO("  - Initialize API finish");
	return true;
}

// Initialize ros_control Hardware Interface
void NextageAHardwareInterface::initHWInterface()
{
	ROS_INFO("  - Initialize Interface");

	// Connect joint_state_interface and position_joint_command
	for (int i = 0; i < num_joints; i++ )
	{
		// Joint State Interface
		hardware_interface::JointStateHandle state_handle(joint_name[i],
			&joint_position_state[i],
			&joint_velocity_state[i],
			&joint_effort_state[i]);
		joint_state_interface.registerHandle(state_handle);

		// Position joint Interface
		hardware_interface::JointHandle pos_handle(state_handle,
			&joint_position_command[i]);
		position_joint_interface.registerHandle(pos_handle);

		// Position Joint Saturation Interface
		joint_limits_interface::PositionJointSaturationHandle limit_handle(
			pos_handle, joint_limits);
		position_joint_saturation_interface.registerHandle(limit_handle);
	}

	// Register
	registerInterface(&joint_state_interface);
	registerInterface(&position_joint_interface);
	registerInterface(&position_joint_saturation_interface);
}

// Initialize publishers
void NextageAHardwareInterface::initHWPublisher(
	ros::NodeHandle& nh)
{
	ROS_INFO("  - Initialize Publisher");

	number_of_board = (int)Dio->getDioBoardNum();
	unsigned int n_inputs = number_of_board * 32;
	unsigned int n_outputs = number_of_board * 32;

	input_buffer.resize(n_inputs);

	XmlRpc::XmlRpcValue inputs;
	XmlRpc::XmlRpcValue outputs;

	if (nh.hasParam("inputs"))
	{
		ROS_INFO_STREAM("DIO inputs:");
		nh.getParam("inputs", inputs);
		for (auto& it : inputs)
		{
			unsigned int id = static_cast<int>(it.second);
			std::string name = static_cast<std::string>(it.first);
			if (id > n_inputs) throw std::runtime_error("Invalid input ID!");
			ROS_INFO_STREAM("  - " << id << " -> " << name);
			pub_din[id].reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh, "dio/inputs/" + name, 1));
		}
	}
	else
		ROS_INFO_STREAM("No DIO inputs specified");

	if (nh.hasParam("outputs"))
	{
		ROS_INFO_STREAM("DIO outputs:");
		nh.getParam("outputs", outputs);
		for (auto& it : outputs)
		{
			unsigned int id = static_cast<int>(it.second);
			std::string name = static_cast<std::string>(it.first);
			if (id > n_inputs) throw std::runtime_error("Invalid output ID!");
			ROS_INFO_STREAM("  - " << id << " -> " << name);
			svc_dout[id] = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("dio/outputs/" + name, 
				[&, id](std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) -> bool {
					return writeDigitalOutput(req, res, id);
				});
		}
	}
	else
		ROS_INFO_STREAM("No DIO inputs specified");

	pub_command_feedback.init(nh, "command_feedback", 1);
	pub_command_feedback.msg_.name = joint_name;

	svc_servo = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("servo",
				[&](std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) -> bool {
					switchServo(req.data);
					return true;
				});

	// Get alpha filter parameter: 1 = accept new value, 0 = keep old value
	nh.param<double>("alpha", velocity_alpha, 0.05);
	if (velocity_alpha >= 1.0)
	{
		velocity_alpha = 1.0;
		ROS_WARN_STREAM("Alpha filter disabled");
	}
	else if (velocity_alpha <= 0.0)
	{
		velocity_alpha = 0.0;
		ROS_WARN_STREAM("Velocity measurement disabled");
	}

	nh.param<double>("transition_time", transition_time, 5.0);

	last_servo_enabled = false;
	last_estop = false;
	estop_buffer.writeFromNonRT(last_estop);

	pub_soft_estop.reset(new realtime_tools::RealtimePublisher<std_msgs::Bool>(nh, "soft_estop", 1));

	svc_soft_estop = nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("soft_estop",
				[&](std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) -> bool {
					bool result = estopCallback(req);
					if (!result)
					{
						ROS_ERROR_STREAM("NextageA soft estop service call failed");
					}
					return true;
				});
	svc_soft_reset = nh.advertiseService<std_srvs::Trigger::Request, std_srvs::Trigger::Response>("soft_reset",
				[&](std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) -> bool {
					bool result = resetCallback();
					if (!result)
					{
						ROS_ERROR_STREAM("NextageA soft reset service call failed");
					}
					return true;
				});

	pub_diagnostics.reset(new realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>(nh, "/diagnostics", 1));
}

bool NextageAHardwareInterface::estopCallback(std_srvs::SetBool::Request &req)
{
	if (!fake_estop & req.data)
	{
		fake_estop = true;
		soft_reset = false;
		estop_buffer.writeFromNonRT(true);
		ROS_WARN_STREAM("NEXTAGEA SOFT E-STOP TRUE");
		return true;
	}
	else if (fake_estop & !req.data)
	{
		fake_estop = false;
		ROS_WARN_STREAM("NEXTAGEA SOFT E-STOP FALSE");
		return true;
	}
	return false;
}

bool NextageAHardwareInterface::resetCallback()
{
	if (!fake_estop & !soft_reset)
	{
		ROS_ERROR_STREAM("NEXTAGEA SOFT E-STOP RESET");
		estop_buffer.writeFromNonRT(false);
		soft_reset = true;
		return true;
	}
	return false;
}

/*o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o
    READ & WRITE FOR ROS_CONTROL
  o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o*/

// Read Actual Joint Angle from Robot
void NextageAHardwareInterface::read(
	const ros::Time& time,
	const ros::Duration& period)
{
	// Initialize parameters for reading
	CORBA::Octet startJoint = 0;
	CORBA::Octet jointNum = num_joints;
	statusPluginTypes::kindType outKind;
	statusPluginTypes::JointValues_var jointAngles;
	commonStatus::statSequence_var state = new commonStatus::statSequence;
	outKind = statusPluginTypes::K_MM_DEG + statusPluginTypes::K_STATUS;

	// Get actual joint value
	Status->getJointValue(startJoint, jointNum, outKind, jointAngles, state);

	// Get current status
	::servoPluginTypes::LngArray15 status;

	try {
		Servo->getStatus(status);
	} catch (const CORBA::COMM_FAILURE &e) {
		ROS_ERROR_STREAM("COMM_FAILURE [name: " << e._name() << ", repository ID: " << e._rep_id() << "]");
		ROS_ERROR_STREAM("COMM_FAILURE: " << e.NP_minorString() << " (minor code: " << e.minor() << ", completed: " << e.completed() << ")");
		// keep current joint states and skip reading new state
		return;
	} catch (const CORBA::TRANSIENT &e) {
		ROS_ERROR_STREAM("TRANSIENT [name: " << e._name() << ", repository ID: " << e._rep_id() << "]");
		ROS_ERROR_STREAM("TRANSIENT: " << e.NP_minorString() << " (minor code: " << e.minor() << ", completed: " << e.completed() << ")");
		// keep current joint states and skip reading new state
		return;
	} catch (const CORBA::OBJECT_NOT_EXIST &e) {
		ROS_ERROR_STREAM("OBJECT_NOT_EXIST [name: " << e._name() << ", repository ID: " << e._rep_id() << "]");
		ROS_ERROR_STREAM("OBJECT_NOT_EXIST: " << e.NP_minorString() << " (minor code: " << e.minor() << ", completed: " << e.completed() << ")");
		// keep current joint states and skip reading new state
		return;
	}

	// Copy joint value to ros_control HWIF
	constexpr long STATUS_BIT = 0b100;
	bool servo_enabled = true;
	for( int i = 0; i < num_joints; i++ )
	{
		joint_position_state_last[i] = joint_position_state[i];
		joint_position_state[i] = jointAngles[i] * DEG2RAD;
		if ((status[i] & STATUS_BIT) == 0)
			servo_enabled = false;
		joint_velocity_state[i] = joint_velocity_state[i] * (1.0 - velocity_alpha) + (joint_position_state[i] - joint_position_state_last[i]) / period.toSec() * velocity_alpha;
	}

	soft_estop = *estop_buffer.readFromRT();
	// Check if soft estop has been released or the estop has been enabled
	if(!soft_estop && last_estop)
	{
		disable_time = ros::Time::now().toSec();
	}
	if(servo_enabled && !last_servo_enabled)
	{
		disable_time = ros::Time::now().toSec();
	}
	last_estop = soft_estop;
	last_servo_enabled = servo_enabled;

	// Set command to actual when the servo is off
	// to prevent jumps when the servo is switched on.
	if(!servo_enabled || soft_estop)
	{
		setCommandToActual();
	}

	if(timer > 250)
	{
		timer = 0;
		// Update diagnostics
		pub_soft_estop->msg_.data = soft_estop;
		pub_soft_estop->unlockAndPublish();

		if (fake_estop)
		{
			robot_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
			robot_status.message = "NextageA soft e-stopped.";
		}
		else if (!fake_estop & !soft_reset)
		{
			robot_status.level = diagnostic_msgs::DiagnosticStatus::WARN;
			robot_status.message = "NextageA awaiting reset.";
		}
		else
		{
			robot_status.level = diagnostic_msgs::DiagnosticStatus::OK;
			robot_status.message = "Everything seem to be ok.";
		}

		// Publish diagnostics
		dia_soft_estop.value = (soft_estop ? "true" : "false" );
		dia_soft_reset.value = (soft_reset ? "true" : "false" );
		robot_status.values.push_back(dia_soft_estop);
		robot_status.values.push_back(dia_soft_reset);
		dia_array.status.push_back(robot_status);
		pub_diagnostics->msg_ = dia_array;
		pub_diagnostics->unlockAndPublish();
		robot_status.values.clear();
		dia_array.status.clear();

		readDigitalInput();
	} timer++;
}

// Write Joint Angle Command to Robot
void NextageAHardwareInterface::write(
	const ros::Time& time,
	const ros::Duration& period)
{
	// Limit joint angle and angular velocity
	position_joint_saturation_interface.enforceLimits(period);

	const double now = time.toSec();
	double t;
	if (soft_estop)
	{
		t = 0.0;
	}
	else if(now - disable_time < transition_time)
	{
		// Interpolate from last position when stopped to the new command over the period the transition_time
		t = (now - disable_time) / transition_time;
		t = 0.5 - cos(t * M_PI) * 0.5; // Use cosine profile for smoother transition
	}
	else
	{
		t = 1.0;
	}

	// Initialize command parameters
	statusPluginTypes::kindType kind = statusPluginTypes::K_MM_DEG;
	jointAnglePluginTypes::Nums jointNums;
	statusPluginTypes::JointValues jointAngles;
	commonStatus::statSequence_var Status = new commonStatus::statSequence;
	CORBA::Double speed = 1000;
	jointNums.length(num_joints);
	jointAngles.length(num_joints);

	// Copy angle values to double list
	double angles[num_joints];
	for( int i = 0; i < num_joints; i++ )
	{
		// Interpolate between last stopped command and new command
		// New command will be passed through after the transition ended
		jointAngles[i] = (joint_position_state_stopped[i] * (1.0 - t) + joint_position_command[i] * t) * RAD2DEG;
		jointNums[i] = i;
	}

	// Override segment duration for better trajectory overlap
	try {
		// Write command to robot
		Joint->setJointAngles(kind, jointNums, jointAngles, 0.0, speed, Status);
	} catch (const CORBA::TRANSIENT &e) {
		ROS_ERROR_STREAM("TRANSIENT [name: " << e._name() << ", repository ID: " << e._rep_id() << "]");
		ROS_ERROR_STREAM("TRANSIENT: " << e.NP_minorString() << " (minor code: " << e.minor() << ", completed: " << e.completed() << ")");
	} catch (const CORBA::COMM_FAILURE &e) {
		ROS_ERROR_STREAM("COMM_FAILURE [name: " << e._name() << ", repository ID: " << e._rep_id() << "]");
		ROS_ERROR_STREAM("COMM_FAILURE: " << e.NP_minorString() << " (minor code: " << e.minor() << ", completed: " << e.completed() << ")");
	}

	// Send feedback
	pub_command_feedback.msg_.header.stamp = time;
	pub_command_feedback.msg_.position.assign(angles, angles + num_joints);
	pub_command_feedback.unlockAndPublish();
}

void NextageAHardwareInterface::setCommandToActual()
{
	for( int i = 0; i < num_joints; i++ )
	{
		joint_position_command[i] = joint_position_state[i];
		joint_position_state_stopped[i] = joint_position_state[i];
		joint_velocity_state[i] = 0.0;
	}
}

/*o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o
    SERVICES & MESSAGES
  o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o*/

// Read digital input message
void NextageAHardwareInterface::readDigitalInput()
{
	// Get DIn information
	uint32_t din = Dio->getDin();
	uint32_t din_ext = Dio->getDinExt();

	// Read IO
	input_buffer.reset();
	for (int i=0; i<number_of_board; i++)
	{
		for(int j=0; j<32; j++)
		{
			if (i <= 0) {
				input_buffer[j + i*32] = (din >> j)&1;
			}
			else {
				input_buffer[j + i*32] = (din_ext >> j)&1;
			}
		}
	}

	// Publish
	for (auto& it : pub_din)
	{
		it.second->msg_.data = input_buffer[it.first-1];
		it.second->unlockAndPublish();

	}
}

// Write digital output service callback
bool NextageAHardwareInterface::writeDigitalOutput(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, unsigned int idx)
{
	if (*estop_buffer.readFromRT())
	{
		res.success = false;
		res.message = "The robot is e-stopped";
		return false;
	}

	// Convert to Octet Sequence
	CORBA::ULong doutBitLength = number_of_board * 32;
	CORBA::ULong doutByteLength = (doutBitLength / 8) + 1;

	if (idx > doutBitLength)
	{
		res.success = false;
		return false;
	}

	uint32_t cycle;
	if (req.data) {
		cycle = 0x00;
	}
	else {
		cycle = 0xFFFFFFFF;
	}
	CORBA::Boolean success = Dio->setDout((idx-1), cycle);
	if (success == false) {
		return false;
	}
	res.success = true;
	return true;
}

/*o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o
    OTHERS
  o~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~o*/

/* Switch servo to ON (true) or OFF (false) */
void NextageAHardwareInterface::switchServo(bool sw_servo)
{
	::CORBA::Boolean servo_command;
	if(sw_servo) {
		servo_command = true;
	}
	else {
		servo_command = false;
	}
	Servo->switchServo(servo_command);
}

}

PLUGINLIB_EXPORT_CLASS(nextage_hardware_interface::NextageAHardwareInterface,
	hardware_interface::RobotHW)
