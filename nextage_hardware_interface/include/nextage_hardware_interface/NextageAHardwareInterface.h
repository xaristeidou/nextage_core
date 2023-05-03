/*__INCLUDE___________________________________________________________________*/
// From ros
#include <ros/ros.h>
// From ros_control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <pluginlib/class_list_macros.hpp>
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
// Nextage API
#include "statusPlugin.h"
#include "jointAnglePlugin.h"
#include "servoPlugin.h"
#include "dioPlugin.h"
// Messages and Servers
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/JointState.h>
// Common
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <typeinfo>
#include <math.h>
#include <unistd.h>
#include <boost/dynamic_bitset.hpp>
// Diagnostics
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>


/*__DEFINE____________________________________________________________________*/
// For display
#define COL_RESET       "\033[0m"
#define COL_RETURN      "\033[1A"
#define COL_RED         "\033[31m"
#define COL_GREEN       "\033[32m"
#define COL_YELLOW      "\033[33m"
#define COL_BLUE        "\033[34m"
// For Unit conversion
#define DEG2RAD         M_PI/180
#define RAD2DEG         180/M_PI
#define U_MICROSECOND   1
#define U_MILLISECOND   1000
#define U_SECOND        1000000
// For CORBA Init
#define CORBA_BUFSIZE   256
#define CORBA_ARGNUM    4
#define CORBA_OPT1      "-ORBInitRef"
#define CORBA_ARG1      "NameService=corbaloc:iiop:nextage:2809/NameService"
#define CORBA_OPT2      "-ORBgiopMaxMsgSize"
#define CORBA_ARG2      "104857600"

namespace nextage_hardware_interface
{

/*__CLASS_____________________________________________________________________*/
class NextageAHardwareInterface : public hardware_interface::RobotHW
{
public:
    // Constructor & Destructor
    NextageAHardwareInterface();
    ~NextageAHardwareInterface();

    // Init Functions
    bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh);
    void initCORBA();
    bool initAPI();
    void initHWInterface();
    void initHWPublisher(ros::NodeHandle& pub_hw_nh);
    void initModel(ros::NodeHandle& robot_hw_nh);

	// Corba API
    CORBA::Object_ptr getObject(const std::string& objname);

    // Standard ros_control interface functions
    void read(const ros::Time& time, const ros::Duration& period);
    void write(const ros::Time& time, const ros::Duration& period);

    // Additional functions
    void setCommandToActual();
    void switchServo(bool sw_servo);
    void readDigitalInput();
    bool writeDigitalOutput(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res, unsigned int idx);

protected:
    bool estopCallback(std_srvs::SetBool::Request &req); //const std_msgs::Bool::ConstPtr& msg
    bool resetCallback(void); //const std_msgs::Bool::ConstPtr& msg

    int timer;

    // ROS Node
    ros::NodeHandle nh_;
    bool initialised = false;
    ros::Subscriber estop_sub;

    // Publishers
    std::map<unsigned int, std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>>> pub_dout;
    std::map<unsigned int, std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>>> pub_din;
    std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::Bool>> pub_soft_estop;
    std::shared_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::DiagnosticArray>> pub_diagnostics;
    realtime_tools::RealtimePublisher<sensor_msgs::JointState> pub_command_feedback;
    std::map<unsigned int, ros::ServiceServer> svc_dout;
    ros::ServiceServer svc_servo, svc_soft_estop, svc_soft_reset;
    // CORBA
    ::CORBA::ORB_var orb;
    ::CORBA::Object_var obj;
	CosNaming::NamingContext_var rootContext;
    // Nextage API
	jointAnglePlugin_var Joint;
	statusPlugin_var Status;
    servoPlugin_var Servo;
	dioPlugin_var Dio;

    // ros_control
    hardware_interface::JointStateInterface joint_state_interface;
    hardware_interface::PositionJointInterface position_joint_interface;
    int num_joints;
    std::vector<std::string> joint_name;
    std::vector<double> joint_position_state;
    std::vector<double> joint_position_state_stopped;
    std::vector<double> joint_velocity_state;
    std::vector<double> joint_effort_state;
    std::vector<double> joint_position_command;
    std::vector<double> joint_position_state_last;
    double velocity_alpha;
    realtime_tools::RealtimeBuffer<bool> estop_buffer;
    bool soft_estop, soft_reset, fake_estop;
    bool last_estop;
    bool last_servo_enabled;
    double transition_time;
    double disable_time;

    // IO
    unsigned int number_of_board;
    boost::dynamic_bitset<> input_buffer;

    // joint limit
    joint_limits_interface::JointLimits joint_limits;
    joint_limits_interface::PositionJointSaturationInterface position_joint_saturation_interface;

    // Axes data
    std::vector<int> axes_num;
    std::vector<std::string> axes_tag;

    // For display
    double last_time = 0.0;

    // Diagnostics
    diagnostic_msgs::DiagnosticArray dia_array;
    diagnostic_msgs::DiagnosticStatus robot_status;
    diagnostic_msgs::KeyValue dia_soft_estop, dia_soft_reset;
};
}
