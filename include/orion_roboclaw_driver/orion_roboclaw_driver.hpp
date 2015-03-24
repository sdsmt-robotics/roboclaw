#ifndef _ORION_ROBOCLAW_DRIVER_HPP_
#define _ORION_ROBOCLAW_DRIVER_HPP_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sstream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <math.h>
#include <cstdlib>
using namespace std;

//Channel information needed in the joint_trajectory_callback
struct mapping
{
	//Corresponds to the Mode in the roboclaw datasheet
	//valid range 7-14 for packet serial
	int mode;
	//The user-readable name. i.e. left front wheel
	string label;
	//the motor controller channel, 1 or 2
	//i.e. left or right for a wheeled robot
	int channel;
	double vel;
};

//Drive Commands - same as documentation
enum roboclaw_commands
{
	//Standard Commands
	CMD_DRIVE_M1_FORWARD = 0,
	CMD_DRIVE_M1_REVERSE = 1,
	CMD_SET_MIN_VOLTAGE = 2,
	CMD_SET_MAX_VOLTAGE = 3,
	CMD_DRIVE_M2_FORWARD = 4,
	CMD_DRIVE_M2_REVESE = 5,
	CMD_DRIVE_M1 = 6,
	CMD_DRIVE_M2 = 7,

	//Mix Mode Commands
	CMD_DRIVE_FORWARD = 8,
	CMD_DRIVE_REVERSE = 9,
	CMD_TURN_RIGHT = 10,
	CMD_TURN_LEFT = 11,
	CMD_DRIVE_FORWARD_BACKWARD = 12,
	CMD_TURN_LEFT_RIGHT = 13,

	//Advanced Packet Serial
	CMD_READ_FIRMWARE_VERSION = 21,
	CMD_READ_MAIN_BATTERY_VOLTAGE = 24,
	CMD_READ_LOGIC_BATTERY_VOLTAGE = 25,
	CMD_SET_MIN_LOGIC_VOLTAGE = 26,
	CMD_SET_MAX_LOGIC_VOLTAGE = 27,
	CMD_READ_MOTOR_CURRENTS = 49,
	CMD_READ_M1_PIDQ = 55,
	CMD_READ_M2_PIDQ = 56,
	CMD_SET_MAIN_BATTERY_VOLTAGES = 57,
	CMD_SET_LOGIC_BATTERY_VOLTAGES = 58,
	CMD_READ_MAIN_BATTERY_VOLTAGES = 59,
	CMD_READ_LOGIC_BATTERY_VOLTAGES = 60,
	CMD_READ_TEMPERATURE = 82,
	CMD_READ_ERROR_STATUS = 90,
	CMD_WRITE_SETTINGS_TO_EEPROM = 94,

	//Reading Quadrature Encoders
	CMD_READ_M1_ENCODER = 16,
	CMD_READ_M2_ENCODER = 17,
	CMD_READ_M1_QPPS = 18,
	CMD_READ_M2_QPPS = 19,
	CMD_RESET_M1_M2_ENCODERS = 20,

	//Advanced Motor Control
	CMD_SET_PID_M1 = 28,
	CMD_SET_PID_M2 = 29,
	CMD_READ_M1_SPEED = 30,
	CMD_READ_M2_SPEED = 31,
	CMD_DRIVE_M1_SIGNED_DUTY_CYCLE = 32,
	CMD_DRIVE_M2_SIGNED_DUTY_CYCLE = 33,
	CMD_DRIVE_M1_M2_SIGNED_DUTY_CYCLE = 34,
	CMD_DRIVE_M1_SIGNED_SPEED = 35,
	CMD_DRIVE_M2_SIGNED_SPEED = 36,
	CMD_DRIVE_M1_M2_SIGNED_SPEED = 37,
	CMD_DRIVE_M1_SIGNED_SPEED_ACCEL = 38,
	CMD_DRIVE_M2_SIGNED_SPEED_ACCEL = 39,
	CMD_DRIVE_M1_M2_SIGNED_SPEED_ACCEL = 40,
	CMD_DRIVE_M1_SIGNED_SPEED_DISTANCE = 41,
	CMD_DRIVE_M2_SIGNED_SPEED_DISTANCE = 42,
	CMD_DRIVE_M1_M2_SIGNED_SPEED_DISTANCE = 43,
	CMD_DRIVE_M1_SIGNED_SPEED_ACCEL_DISTANCE = 44,
	CMD_DRIVE_M2_SIGNED_SPEED_ACCEL_DISTANCE = 45,
	CMD_DRIVE_M1_M2_SIGNED_SPEED_ACCEL_DISTANCE = 46,
	CMD_READ_BUFFER_LENGTH = 47,
	CMD_DRIVE_M1_M2_INDIVIDUAL_SIGNED_SPEED_ACCEL = 50,
	CMD_DRIVE_M1_M2_INDIVIDUAL_SIGNED_SPEED_ACCEL_DISTANCE = 51,
	CMD_DRIVE_M1_DUTY_CYCLE_ACCEL = 52,
	CMD_DRIVE_M2_DUTY_CYCLE_ACCEL = 53,
	CMD_DRIVE_M1_M2_DUTY_CYCLE_ACCEL = 54
};


//Function Definitions
void drive_motor(int fd, struct mapping m, double velocity);
void drive_motor_with_acceleration(int fd, struct mapping m, double velocity, double acceleration);
void read_pid(unsigned char address);
void set_pid_constants(unsigned char address, unsigned char cmd, int qpps, int p, int i , int d);
int open_serial_port( const char *port );
int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes );
int read_data( const int fd, unsigned char *data, size_t num_bytes );
unsigned char mode_to_address(int mode);
void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryPtr &msg);
unsigned char get_address(string name);
unsigned char get_cmd(int cmd_type,int channel);
int get_pps(double vel);
int get_mapping(string name);
void read_firmware_version(int fd);

class XmlRpcValueAccess : private XmlRpc::XmlRpcValue
{
	public:
	XmlRpcValueAccess( XmlRpc::XmlRpcValue xml_rpc_value ) :
		XmlRpc::XmlRpcValue( xml_rpc_value ) { }
		XmlRpc::XmlRpcValue::ValueStruct getValueStruct( )
	{
		assertStruct( );
		return *_value.asStruct;
	}
		XmlRpc::XmlRpcValue::ValueArray getValueArray( )
	{
	assertArray( size( ) );
		return *_value.asArray;
	}
};

#endif
