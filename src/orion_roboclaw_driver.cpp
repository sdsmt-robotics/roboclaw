#define CMD_DRIVE 0
#define CMD_DRIVE_W_ACCEL 1
#define CMD_DRIVE_PWM 2

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "trajectory_msgs/JointTrajectory.h"
#include <sstream>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>
#include <math.h>

using namespace std;

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

//Channel information needed in the joint_trajectory_callback
struct mapping
{
	//Corresponds to the Mode in the roboclaw datasheet
	//valid range 7-14
	int mode;
	//The user-readable name. i.e. left front wheel
	string label;
	//the motor controller channel, 1 or 2
	//i.e. left or right for a wheeled robot
	int channel;
	double vel;
};

vector<mapping> address_map;
int fd;

//420 pulses per revolution for the current motors
void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
	//Joint Trajectory Messages have a vector of String - names
	//and a vectory of JointTrajectoryPoint - points
	//These should have a 1-1 correspondance
	struct mapping m;
	for(int i = 0; i < msg->joint_names.size(); i++)
	{		
		//Match the joint name to one of our mappings
		int index = get_mapping(msg->joint_names[i]);
		if(-1 == index)
		{
			ROS_WARN("Unknown joint_name: %s",msg->joint_names[i].c_str());	
			continue;
		}
		//set motor speeds for update in the main loop
		address_map[index].vel = msg->points[0].velocities[i];
	}
}

int get_mapping(string name)
{
	for(int i = 0; i < address_map.size(); i++)
		if(name == address_map[i].label)
			return i;
	return -1;
}

void drive_motor_with_acceleration(int fd, struct mapping m, double velocity, double acceleration)
{	
	
	unsigned char send_str[11];
	int pps_speed = get_pps(velocity);
	int pps_accel = get_pps(acceleration);
	
	int checksum = 0;

	send_str[0] = get_address(m.label);
	checksum +=   get_address(m.label);

	send_str[1] = get_cmd(CMD_DRIVE_W_ACCEL,m.channel);
	checksum +=   get_cmd(CMD_DRIVE_W_ACCEL,m.channel);
	
	send_str[2] = (pps_accel>>24) & 0xFF;
	checksum +=   (pps_accel>>24) & 0xFF;
	
	send_str[3] = (pps_accel>>16) & 0xFF;
	checksum +=   (pps_accel>>16) & 0xFF;

	send_str[4] = (pps_accel>>8) & 0xFF;
	checksum +=   (pps_accel>>8) & 0xFF;

	send_str[5] = pps_accel & 0xFF;
	checksum +=   pps_accel & 0xFF;
	
	send_str[6] = (pps_speed>>24) & 0xFF;
	checksum +=   (pps_speed>>24) & 0xFF;
	
	send_str[7] = (pps_speed>>16) & 0xFF;
	checksum +=   (pps_speed>>16) & 0xFF;

	send_str[8] = (pps_speed>>8) & 0xFF;
	checksum +=   (pps_speed>>8) & 0xFF;

	send_str[9] = pps_speed & 0xFF;
	checksum +=   pps_speed & 0xFF;

	send_str[10] = checksum & 0x7F;
	send_cmd(fd,send_str,11);
	tcflush(fd, TCIOFLUSH); 
}

void drive_motor(int fd, struct mapping m, double velocity)
{	
	
	unsigned char send_str[7];
	int pps = get_pps(velocity);
	int checksum = 0;

	send_str[0] = get_address(m.label);
	checksum +=   get_address(m.label);

	send_str[1] = get_cmd(CMD_DRIVE,m.channel);
	checksum +=   get_cmd(CMD_DRIVE,m.channel);
	
	send_str[2] = (pps>>24) & 0xFF;
	checksum +=   (pps>>24) & 0xFF;
	
	send_str[3] = (pps>>16) & 0xFF;
	checksum +=   (pps>>16) & 0xFF;

	send_str[4] = (pps>>8) & 0xFF;
	checksum +=   (pps>>8) & 0xFF;

	send_str[5] = pps & 0xFF;
	checksum +=   pps & 0xFF;

	send_str[6] = checksum & 0x7F;
	send_cmd(fd,send_str,7);
	tcflush(fd, TCIOFLUSH); 
}

void read_pid(unsigned char address)
{
	unsigned char send_str[100];
	send_str[0] = address;
	send_str[1] = 55;
	send_cmd(fd,send_str,2);
	usleep(1000);
	read_data(fd,send_str,17);
	ROS_INFO("Read PID: %d, %d, %d, %d\n",*((int*)send_str), *((int*)send_str+4),*((int*)send_str+8),*((int*)send_str+12));
}

void set_pid_constants(unsigned char address, unsigned char cmd, int qpps, int p, int i , int d)
{
	unsigned char send_str[19];
	int checksum = 0;

	send_str[0] = address;
	checksum += send_str[0];
	send_str[1] = cmd;
	checksum += send_str[1];
	
	send_str[2] = (d >> 24) & 0xFF;
	checksum += send_str[2];
	send_str[3] = (d >> 16) & 0xFF;
	checksum += send_str[3];
	send_str[4] = (d >> 8 ) & 0xFF;
	checksum += send_str[4];
	send_str[5] = d & 0xFF;
	checksum += send_str[5];
	
	send_str[6] = (p >> 24) & 0xFF;
	checksum += send_str[6];
	send_str[7] = (p >> 16) & 0xFF;
	checksum += send_str[7];
	send_str[8] = (p >> 8 ) & 0xFF;
	checksum += send_str[8];
	send_str[9] = p & 0xFF;
	checksum += send_str[9];
	
	send_str[10] = (i >> 24) & 0xFF;
	checksum += send_str[10];
	send_str[11] = (i >> 16) & 0xFF;
	checksum += send_str[11];
	send_str[12] = (i >> 8 ) & 0xFF;
	checksum += send_str[12];
	send_str[13] = i & 0xFF;
	checksum += send_str[13];
	
	send_str[14] = (qpps >> 24) & 0xFF;
	checksum += send_str[14];
	send_str[15] = (qpps >> 16) & 0xFF;
	checksum += send_str[15];
	send_str[16] = (qpps >> 8 ) & 0xFF;
	checksum += send_str[16];
	send_str[17] = qpps & 0xFF;
	checksum += send_str[17];

	send_str[18] = (checksum&0x7F);
	send_cmd(fd,send_str,19);
	tcflush(fd, TCIOFLUSH);
}

int get_pps(double vel)
{
	//vel is in rad/s
	//We know we have 420 ticks per rev*4 pulses per tick
	//2*Pi rads in a rev
	int pps = (vel*420*4)/(2*M_PI);
	ROS_INFO("PPS: %d",pps);
	return pps;
}

unsigned char get_address(string name)
{
	//First, find the label that matches the name of the joint
	//in the message we received.
	//If we find it, convert it to an address and return it.
	for(int i = 0; i < address_map.size(); i++)
		if(name == address_map[i].label)
			return mode_to_address(address_map[i].mode);
	ROS_ERROR("orion_roboclaw_driver:\n\tget_address: Failed to match name to a label\n");
	return 0;
}

unsigned char mode_to_address(int mode)
{
	if(mode < 7 || mode > 14)
		ROS_ERROR("orion_roboclaw_driver: \n\tmode_to_address: Invalid mode: %d\n",mode);
	return 0x80 + (mode-7);
}

unsigned char get_cmd(int cmd_type,int channel)
{
	int cmd;
	if(channel == 1)
		cmd = 0;
	else if(channel == 2)
		cmd = 1;
	else
		ROS_ERROR("orion_roboclaw_driver:\n\tget_cmd: Invalid channel:%d\n",channel);

	switch(cmd_type)
	{
		case CMD_DRIVE:
			return cmd + 35;
		break;
		case CMD_DRIVE_W_ACCEL:
			return cmd + 38;
		break;
		//case CMD_DRIVE_PWM:
	}
	return 0;
}

int main(int argc, char **argv)
{
	char port_name[100] = "/dev/ttySAC0";
	ros::init(argc, argv, "orion_roboclaw_driver_node");
	ros::NodeHandle n;
	ros::NodeHandle nh_priv( "~" );
	
	//Publish commands at 50Hz
	ros::Rate loop_rate(10);

	//set up a subscriber to listen for joint_trajectory messages
	ros::Subscriber joint_trajectory_sub = n.subscribe("cmd_joint_traj", 1, joint_trajectory_callback);

	//check for non-detault port name
	if(nh_priv.hasParam("port"))
	{

	}
	//Check for non-default channel mode
	if(nh_priv.hasParam("channel_mapping"))
	{
			
	}	
	else
	{
		mapping m;
		
		//Front left wheel
		m.label = "front_left";
		m.mode = 7;
		m.channel = 1;
		address_map.push_back(m);

		//Front right wheel
		m.label = "front_right";
		m.mode = 7;
		m.channel = 2;
		address_map.push_back(m);

		//back left wheel
		m.label = "back_left";
		m.mode = 8;
		m.channel = 1;
		address_map.push_back(m);

		//back right wheel
		m.label = "back_right";
		m.mode = 8;
		m.channel = 2;
		address_map.push_back(m);
	}
	
	int count = 0;
	fd = open_serial_port(port_name);
	if(fd < 0)
		return 0;
	//stop all the motors
	for( int i = 0; i < address_map.size(); i++)
		drive_motor(fd,address_map[i],0);

	//this needs to be replaced with params, but these defaults 
	//work for mecanum	
	set_pid_constants(129,29,2400,0x100,0x10,0x0);
	usleep(10000);
	set_pid_constants(128,28,2400,0x100,0x10,0x0);
	usleep(10000);
	set_pid_constants(128,29,2400,0x100,0x10,0x0);
	usleep(10000);
	set_pid_constants(129,28,2400,0x100,0x10,0x0);
	usleep(10000);
	
	while (ros::ok())
	{
		loop_rate.sleep();
		//Send current command
		for(int i = 0; i < address_map.size(); i++)
			drive_motor(fd,address_map[i],address_map[i].vel);
		ros::spinOnce();
	}
	//stop all the motors
	for( int i = 0; i < address_map.size(); i++)
		drive_motor(fd,address_map[i],0);
	return 0;
}

void read_firmware_version(int fd)
{
	unsigned char str[100];
	str[0] = 129;
	str[1] = 21;
	send_cmd(fd,str,2);
	//Wait 1ms for response
	usleep(1000);
	read_data(fd,str,32);
	ROS_INFO("Version: %s",str);
}

void publish_command()
{

}


int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes )
{
	int bytes_sent = write( fd, data, num_bytes );
	if( bytes_sent < 0 )
	{
		ROS_INFO("Send error 1\n");
		ROS_INFO("Send message: %s\n",strerror(errno));
		return -1;
	}
	else if( bytes_sent == 0 )
	{
		ROS_INFO("Send error 2\n");
		return -1;
	}
	else if( (unsigned)bytes_sent != num_bytes )
	{
		ROS_INFO("Send error 3\n");
		return -1;
	}
	return 0;
}

int read_data( const int fd, unsigned char *data, size_t num_bytes )
{
//	ROS_INFO("In read data");
	int bytes_recv;
	while( num_bytes )
	{
		bytes_recv = read( fd, data, num_bytes );
//		ROS_INFO("Received %d bytes",bytes_recv);
		if( bytes_recv < 0 )
			return -1;
		if( bytes_recv == 0 )
			return -1;
		num_bytes -= bytes_recv;
//		for(int i = 0; i < bytes_recv; i++)
//		{
//			ROS_INFO("Received byte: %d",data[i]);
//		}
	}
	return -1;
}

int open_serial_port( const char *port )
{
	/* Step 1: Make sure the device opens OK */
	int fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
	{
		ROS_ERROR("Failed to open port: %s",port);
		return -1;
	}

	if(-1 == fcntl( fd, F_SETFL, 0 ))
		ROS_INFO("fcntl failed with error: %s\n",strerror(errno));

	struct termios options;
	cfmakeraw( &options );
	if( cfsetispeed( &options, B38400 ) < 0 )
	{
		close( fd );
		ROS_ERROR("Failed to set input baud rate for port: %s.",port);
		return -1;
	}
	if( cfsetospeed( &options, B38400 ) < 0 )
	{
		close( fd );
		ROS_ERROR("Failed to set output baud rate for port: %s.",port);
		return -1;
	}
	options.c_cflag &= ~HUPCL;
	options.c_lflag &= ~ICANON;
	options.c_cc[VTIME] = 2;
	options.c_cc[VMIN] = 0;
	int ret_val = tcsetattr( fd, TCSANOW, &options );
	if( ret_val < 0 )
	{
		ROS_INFO("Failed with error %s\n",strerror(errno));
		close( fd );
		return -1;
	}
	ROS_INFO("FD: %d\n",fd);
	return fd;
}
