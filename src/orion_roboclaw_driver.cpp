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
void read_pid(unsigned char address);
void set_pid_constants(unsigned char address, unsigned char cmd, int qpps, int p, int i , int d);
int tss_usb_open( const char *port );
int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes );
int read_data( const int fd, unsigned char *data, size_t num_bytes );
unsigned char mode_to_address(int mode);
void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryPtr &msg);
unsigned char get_address(string name);
unsigned char get_cmd(int channel);
int get_pps(double vel);

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
};

vector<mapping> address_map;
int fd;

//420 pulses per revolution for the current motors
void joint_trajectory_callback(const trajectory_msgs::JointTrajectoryPtr &msg)
{
	//Joint Trajectory Messages have a vector of String - names
	//and a vectory of JointTrajectoryPoint - points
	//These should have a 1-1 correspondance
	
	//The address (motor controller) to send the command to
	unsigned char address;
	//The command to send. 35 for driver M1 with signed speed, 36 for M2
	//speed is in quad pulses per second
	unsigned char cmd;
	unsigned char send_str[100];

	//the four byte speed
	unsigned char speed[4];
	for(int i = 0; i < msg->joint_names.size(); i++)
	{		
		//Match the name to a controller - which motor controller to drive
		address = get_address(msg->joint_names[i]);

		//Select cmd - which channel to drive
		cmd = get_cmd(address_map[i].channel);

		//Build the velocity command - how fast (rad/s)
		double vel = msg->points[0].velocities[i];
		int pulses_per_second = get_pps(vel);
		int checksum = 0;

		send_str[0] = address;
		checksum += address;
		ROS_INFO("byte[0]: %d\n",send_str[0]);

		send_str[1] = cmd;
		checksum += cmd;
		ROS_INFO("byte[1]: %d\n",send_str[1]);
		
		//Set the acceleration to lots
		long acceleration = 0xFFFFFFFF; //pulses/s/s probably
		
		/*send_str[2] = (acceleration >> 24) & 0xFF;
		checksum += send_str[2];
		ROS_INFO("byte[2]: %d\n",send_str[2]);

		send_str[3] = (acceleration >> 16) & 0xFF;
		checksum += send_str[3];
		ROS_INFO("byte[3]: %d\n",send_str[3]);

		send_str[4] = (acceleration >> 8) & 0xFF;
		checksum += send_str[4];
		ROS_INFO("byte[4]: %d\n",send_str[4]);

		send_str[5] = acceleration & 0xFF;
		checksum += send_str[5];
		ROS_INFO("byte[5]: %d\n",send_str[5]);*/

		send_str[2] = (pulses_per_second >> 24) & 0xFF;
		checksum += send_str[2];
		ROS_INFO("byte[2]: %d\n",send_str[2]);
		
		send_str[3] = (pulses_per_second >> 16) & 0xFF;
		checksum += send_str[3];
		ROS_INFO("byte[3]: %d\n",send_str[3]);
		
		send_str[4] = (pulses_per_second >> 8) & 0xFF;
		checksum += send_str[4];
		ROS_INFO("byte[4]: %d\n",send_str[4]);
	
		send_str[5] = pulses_per_second & 0xFF;
		checksum += send_str[5];
		ROS_INFO("byte[5]: %d\n",send_str[5]);

		send_str[6] = checksum & 0x7F;
		//set confirm packet bit
		send_str[6] |= 0x80;
		ROS_INFO("byte[6]: %d\n\n\n",send_str[6]);

		send_cmd(fd,send_str,7);
		tcflush(fd, TCIOFLUSH); 
	}
}

void read_pid(unsigned char address)
{
	unsigned char send_str[100];
	send_str[0] = address;
	send_str[1] = 55;
	send_cmd(fd,send_str,2);
	//read_data(fd,send_str,17);
	//ROS_INFO("Read PID: %d, %d, %d, %d\n",*((int*)send_str), *((int*)send_str+4),*((int*)send_str+8),*((int*)send_str+12));
}

void set_pid_constants(unsigned char address, unsigned char cmd, int qpps, int p, int i , int d)
{
	unsigned char send_str[100];
	int checksum = 0;

	send_str[0] = address;
	checksum += send_str[0];
	send_str[1] = cmd;
	checksum += send_str[1];
	
	send_str[2] = (qpps >> 24) & 0xFF;
	checksum += send_str[2];
	send_str[3] = (qpps >> 16) & 0xFF;
	checksum += send_str[3];
	send_str[4] = (qpps >> 8 ) & 0xFF;
	checksum += send_str[4];
	send_str[5] = qpps & 0xFF;
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
	
	send_str[14] = (d >> 24) & 0xFF;
	checksum += send_str[14];
	send_str[15] = (d >> 16) & 0xFF;
	checksum += send_str[15];
	send_str[16] = (d >> 8 ) & 0xFF;
	checksum += send_str[16];
	send_str[17] = d & 0xFF;
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
	return (vel*420*4)/(2*M_PI);
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

unsigned char get_cmd(int channel)
{
	if(channel == 1)
		return 35;
	else if(channel == 2)
		return 36;
	ROS_ERROR("orion_roboclaw_driver:\n\tget_cmd: Invalid channel:%d\n",channel);
	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "orion_roboclaw_driver_node");
	ros::NodeHandle n;
	ros::NodeHandle nh_priv( "~" );

	//set up a subscriber to listen for joint_trajectory messages
	ros::Subscriber joint_trajectory_sub = n.subscribe("cmd_joint_traj", 1, joint_trajectory_callback);

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
	fd = tss_usb_open("/dev/ttySAC0");
	if(fd < 0)
	{
		ROS_ERROR("orion_roboclaw_driver: Failed to open port\n");
		return 0;
	}
	set_pid_constants(128,28,44000,0xF0000,0xF0000,0x40000);
	set_pid_constants(128,29,44000,0xF0000,0xF0000,0x40000);
	set_pid_constants(129,28,44000,0xF0000,0xF0000,0x40000);
	set_pid_constants(129,29,44000,0xF0000,0xF0000,0x40000);
	//read_pid(128);
	unsigned char send_str[1000];
	send_str[0] = 129;
	send_str[1] = 36;
	send_str[2] = 0;
	send_str[3] = 0;
	send_str[4] = 0;
	send_str[5] = 0;
	send_str[6] = (129 + 36)&0x7F;
	send_cmd(fd,send_str,7);
	send_str[0] = 129;
	send_str[1] = 35;
	send_str[2] = 0;
	send_str[3] = 0;
	send_str[4] = 0;
	send_str[5] = 0;
	send_str[6] = (129 + 35)&0x7F;
	send_cmd(fd,send_str,7);
	send_str[0] = 128;
	send_str[1] = 35;
	send_str[2] = 0;
	send_str[3] = 0;
	send_str[4] = 0;
	send_str[5] = 0;
	send_str[6] = (128 + 35)&0x7F;
	send_cmd(fd,send_str,7);
	send_str[0] = 128;
	send_str[1] = 36;
	send_str[2] = 0;
	send_str[3] = 0;
	send_str[4] = 0;
	send_str[5] = 0;
	send_str[6] = (128 + 36)&0x7F;
	send_cmd(fd,send_str,7);
	//read_data(fd,send_str,5);
	//ROS_INFO("%s\n",send_str);
	
	while (ros::ok())
	{
		ros::spinOnce();
		read_data(fd,send_str,1);
	}
	return 0;
}


int send_cmd( const int fd, const unsigned char *data, const size_t num_bytes )
{
	int bytes_sent = write( fd, data, num_bytes );
	ROS_INFO("Sent %d bytes\n",bytes_sent);
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
	int bytes_recv;
	while( num_bytes )
	{
		bytes_recv = read( fd, data, num_bytes );
		if( bytes_recv < 0 )
			return -1;
		if( bytes_recv == 0 )
			return -1;
		num_bytes -= bytes_recv;
		for(int i = 0; i < bytes_recv; i++)
		{
			ROS_INFO("Received byte: %d",data[i]);
		}
	}
	return -1;
}

int tss_usb_open( const char *port )
{
	/* Step 1: Make sure the device opens OK */
	int fd = open( port, O_RDWR | O_NOCTTY | O_NDELAY );
	if( fd < 0 )
	{
		ROS_INFO("Error 1\n");
		return -1;
	}

	if(-1 == fcntl( fd, F_SETFL, 0 ))
		ROS_INFO("fcntl failed with error: %s\n",strerror(errno));

	struct termios options;
	cfmakeraw( &options );
	if( cfsetispeed( &options, B38400 ) < 0 )
	{
		close( fd );
		ROS_INFO("Error 2\n");
		return -1;
	}
	if( cfsetospeed( &options, B38400 ) < 0 )
	{
		close( fd );
		ROS_INFO("Error 3\n");
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
		ROS_INFO("Error 4: %d %d\n",ret_val,errno);
		close( fd );
		return -1;
	}
	ROS_INFO("FD: %d\n",fd);
	return fd;
}
