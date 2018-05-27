#include <iostream>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <termio.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/select.h>
#include <ros/ros.h>
#include <ros/console.h>

using namespace std;
using namespace ros;
ros::NodeHandle* nh;
int fd;

typedef struct robotGoal {
	uint32_t id;
	char message[20];
} RobotGoal;

typedef struct robotstate {
	uint32_t id;
	char message[80];
} RobotState;

RobotState robotState;
RobotGoal robotGoal;

void send_goal(RobotGoal* robotGoal) {
	// Encapsulates the data into a packet
	int wlen = write(fd, robotGoal, sizeof(RobotGoal));
	if (wlen != sizeof(RobotGoal)) {
		printf("Error from write: %d, %d\n", wlen, errno);
	}
	tcflush(fd, TCOFLUSH);
}

int open_port(void) {
	int fd; // file description for the serial port

	fd = open("/dev/ttyACM0",
			O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);

	if (fd == -1)
		fd = open("/dev/ttyACM1", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1)
		fd = open("/dev/ttyACM2", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
	if (fd == -1) {
		cout << "open_port: Unable to open /dev/ttyACM0. \n" << endl;
		exit(1);
	}

	fcntl(fd, F_SETFL, 0);

	tcflush(fd, TCIOFLUSH);
	cout << "Opened Port" << endl;

	return (fd);
} //open_port

int configure_port(int fd) {
	struct termios port_settings;     // structure to store the port settings in

	cfsetispeed(&port_settings, B2000000);    // set baud rates
	cfsetospeed(&port_settings, B2000000);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~PARODD;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;

	port_settings.c_cflag |= CLOCAL | CREAD;

	port_settings.c_iflag &= ~IGNBRK;
	port_settings.c_iflag &= ~(IXON | IXOFF | IXANY);
	port_settings.c_iflag &= ~IGNCR;
	port_settings.c_iflag &= ~ICRNL;
	port_settings.c_iflag &= ~INLCR;
	port_settings.c_iflag &= ~INPCK;
	port_settings.c_iflag &= ~ISTRIP;

	port_settings.c_lflag = 0;
	port_settings.c_oflag = 0;

	port_settings.c_cc[VTIME]=1;
	port_settings.c_cc[VMIN]=1;

	port_settings.c_lflag &= ~(ECHONL|NOFLSH);

	int mcs = 0;
    ioctl(fd, TIOCMGET, &mcs);
    mcs |= TIOCM_RTS;
	ioctl(fd, TIOCMSET, &mcs);

	port_settings.c_cflag &= ~CRTSCTS;

	port_settings.c_cflag |= CS8;

	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port

	cout << "Configured Port" << endl;
	return (fd);

}

void send_to_hardware(ros::Publisher hardware_publisher) {
	memset(robotState.message, 0, strlen(robotState.message));
	// Send Goal
	// while (strcmp(robotState.message, "ACK") || robotGoal.id != robotState.id) {
	// 	tcflush(fd, TCIOFLUSH);
	// 	// send_goal(robotGoalPtr);
	// 	cout << "Sending Message " << robotGoal.id << endl;

	// 	usleep(200000);
	// 	// robotState = receive_state();
	// 	cout << "Receiving Message " << robotState.id << endl;

	// 	if(strcmp(robotState.message, "Done")){
	// 		successcall.message = MoveType::success;
	// 		hardware_publisher.publish(successcall);
	// 		break;
	// 	};
	// }

	cout << robotState.id << " " << robotState.message << endl;
	robotGoal.id++;
}

int max_buf_size = 128;

void receive_loop() {

	int totalSentPackets = 0;
	int totalFailedPackets = 0;

	while(1) {

		unsigned char robotStateData[sizeof(RobotState)];
		unsigned char *robotStateDataPtr = robotStateData;
		int totalBytesRead = 0;

		int startseqcount = 0;

		unsigned char m_buf[max_buf_size];

		while (1) {

			int bytesRead = read(fd, m_buf, max_buf_size);

			if(bytesRead < 0) continue;

			for (int x = 0; x < bytesRead; ++x) {
				// Start reading
				if(startseqcount == 4) {
					*robotStateDataPtr = m_buf[x];
					robotStateDataPtr++;
					totalBytesRead++;
					if(totalBytesRead == sizeof(RobotState))
						break;
				}
				else {
					ROS_INFO(".");
					if(m_buf[x] == 255)
						startseqcount++;
					else
						startseqcount = 0;
				}
			}

			if(totalBytesRead == sizeof(RobotState))
				break;
		}

		memset(&robotState, 0, sizeof(RobotState));
		memcpy(&robotState, robotStateData, sizeof(RobotState));

		ROS_INFO("DATA");
		for (int i = 0; i < sizeof(RobotState); ++i) {
			ROS_INFO("%d", robotStateData[i]);
		}

		// Checking the correctness
		bool dataCorrect = true;
		for(int i = 0; i < 80; ++i) {
			// ROS_INFO("%d", robotState.message[i]);
			if((int) robotState.message[i] != i) {
				ROS_INFO("Not matching %d %d", robotState.message[i], i);
				dataCorrect = false;
			}
		}

		totalSentPackets++;
		if(!dataCorrect)
			totalFailedPackets++;

		ROS_INFO("%d/%d Failed", totalFailedPackets, totalSentPackets);
	}
}

// void receive_loop2() {
// 	int blen;
// 	int buf_offset = 0;
// 	char buf[128];

// 	while(1) {
// 		int x = check_io(fd, 0, 1000, buf + buf_offset, sizeof(buf) - buf_offset, &blen);
// 		blen += buf_offset;
// 	}
// }


int main(int argc, char **argv) {

	//establishConnection();
	ros::init(argc, argv, "soccer_hardware");
	ros::NodeHandle n;
	nh = &n;

	ros::Rate r(10); // 10 Hz

	fd = open_port();
	configure_port(fd);
	tcflush(fd, TCIOFLUSH);

	receive_loop();

	ros::spin();
}
