#include <stdio.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <math.h>


#include "generic_interface.hpp"
#include "multi_turn_angle_control_client.hpp"
#include "complex_motor_control_client.hpp"

int set_interface_attribs (int fd, int speed, int parity);
void set_blocking (int fd, int should_block);
void sendCommand(float time_cmd, float angle_cmd);

const char *portname = "/dev/ttyUSB1";

// Sets the angle to go to in radians
const float kAngle = 2.0f*M_PI;
// Sets the trajectory time in seconds
const float kTime = 1;

int msg_count = 0;
int fd = -1; // fd for serial port

// Make a communication interface object
GenericInterface com;
// Make a Multi-turn Angle Controller object with obj_id 0
MultiTurnAngleControlClient angle(0);
// make a complex motor control object
//ComplexMotorControlClient motor_client(0);

int main()
{
  char c;
  int n, tem;
  int cnt = 0;
  int spin_direction = -1;
  int retangle;
  
  tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));

  fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    printf("error %d opening %s: %s", errno, portname, strerror (errno));
    return -1;
  }
  
  set_interface_attribs (fd, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  // set_blocking (fd, 0);                // set no blocking
  
  
  // Insert code for interfacing with hardware here
  printf( "Press enter to break.\n" );
  while (1) {
    // kbhit code
    n = read(0, &c, 1);
    if (n > 0) break;
    cnt = cnt + 1;

    // Send next message
    if(spin_direction == 1) {
	sendCommand(kTime, kAngle);
    }
    else {
      sendCommand(kTime, 0);
    }
    spin_direction = -1*spin_direction;
    usleep((useconds_t) kTime*1000*1000);
  }
  fcntl(0, F_SETFL, tem);
  printf("cnt=%d\n", cnt);
  
  return 0;
}

#define BUFLEN 1024
void sendCommand(float time_cmd, float angle_cmd) {
  // This buffer is for passing around messages.
  uint8_t communication_buffer[BUFLEN];
  // Stores length of message to send or receive
  uint8_t communication_length;

  // Generate the set messages
  angle.trajectory_angular_displacement_.set(com,angle_cmd);
  angle.trajectory_duration_.set(com,time_cmd);

  //motor_client.obs_absolute_angle_.get(com);
  angle.obs_angular_displacement_.get(com);
  
  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(com.GetTxBytes(communication_buffer,communication_length))
  {
    // original arduino prototype
    // Serial.write(communication_buffer,communication_length);
    write(fd, communication_buffer,communication_length);
  }

  // Reads however many bytes are currently available
  communication_length = read(fd, communication_buffer, BUFLEN);
  // Puts the recently read bytes into com's receive queue
  com.SetRxBytes(communication_buffer,communication_length);
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
  // while we have message packets to parse
  while(com.PeekPacket(&rx_data,&rx_length)) {
    // Remember time of received packet
    // communication_time_last = millis();
    // Share that packet with all client objects
    //motor_client.ReadMsg(com,rx_data,rx_length);
    angle.ReadMsg(com,rx_data,rx_length);
    
    // Once we're done with the message packet, drop it
    com.DropPacket();
  }

  // Check if we have any fresh data
  // Checking for fresh data is not required, it simply
  // lets you know if you received a message that you
  // have not yet read.
  //  if(motor_client.obs_absolute_angle_.IsFresh()) {
  if(angle.obs_angular_displacement_.IsFresh()) {
    //    printf( "angle: %2.4f\n", motor_client.obs_absolute_angle_.get_reply() );
    printf( "angle: %3.2f\n", angle.obs_angular_displacement_.get_reply() * 180.0 * M_PI / 10.0 );
  }

#if 0
  write (fd, "hello!\n", 7);           // send 7 character greeting
  
  usleep ((7 + 25) * 100);             // sleep enough to transmit the 7 plus
  // receive 25:  approx 100 uS per char transmit
  char buf [100];
  int n = read (fd, buf, sizeof buf);  // read up to 100 characters if ready to read
#endif

  // Toggle the LED so you know a message just got sent
  printf( "message %d sent\n", ++msg_count );
}

int set_interface_attribs (int fd, int speed, int parity) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0) {
    printf ("error %d from tcgetattr", errno);
    return -1;
  }

  cfsetospeed (&tty, speed);
  cfsetispeed (&tty, speed);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
  // disable IGNBRK for mismatched speed tests; otherwise receive break
  // as \000 chars
  tty.c_iflag &= ~IGNBRK;         // disable break processing
  tty.c_lflag = 0;                // no signaling chars, no echo,
  // no canonical processing
  tty.c_oflag = 0;                // no remapping, no delays
  tty.c_cc[VMIN]  = 0;            // read doesn't block
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

  tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
  // enable reading
  tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
  tty.c_cflag |= parity;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  if (tcsetattr (fd, TCSANOW, &tty) != 0) {
    printf ("error %d from tcsetattr", errno);
    return -1;
  }
  return 0;
}


void set_blocking (int fd, int should_block) {
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  if (tcgetattr (fd, &tty) != 0)
    {
      printf ("error %d from tggetattr", errno);
      return;
    }

  tty.c_cc[VMIN]  = should_block ? 1 : 0;
  tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

  if (tcsetattr (fd, TCSANOW, &tty) != 0)
    printf ("error %d setting term attributes", errno);
}


