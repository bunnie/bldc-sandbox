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
void copyState();

const char *portnameIn = "/dev/ttyUSB0";
const char *portnameOut = "/dev/ttyUSB1";

int msg_count = 0;
int fdIn = -1; // fd for serial port
int fdOut = -1; // fd for serial port

// Make a communication interface object
GenericInterface comIn;
GenericInterface comOut;
// Make a Multi-turn Angle Controller object with obj_id 0
MultiTurnAngleControlClient angleIn(0);
MultiTurnAngleControlClient angleOut(0);


#define UPDATE_RATE 30  // in ms
int main()
{
  char c;
  int n, tem;
  int cnt = 0;
  int spin_direction = -1;
  int retangle;

  /// setup console terminal prooperties
  tem = fcntl(0, F_GETFL, 0);
  fcntl (0, F_SETFL, (tem | O_NDELAY));

  /// now setup motor controller terminal properties
  fdIn = open (portnameIn, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdIn < 0) {
    printf("error %d opening input controller %s: %s", errno, portnameIn, strerror (errno));
    return -1;
  }
  set_interface_attribs (fdIn, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  
  fdOut = open (portnameOut, O_RDWR | O_NOCTTY | O_SYNC);
  if (fdOut < 0) {
    printf("error %d opening input controller %s: %s", errno, portnameOut, strerror (errno));
    return -1;
  }
  set_interface_attribs (fdOut, B115200, 0);  // set speed to 115,200 bps, 8n1 (no parity)
  
  // set_blocking (fd, 0);                // set no blocking
  
  
  // Insert code for interfacing with hardware here
  printf( "Press enter to break.\n" );
  while (1) {
    // kbhit code
    n = read(0, &c, 1);
    if (n > 0) break;
    cnt = cnt + 1;

    // Send next message
    copyState();

    usleep((useconds_t) UPDATE_RATE * 1000);
  }
  fcntl(0, F_SETFL, tem);
  printf("cnt=%d\n", cnt);
  
  return 0;
}

#define BUFLEN 1024
void copyState() {
  // This buffer is for passing around messages.
  uint8_t communication_buffer_in[BUFLEN];
  uint8_t communication_buffer_out[BUFLEN];
  // Stores length of message to send or receive
  uint8_t communication_length_in;
  uint8_t communication_length_out;

  float target_angle;

  ///////////// READ THE INPUT CONTROLLER
  // Generate the set messages
  angleIn.ctrl_coast_.set(comIn); // put the input controller in "coast" mode
  angleIn.obs_angular_displacement_.get(comIn); // get the angular displacement

  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(comIn.GetTxBytes(communication_buffer_in, communication_length_in))
  {
    write(fdIn, communication_buffer_in, communication_length_in);
  }
  
  usleep((useconds_t) 1 * 1000); // delay 1ms for serial data to clear
  
  // Reads however many bytes are currently available
  communication_length_in = read(fdIn, communication_buffer_in, BUFLEN);
  
  // Puts the recently read bytes into com's receive queue
  comIn.SetRxBytes(communication_buffer_in, communication_length_in);
  uint8_t *rx_data; // temporary pointer to received type+data bytes
  uint8_t rx_length; // number of received type+data bytes
  // while we have message packets to parse
  while(comIn.PeekPacket(&rx_data,&rx_length)) {
    // Share that packet with all client objects
    angleIn.ReadMsg(comIn,rx_data,rx_length);
    
    // Once we're done with the message packet, drop it
    comIn.DropPacket();
  }

  // Check if we have any fresh data
  // Checking for fresh data is not required, it simply
  // lets you know if you received a message that you
  // have not yet read.
  if(angleIn.obs_angular_displacement_.IsFresh()) {
    target_angle = angleIn.obs_angular_displacement_.get_reply();
    printf( "angle: %3.2f\n", target_angle * 180.0 * M_PI / 10.0 );
  }


  /////////////// WRITE OUTPUT CONTROLLER
  // Generate the set messages
  angleOut.trajectory_angular_displacement_.set(comOut,target_angle);
  angleOut.trajectory_duration_.set(comOut, (UPDATE_RATE - 1.0) / 1000.0 ); // try to get there in 100ms

  angleOut.obs_angular_displacement_.get(comOut);
  
  // Grab outbound messages in the com queue, store into buffer
  // If it transferred something to communication_buffer...
  if(comOut.GetTxBytes(communication_buffer_out,communication_length_out))
  {
    write(fdOut, communication_buffer_out,communication_length_out);
  }

  // printf( "message %d sent\n", ++msg_count );
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


