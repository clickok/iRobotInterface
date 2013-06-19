/* 
  Sarsa demo of the Create serial-port packet processor" (aka csp3).

  *******************************************************************
  NB: This version is non-canonical-- I am making alterations to the
      code for functional and stylistic reasons. The code is subject
      to change according to the needs of the project.

      The goal for this version is that it run flawlessly on a small
      linux system (likely a Raspberry Pi) which can be attached to
      the robot for increased portability. It may work with other
      environments, but that is not guaranteed.

      Rich and Rupam are responsible for canonical version.
  *******************************************************************

  This file is a self-contained demonstration of the use of the
  "Create serial-port packet processor", a small program for managing
  the real-time interaction of a serial-port connection to an iRobot
  Create robot. The idea is to make it easy to write control programs
  (e.g., reinforcement learning agents) for the Create without
  sacrificing performance in any way. Here we use it to write a simple
  Sarsa(lambda) agent using the four cliff sensors in a binary way to
  make 16 states.

  The recommended usage is to make a copy of this demo file, then modify
  it according to your needs. (For example, adding or subtracting from 
  the set of sensors that are streamed out from the Create robot.)

  But first, you should understand the basics of the program's
  operation.  The serial-port connection to the Create is set up and
  put in a mode where it streams out a packet of sensor data every 15
  milliseconds. Each packet contains data for a specific set of
  sensors that is specified at setup time. The 15ms cycle corresponds
  to how fast the data is generated internally by the Create, so
  getting it in this way is arguably the best possible way. However,
  this way of receiving data is not convenient (in general) for
  writing agent code that processes the sensor data and sends commands
  to the Create. The agent code may involve extensive processing
  lasting more than 15ms, or it may want to send commands for actions
  that are meant to apply for more or less than 15ms. It would be
  awkward to try to break the processing or the actions into smaller
  pieces that fit into 15ms chunks. Even worse, when using the
  bluetooth version of the serial port (via a BAM) the interval
  between packets can become very variable, from 0 to 100ms. There is
  no easy way for agent code to fit into the spaces between packets.

  Basically, at the agent level, one would like to ignore the 15ms
  streaming cycle. Ideally the agent would just obtain the latest
  sensory information at any time, use it for action selections or
  computations of any duration, then repeat. This agent cycle, or
  timestep, might last 100ms, for example. Ideally the cycle would be
  allowed to be of any duration, even less than 15ms, and even varying
  from cycle to cycle.

  The csp3 allows you to do all these things. Basically, we just have
  two threads, one for the csp3 and one for the agent code. The csp3
  thread manages the interaction with the serial port, collecting data
  as it arrives to form a complete packet roughly every 15ms, then
  processes the packets data into a form convenient for reading by the
  agent thread when it next chooses to. Several packets (or no
  packets) may be processed in between agent cycles, and all the
  historical packet data from all of them (up to some limit into the
  past) are made available to the agent thread.

  As a user and modifier of this code, you should understand the
  following key aspects of the communication and coordination between
  the csp3 thread and the agent thread:

    1. After the serial-port connection to the robot is set up, the
       csp3 thread does all the reading from the serial port (and
       because of this no lock is needed).

    2. The global variable pktNum is the number of the currently being
       constructed packet. (Thus pktNum-1 is the number of the last
       completed packet, the latest packet with reliable data in it
       that can be read by the agent thread.) pktNum starts at 0 and
       is incremented forever. (Thus, if a run lasted more than
       15*2^32 ms, which is more than two years, there could be a
       problem.) All writes of pktNum (by the csp3 thread only) and
       reads of pktNum by the agent thread, must be protected by a
       mutex_lock to remove any possibilityof corruption by the two
       processes trying to work with it at the same time.

    3. Sensor readings are constructed by the csp3 thread, and made
       available to the agent thread, in many "sensory" arrays, each
       indexed by packet number. These are given natural names
       descriptive of their sensory data, except that they begin with
       's', for "sensory". For example (but see 5 below):
           sCliffL[p] is the left cliff-sensor reading in the p-th packet
           sRemote[p] is the button pressed on the remote in the p-th packet
           sBumperR[p] is 1/0 corresponding to the right bumper in the p-th packet
           sDeltaT[p] is the time elapsed between the p-th and (p-1)th packet
       The sensory arrays are many and varied and are specific to your
       use; feel free to add more and remove those that you don't
       need. These are all written exclusively by the csp3 thread and
       read by the agent thread. No locks are needed on them because
       the agent only reads data for packets after their writing has
       been completed (p<pktNum).

    4. When the agent thread starts a cycle, it typically will make
       its own copy of pktNum, called myPktNum. The value of myPktNum
       on the last agent cycle may be kept in an agent-thread variable
       called lastPktNum. Then the agent knows that it might want to
       process, at most, all the packet data from lastPktNum to
       myPktNum-1.

    5. As described so far, the sensory arrays in 3 above would need
       an entry for every packet, which would be rather big, and
       unnecessary because generally the agent needs to see only the
       packets since its last cycle, as described in 4 above. Thus, in
       reality only data for the most recent M packets is retained,
       and all the sensory arrays are only of length M. Thus, when you
       access, say, the sCliffL data for the p-th packet, you actually
       access sCliffL[p%M] (p mod M). The mod-ing is typically done at
       array-access time; pktNum, myPktNum, and lastPktNum are all
       kept in explicit form without mod-ing.

    6. The agent thread presumably wants to send commands to the
       Create. This involves sending bytes (according the Create Open
       Interface specification) to the serial port. When doing so, the
       agent thread must lock the serial port so that the csp3 thread
       does not try to use it at the same time, with unreliable
       results. After sending the bytes, the lock must be released, or
       else the csp3 thread will be permanently blocked and your
       computer may even crash. It is trivial to prevent this: you
       just obtain the lock, write your bytes, and then immediately
       release the lock. This demo file is an example.

    7. The csp3 thread may need to read some variables maintained and
       written by the agent thread, and may also send bytes to the
       agent. In this demo, the csp3 thread accesses the most recent
       "action" (motor command) sent by the agent thread to the
       Create, which is kept in the agent variable lastAction. This is
       used, in combination with the cliff sensor data and their
       thresholds, to detect when the action might be take the robot
       outside a prescribed region, in which case the robot is sent a
       command to stop its motion. This is an example of a "reflex",
       which we here define as an immediate response to sensor data
       implemented by the csp3 at its 15ms time cycle for each
       completed packet. The triggering of a reflex for a packet might
       be recorded in a corresponding sensory array. Other reflexes
       might set the Create's lights or cause it to make sound. The
       ability to implement reflexes running at the Creates fastest
       sensory rate is another advantage of csp3's two-thread
       approach.

-----
  To add a new sensor you need to make a number of changes in a number 
  of places. Here are the steps:

    1. Find the sensor id of the new sensor in the iRobot Create Open 
       Interface manual. In the manual, sensor id's are called packet 
       id's (because they indicate that a little "packet" of data is 
       due to that sensor). For example, the left cliff sensor has sensor
       (packet) id 28. This code contains a list of symbolic names many
       of the sensor id's, and the one you want to add may be included.
       If not, feel free to add it.
    2. While you have the Open Interface manual open to the page for
       your sensor, give it a good read. Fully understand the data 
       produced by your sensor. How many bytes is it? How is the data
       represented? You will need to understand these things for the 
       next steps.
    3. Find the global #define for B, the number of bytes in a sensor
       packet. With your new sensor, B will have to be increased. For
       example, if your sensor data has two bytes, then B will have to 
       be increased by 3 (1 for the sensor id and 2 for the data).
    4. Add a declaration of the sensory array for your new sensor to the
       beginning of this file in the Global Names and Variables section.
       Follow the example of the existing names.
    5. Edit the function setupSerialPort: 
         a) add 1 to the value that bytes[1] is set to
	 b) add 1 to the size of the bytes array
	 c) add a new line setting the new last element of the bytes 
	    array to the sensor id of your new sensor
	 d) add 1 to the number in the sendBytesToRobot call
    6. Edit the function checkPacket by adding one line for the new
       sensor at the end of the list of &&s. Follow the pattern of
       the existing lines. To get the array index right you will have
       to know the number of bytes in the data of the previous sensor
       (or infer it from the previous value of B).
    7. Edit the function extractPacket by adding one line for the new
       sensor just before the blank line. Follow the pattern of
       the existing lines. This is where your understanding of the data
       format for your sensor will come in.

-----
  Other things you will need to know:

  To get going, you need a port name to provide as a command line
  argument. This could be either using bluetooth and a BAM or using
  a physical wire to the robot. The wire gives faster and more uniform
  timing but then...it's a wire. In both case, after configuring, the
  port name will appear in /dev with a name like /dev/tty.something, 
  where the something might be usbserial for a wire or SerialElement-Se
  for bluetooth. To configure the wire, it may be sufficient to simply
  plug in the wire, or you may need to download a software driver 
  provided by the manufacturer of your usb-to-serial cable. For bluetooth, 
  go to system preferences > bluetooth and set up the connection and get
  the name. You can also set up a better name for the connection there.

  How do you stop? Ideally, you can press the pause button on the
  remote to stop your robot. This should also terminate the program.
  Otherwise, you may have to chase the robot down to press the power
  off button.

  It seems to be a good idea to power the robot down and up again
  before running the program.

  Many times the program can't get started properly and you have to
  try starting it several times. We have not figured that out yet but
  it does not seem to be a big deal. It will work eventually. On the
  other hand, if the robot's battery gets low it just will never start.
  The battery may be good enough for the built-in demos and for slow
  actions and still too low for the program to start with faster actions.

  This program was originally written by
  A. Rupam Mahmood (ashique@ualberta.ca) and
  Rich Sutton (rich@richsutton.com) in June, 2013.
 */

#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <sys/select.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <float.h>

/******************************************************************************
 *                          Overall To Do List
 *****************************************************************************/
//TODO: Benchmark with minor optimizations (using volatile, using poll())
//TODO: Run code through a profiler once it is stable
//TODO: Look for a better way of displaying real time information than printf()
//TODO: Standardize data output to make it more machine friendly
//TODO: Try to decouple multipurpose functions, if it makes sense
//TODO: Consider separating agent thread from main() function.
//TODO: Add return codes to some of these void functions, if appropriate
//TODO: Set up state-action array to make it more easily modified
//TODO: Check all modular arithmetic statements
//TODO: Figure out rationale for having both myPktNum and pktNum

/******************************************************************************
 *                           Defines and Typedefs
 *****************************************************************************/

#define FALSE 0
#define TRUE 1

/* The Create communiates using 4-bit data bytes, the same size as an
 * (unsigned) char.  The ubyte data type is exactly equivalent, but should
 * lead to less confusion.
 */
typedef unsigned char ubyte;
//TODO: Ensure this behaves properly across various systems

/* The speed at which the Create drives its wheels */
#define SPEED 300

/******************************************************************************
 *                             Create Opcodes
 *****************************************************************************/

#define CREATE_START         128
#define CREATE_SAFE          131
#define CREATE_FULL          132
#define CREATE_SPOT          134
#define CREATE_DRIVE         137
#define CREATE_SONG          140
#define CREATE_PLAY          141
#define CREATE_SENSORS       142
#define CREATE_DRIVE_DIRECT  145
#define CREATE_DIGITAL_OUTS  147
#define CREATE_STREAM        148
#define CREATE_STREAM_PAUSE  150

#define SENSOR_ALL_SENSORS        6
#define SENSOR_WHEEL_DROP         7
#define SENSOR_IRBYTE             17
#define SENSOR_DISTANCE           19
#define SENSOR_ROTATION           20
#define SENSOR_BAT_VOLTAGE        22
#define SENSOR_BAT_CURRENT        23
#define SENSOR_BAT_TEMP           24
#define SENSOR_BAT_CHARGE         25
#define SENSOR_CLIFF_LEFT         28
#define SENSOR_CLIFF_FRONT_LEFT   29
#define SENSOR_CLIFF_FRONT_RIGHT  30
#define SENSOR_CLIFF_RIGHT        31

#define SENSOR_START_BYTE         19

/******************************************************************************
 *                       Global Names and Variables
 *****************************************************************************/

sem_t should_terminate;       // A semaphore for csp3() termination

volatile unsigned int pktNum = 0;      // Number of the packet currently being constructed by csp3
pthread_mutex_t pktNumMutex, serialMutex, lastActionMutex; // locks
int lastAction = 0;           // last action sent to Create by agent
struct timeval lastPktTime;   // time of last packet
int fd = 0;                   // file descriptor for serial port
#define B 20                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

//sensory arrays:
#define M 1000                /* The ring buffer size */
unsigned short  sCliffL[M], sCliffR[M], sCliffFL[M], sCliffFR[M]; // cliff sensors (small positive integers)
ubyte  sCliffLB[M], sCliffRB[M], sCliffFLB[M], sCliffFRB[M];      // (binary 1/0)
short  sDistance[M];          // wheel rotation counts (small integers, pos/neg)
double sDeltaT[M];            // in milliseconds
ubyte sIRbyte[M];             // Infrared byte e.g. remote

int cliffThresholds[4];       // left, front left, front right, right
int cliffHighValue;           // binary value taken if threshold exceeded



/*****************************************************************************
 *                                Macros
 *****************************************************************************/

#define MAX(a,b) (a > b?a:b)
#define MIN(a,b) (a < b?a:b)

/*****************************************************************************
 *                           Helper Functions
 *****************************************************************************/

/* void loadCliffThresholds()
 * Opens a file in the local directory named cliffThresholds.dat
 * which contains thresholds for the infrared cliff sensors, in
 * order to differentiate between the "inside" and "outside" regions.
 */
void loadCliffThresholds() {
  FILE *fd;
  if ((fd = fopen("cliffThresholds.dat", "r"))==NULL) {
    fprintf(stderr, "Error opening cliffThresholds.dat file: %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  fscanf(fd, "%d%d%d%d%d", &cliffHighValue, 
	 &cliffThresholds[0], &cliffThresholds[1], 
	 &cliffThresholds[2], &cliffThresholds[3]);
  fclose(fd);
}

/* void sendBytesToRobot()
 * Sends a series of bytes (passed as an array) to the robot. The number of
 * bytes sent must be specified (with the usual caveats about buffer overruns).
 * It locks the serialMutex while writing, and if the write() fails it returns
 * an error.
 */
void sendBytesToRobot(ubyte* bytes, int numBytes) {
  int ret;
  pthread_mutex_lock( &serialMutex );
  if ((ret=write(fd, bytes, numBytes))==-1) {
    fprintf(stderr, "Problem with write(): %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  pthread_mutex_unlock( &serialMutex );
}

/* void ensureTransmitted()
 * Locks the serialMutex and then calls tcdrain() to ensure that the output
 * buffer is properly written to the serial port.
 */
void ensureTransmitted() {
  int ret;
  pthread_mutex_lock( &serialMutex );
  if ((ret=tcdrain(fd))==-1) {
	fprintf(stderr, "Problem with tcdrain(): %s\n", strerror(errno));
	exit(EXIT_FAILURE);
  }
  pthread_mutex_unlock( &serialMutex );
}


/* void driveWheels()
 * Sends commands to the Create instructing it to drive its wheels with the
 * specified velocity (with a maximum of 500 mm/s and a minimum of -500 mm/s
 */
void driveWheels(int left, int right) {
  ubyte bytes[5];
  left = MIN(500, left);
  left = MAX(-500, left);
  right = MIN(500, right);
  right = MAX(-500, right);
  bytes[0] = CREATE_DRIVE_DIRECT;
  bytes[1] = (right >> 8) & 0x00FF;
  bytes[2] = right & 0x00FF;
  bytes[3] = (left >> 8) & 0x00FF;
  bytes[4] = left & 0x00FF;
  sendBytesToRobot(bytes, 5);
}

/* void setupSerialPort()
 * Connects to the serial port associated with the Create (as specified by
 * the argument, e.g., "/dev/tty.usbserial"), and associating the global file
 * descriptor fd with the serial port connection.
 * It then puts the robot in  "Full" mode, and requests a data stream of the
 * robot's sensors.
 * It also sets up a "song" the Create can play (perhaps when getting rewarded)
 */
//TODO: Consider decoupling the set up of the serial port with that of the Create
void setupSerialPort(char serialPortName[]) {
  struct termios options;
  ubyte byte;
  ubyte bytes[8];

  // open connection
  if((fd = open(serialPortName, O_RDWR | O_NOCTTY | O_NONBLOCK))==-1) {
    fprintf(stderr, "Serial port at %s could not be opened: %s\n", 
	   serialPortName, strerror(errno));
    exit(EXIT_FAILURE);
  }
  tcflush (fd, TCIOFLUSH);
  tcgetattr(fd, &options);
  options.c_iflag = IGNBRK | IGNPAR;
  options.c_lflag = 0;
  options.c_oflag = 0;
  options.c_cflag     = CREAD | CS8 | CLOCAL; // CLOCAL not needed for cu.portname
  cfsetispeed(&options, B57600);
  cfsetospeed(&options, B57600);
  tcsetattr(fd, TCSANOW, &options);
  // go to passive mode:
  byte = CREATE_START;
  sendBytesToRobot(&byte, 1);
  // go to full mode:
  byte = CREATE_FULL;
  sendBytesToRobot(&byte, 1);
  // Request stream mode:
  bytes[0] = CREATE_STREAM;
  bytes[1] = 6;
  bytes[2] = SENSOR_CLIFF_LEFT;
  bytes[3] = SENSOR_CLIFF_FRONT_LEFT;
  bytes[4] = SENSOR_CLIFF_FRONT_RIGHT;
  bytes[5] = SENSOR_CLIFF_RIGHT;
  bytes[6] = SENSOR_DISTANCE;
  bytes[7] = SENSOR_IRBYTE;
  sendBytesToRobot(bytes, 8);
  // Setup songs
  bytes[0] = CREATE_SONG;
  bytes[1] = 0;
  bytes[2] = 1;
  bytes[3] = 100;
  bytes[4] = 6;
  sendBytesToRobot(bytes, 5);
  ensureTransmitted();
}

/* int checkPacket()
 * Performs a check to ensure the data stored in packet[] is properly formed
 * and therefore valid.
 * First it verifies that the packet begins with the start byte (19) and then
 * proceeds through the rest of the packet to ensure that the number of bytes
 * sent is valid, and all the packet IDs are as expected. Then it calculates
 * the checksum of the entire packet.
 * If the packet is valid, it returns 1, else 0.
 * expects (19) (SENSOR_SIZE-3) (SENSOR_CLIFF_LEFT) () () ... (checksum)
 */
// TODO: Replace instances of magic numbers with defined macros
// TODO: Try to make this code more general in case we wish to vary the sensors
int checkPacket() {
  int i, sum;
  if (packet[0]==19 &&
      packet[1]==B-3 &&
      packet[2]==SENSOR_CLIFF_LEFT &&
      packet[5]==SENSOR_CLIFF_FRONT_LEFT &&
      packet[8]==SENSOR_CLIFF_FRONT_RIGHT &&
      packet[11]==SENSOR_CLIFF_RIGHT &&
      packet[14]==SENSOR_DISTANCE &&
      packet[17]==SENSOR_IRBYTE)
  {
    sum = 0;
    for (i = 0; i < B; i++) sum += packet[i];
    if ((sum & 0xFF) == 0) return 1;
  }
  return 0;
}

/* void extractPacket()
 * Performs the extraction on the packet's data. This can be complicated by
 * the fact that some sensors require more than one byte to represent their
 * information, so some bit shifting is required.
 * After this is done, it also gets the time value
 */
//TODO: Improve time extraction (instead of using gettimeofday)
//TODO: Make more general, so that we can use different sensors/combinations
void extractPacket() {
  struct timeval currentTime;
  int p = pktNum%M;
  sCliffL[p]   = packet[3]<<8 | packet[4];
  sCliffLB[p]  = sCliffL[p]>cliffThresholds[0] ? cliffHighValue : 1-cliffHighValue;
  sCliffFL[p]  = packet[6]<<8 | packet[7];
  sCliffFLB[p] = sCliffFL[p]>cliffThresholds[1] ? cliffHighValue : 1-cliffHighValue;
  sCliffFR[p]  = packet[9]<<8 | packet[10];
  sCliffFRB[p] = sCliffFR[p]>cliffThresholds[2] ? cliffHighValue : 1-cliffHighValue;
  sCliffR[p]   = packet[12]<<8 | packet[13];
  sCliffRB[p]  = sCliffR[p]>cliffThresholds[3] ? cliffHighValue : 1-cliffHighValue;
  sDistance[p] = packet[15]<<8 | packet[16];
  sIRbyte[p] = packet[18];

  gettimeofday(&currentTime, NULL);
  sDeltaT[p] = (currentTime.tv_sec - lastPktTime.tv_sec)*1000
    + ((double) currentTime.tv_usec - lastPktTime.tv_usec)/1000;
  lastPktTime = currentTime;
}


//void reflexes();
/* void reflexes()
 * Takes actions at a faster rate to ensure the Create doesn't endanger itself
 * (by falling off a table, for example)
 */
void reflexes() {
  int a;
  int p = pktNum%M;
  pthread_mutex_lock( &lastActionMutex );
  a = lastAction;
  pthread_mutex_unlock( &lastActionMutex );
  if ((a==0 && (sCliffFLB[p] || sCliffFRB[p])) || // attempt to go forward over cliff
      (a==3 && (sCliffLB[p] || sCliffRB[p]))) {   // attempt to go backward over cliff
    driveWheels(0, 0);                            // then interrupt motion
  }

  ubyte bytes[2];
  ubyte frontbit = sCliffFLB[p] || sCliffFRB[p];
  ubyte ledbits = (sCliffLB[p] << 2) | (frontbit << 1) | sCliffRB[p];
  bytes[0] = CREATE_DIGITAL_OUTS;
  bytes[1] = ledbits;
  sendBytesToRobot(bytes, 2);
}

/* void * csp3()
 * A packet processing function, which calls various helper functions in order
 * to get data from the robot, parse it, and place it into the appropriate
 * data structures to be used by the rest of the program.
 */
//TODO: Try using poll() over select()
void csp3(void *arg)
{
	int errorCode, numBytesRead, i;
	int stopFlag = 0;
	ubyte bytes[B];
	int numBytesPreviouslyRead = 0;
	struct timeval timeout;
	fd_set readfs;

	gettimeofday(&lastPktTime, NULL);
	FD_SET(fd, &readfs);

	while (TRUE)
	{
		sem_getvalue(&should_terminate,&stopFlag);
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		errorCode = select(fd+1, &readfs, NULL, NULL, &timeout);
		if (errorCode==0)
		{
			printf("Timed out at select()\n");
		}
		else if (errorCode==-1)
		{
			fprintf(stderr, "Problem with select(): %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		}
		numBytesRead = read(fd, &bytes, B-numBytesPreviouslyRead);
		if (numBytesRead==-1)
		{
			fprintf(stderr, "Problem with read(): %s\n", strerror(errno));
			exit(EXIT_FAILURE);
		}
		else
		{
			for (i = 0; i < numBytesRead; i++)
			{
				packet[numBytesPreviouslyRead+i] = bytes[i];
			}
			numBytesPreviouslyRead += numBytesRead;
			if (numBytesPreviouslyRead==B) /* packet complete! */
			{
				if (checkPacket())
				{
					extractPacket();
					reflexes();
					ensureTransmitted();
					pthread_mutex_lock( &pktNumMutex );
					++pktNum;
					pthread_mutex_unlock( &pktNumMutex );
					numBytesPreviouslyRead = 0;
				}
				else
				{
					printf("misaligned packet.\n");
					for (i = 1; i<B; i++)
					{
						packet[i-1] = packet[i];
					}
					numBytesPreviouslyRead--;
				}
			}
		}
	}
}

/* int getPktNum()
 * Sets the global variable myPktNum to the same value as the global pktNum.
 */
int getPktNum() {
  int myPktNum;
  pthread_mutex_lock( &pktNumMutex );
  myPktNum = pktNum;
  pthread_mutex_unlock( &pktNumMutex );
  return myPktNum;  
}

/* void takeAction()
 * Performs an action (currently using driveWheels to drive forward, backward,
 * or turn) depending on the argument.
 */
void takeAction(int action) {
    switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(-SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, -SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    default : printf("Bad action\n");
    }
}

/* int epsilonGreedy()
 * Takes an array representing state-action values, a state, and an epsilon and
 * chooses an action. It will choose randomly with probability epsilon, but
 * greedily otherwise.
 */
int epsilonGreedy(double Q[16][4], int s, int epsilon)
{
  int max, i;
  int myPktNum, p;
  int firstAction, lastAction;

  myPktNum = getPktNum();
  //TODO: Better modular arithmetic
  p = (myPktNum - 1) % M;
  firstAction = sCliffFLB[p] || sCliffFRB[p];
  if (sCliffLB[p] || sCliffRB[p])
  {
	  lastAction = 2;
  }
  else lastAction = 3;
  if (rand()/((double)RAND_MAX+1) < epsilon)
  {
    printf("random action\n\n");
    return firstAction + rand()%(lastAction + 1 - firstAction);
  }
  else
  {
    max = lastAction;
    for (i = firstAction; i < lastAction; i++)
    {
      if (Q[s][i] > Q[s][max])
      {
        max = i;
      }
    }
    return max;
  }
}

/* void endProgram()
 * Performs tasks related to program termination.
 * Ensures that the csp3() thread is terminated, stops the Create and sets it
 * to "passive" mode, then disconnects from the serial port.
 */
void endProgram()
{
	int val;
	sem_post(&should_terminate);
	sem_getvalue(&should_terminate,&val);
	printf("[DEBUG] should_terminate value: %d\n",val);
}

/* main()
 * The main function. Takes command line arguments specifying the (serial)
 * port that the robot is connected to, and uses the above helper functions to
 * set up the port and communicate with the robot.
 * It spawns an additional thread which handles the data sent by the robot, but
 * controls the robot (using the SARSA algorithm) itself.
 */
int main(int argc, char *argv[])
{
	/* Initialize variables */
	pthread_t tid;                  	   	// Reader thread
	unsigned int myPktNum;                  // Packet number variable
	unsigned int prevPktNum;				// Previous packet number
	int p;									// Byte tracking variable
	double Q[16][4];						// State-Action value array
	double e[16][4];						// Eligibility trace array
	double stepsize = 0.1;					// Stepsize (alpha) parameter
	double lambda = 0.9;					// Trace decay parameter
	double gamma = 0.98;                    // Discount parameter
	double epsilon = 0.01;                  // Exploration parameter
	int a, aprime;                          // Action
	int s, sprime;                          // State
	int reward;                             // Reward
	int i, j;								// Iterator variables
	double delta;							// Update
	ubyte bytes[2];         				// Robot command array
	int rewardReport;						// Reward tracking (for song)
	struct timeval timeStart, timeEnd;      // Timing related
	long computationTime; 					// Timing related

	if (argc < 2)
	{
		fprintf(stderr, "Portname argument required -- something like /dev/tty.usbserial\n");
		exit(EXIT_FAILURE);
	}

	loadCliffThresholds();
	srand(0);
	pthread_mutex_init(&pktNumMutex, NULL);
	pthread_mutex_init(&serialMutex, NULL);
	pthread_mutex_init(&lastActionMutex, NULL);

	setupSerialPort(argv[1]);
	usleep(20000); // wait for at least one packet to have arrived
	if (0 != pthread_create(&tid, NULL, &csp3, NULL))
	{
		perror("Cannot create thread\n");
		exit(EXIT_FAILURE);
	}
	else
	{
		sem_init(&should_terminate, 0, FALSE);
	}
	prevPktNum = 0;

	// initialize Q
	for (i = 0; i < 16; i++)
	{
		for (j = 0; j < 4; j++)
		{
			Q[i][j] = 5 + 0.001*( rand()/((double) RAND_MAX) - 0.5);
			e[i][j] = 0;
		}
	}
	gettimeofday(&timeStart, NULL);
	myPktNum = getPktNum();
	p = (myPktNum - 1) % M;
	s = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
	a = epsilonGreedy(Q, s, epsilon);
	takeAction(a);
	ensureTransmitted();
	prevPktNum = myPktNum;
	rewardReport = 0;
	while (TRUE)
	{
		gettimeofday(&timeEnd, NULL);
		computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
		+ (timeEnd.tv_usec-timeStart.tv_usec);
		printf("Time for iteration (in microseconds): %ld\n", computationTime);
		usleep(100000 - computationTime);
		timeStart = timeEnd;
		gettimeofday(&timeStart, NULL);
		myPktNum = getPktNum();
		if (myPktNum - prevPktNum > M)
		{
			fprintf(stderr, "Buffer overflow!\n");
			exit(EXIT_FAILURE);
		}
		reward = 0;
		for (p = prevPktNum; p < myPktNum; p++)
		{
			reward += sDistance[p%M];
			printf("deltaT: %f cliff sensors: %u(%u) %u(%u) %u(%u) %u(%u) distance: %hd\n",
					sDeltaT[p%M],
					sCliffL[p%M],sCliffLB[p%M],sCliffFL[p%M],sCliffFLB[p%M],
					sCliffFR[p%M],sCliffFRB[p%M],sCliffR[p%M],sCliffRB[p%M],
					(short) sDistance[p%M]);
			if (sIRbyte[p%M]==137)
			{
				driveWheels(0, 0);
				tcdrain(fd);
				endProgram();
				return 0;  // quit on remote pause
			}
		}
		rewardReport += reward;
		if (rewardReport>50)
		{
			bytes[0] = 141;
			bytes[1] = 0;
			sendBytesToRobot(bytes, 2);
			rewardReport -= 50;
		}
		p = (myPktNum - 1) % M;
		sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
		aprime = epsilonGreedy(Q, sprime, epsilon);
		takeAction(aprime);
		ensureTransmitted();
		delta = reward + gamma*Q[sprime][aprime] - Q[s][a];
		for (j = 0; j < 4; j++)
		{
			e[s][j] = 0;
		}
		e[s][a] = 1;
		printf("s a r s' a':%d %d %d %d %d\n", s, a, reward, sprime, aprime);
		for (i = 0; i < 16; i++)
		{
			printf("Action values for state %d: %f %f %f %f\n",i, Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
			printf("Eligibility traces for state %d: %f %f %f %f\n", i, e[i][0], e[i][1], e[i][2], e[i][3]);
			for (j = 0; j < 4; j++)
			{
				Q[i][j] = Q[i][j] + stepsize*delta*e[i][j];
				e[i][j] = gamma*lambda*e[i][j];
			}
		}
		s = sprime;
		a = aprime;
		pthread_mutex_lock( &lastActionMutex );
		lastAction = aprime;
		pthread_mutex_unlock( &lastActionMutex );
		prevPktNum = myPktNum;
	}
	return 0;
}
