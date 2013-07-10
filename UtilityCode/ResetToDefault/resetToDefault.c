/* AvgRewardSarsaReplacing.c
   Author: Rupam
   Modified by Brendan for experiments with modification of alpha parameter
 */

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

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

/* The speed at which the Create drives its wheels */
#define SPEED 100

/* The reward threshold for the Create to "sing"   */
#define SONG_REWARD_THRESHOLD  100

/*****************************************************************************
 *                                Macros
 *****************************************************************************/

#define MAX(a,b) (a > b?a:b)
#define MIN(a,b) (a < b?a:b)

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

int resetPhase = 0;
pthread_mutex_t resetPhaseMutex;

FILE * logFile;
pthread_t tid;                // Thread for csp3()

int terminationFlag = FALSE;  // A flag set when the program should end
unsigned int pktNum = 0;      // Number of the packet currently being constructed by csp3
pthread_mutex_t pktNumMutex, serialMutex, lastActionMutex, endFlagMutex; // locks
int lastAction = 0;           // last action sent to Create by agent
struct timeval lastPktTime;   // time of last packet
int fd = 0;                   // file descriptor for serial port
#define B 20                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

/* Sensor Data Arrays */
#define M 1000                /* The ring buffer size */
/* Infrared cliff sensors (small positive integers) */
unsigned short  sCliffL[M];
unsigned short  sCliffR[M];
unsigned short  sCliffFL[M];
unsigned short  sCliffFR[M];
/* (binary 1/0) */
ubyte           sCliffLB[M];
ubyte           sCliffRB[M];
ubyte           sCliffFLB[M];
ubyte           sCliffFRB[M];
short           sDistance[M]; // wheel rotation counts (small integers, pos/neg)
double          sDeltaT[M];   // in milliseconds
ubyte           sIRbyte[M];   // Infrared byte e.g. remote
char * portName;

/* Cliff thresholds (taken from cliffThresholds.dat) */
int cliffThresholds[4];       // left, front left, front right, right
int cliffHighValue;           // binary value taken if threshold exceeded

/*---------------------------------------------------------------------------*/

/*****************************************************************************
 *                      Helper Function Prototypes
 *****************************************************************************/


void setupSerialPort(char serialPortName[]);
void csp3(void *arg);
void loadCliffThresholds();
int  actionChooser(int s);
void takeAction(int action);
void driveWheels(int left, int right);
void sendBytesToRobot(ubyte* bytes, int numBytes);
void ensureTransmitted();
int  getPktNum();
void endProgram();

/* ****************************************************************************
 * Main function (implements the actual SARSA algorithm)
 * ***************************************************************************/

int main(int argc, char *argv[])
{
	/* Initialize variables */
	unsigned int myPktNum;                  // Packet number variable
	unsigned int prevPktNum = 0;		    // Previous packet number
	int p;									// Byte tracking variable
	int iteration = 0;                      // Control loop counter
	int timestep = 100000;                  // Timestep in microseconds
	int a, aprime;                          // Action
	int s, sprime;                          // State
	int i, j;								// Iterator variables
	struct timeval timeBegin;               // Control loop start time
	struct timeval timeStart, timeEnd;      // Timing related
	long computationTime; 					// Timing related

	/* Load cliff thresholds, seed RNG */
	loadCliffThresholds();
	srand(0);


	/* ************************************************************************
	 *                    Handle command line arguments
	 * ***********************************************************************/

	if (argc < 2)
	{
		fprintf(stderr, "Portname argument required -- something like -p /dev/tty.usbserial\n");
	    return 0;
	}
	while (1)
	{
		static struct option long_options[] =
		{
			{"port",            required_argument,   0, 'p'},
			{"timestep",        required_argument,   0, 't'},
			{"help",            no_argument,         0, 'h'},
			{0, 0, 0, 0}
		};
		int c;
		int option_index = 0;

		c = getopt_long(argc, argv, "p:t:",long_options, &option_index);
		/* Detect end of options */
		if (c == -1) break;

		/* Set options based on command line arguments */
		switch(c)
		{
		case 'p':
			portName = optarg;
			break;
		case 't':
			timestep = strtol(optarg, (char **) NULL, 10);
			break;
		case '?':
			fprintf(stderr,"ERROR: Unknown command line argument\n");
			exit(EXIT_FAILURE);
		}
	}




	/* ************************************************************************
	 *                Set up resources used by program
	 * ***********************************************************************/

	/* Initialize mutex locks */
	pthread_mutex_init(&pktNumMutex, NULL);
	pthread_mutex_init(&serialMutex, NULL);
	pthread_mutex_init(&lastActionMutex, NULL);
	pthread_mutex_init(&endFlagMutex, NULL);
	pthread_mutex_init(&resetPhaseMutex,NULL);

	/* Install signal handler */
	struct sigaction act;                   // The new sigaction
	struct sigaction oldact;                // To preserve old sigaction
	act.sa_handler = endProgram;            // Set endProgram() as sig handler
	sigemptyset(&act.sa_mask);              // Empty act's signal set
	act.sa_flags = 0;                       // Set act's flags to 0
	sigaction(SIGINT, &act, &oldact);       // Set act to respond to SIGINT


	/* Set up serial port and begin receiving data */

	if (portName != NULL) setupSerialPort(portName);
	usleep(20000); // wait for at least one packet to have arrived
	if (0 != pthread_create(&tid, NULL, (void *) &csp3, NULL))
	{
		perror("Could not create thread");
		exit(EXIT_FAILURE);
	}

	/* Get time just before control loop starts */
	gettimeofday(&timeBegin, NULL);

	/* Get first state-action pair */
	gettimeofday(&timeStart, NULL);
	myPktNum = getPktNum();
	p = (myPktNum + M - 1) % M;
	s = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
	a = actionChooser(s);
	takeAction(a);
	ensureTransmitted();
	prevPktNum = myPktNum;

	/* ************************************************************************
	 * Control loop
	 * ***********************************************************************/
	while (TRUE)
	{
		printf("Iteration number: %6d\n",++iteration);
		gettimeofday(&timeEnd, NULL);
		computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
						+ (timeEnd.tv_usec-timeStart.tv_usec);
		printf("Time for iteration (in microseconds): %ld\n", computationTime);
		usleep(timestep - computationTime);
		timeStart = timeEnd; //TODO Is this needed?
		gettimeofday(&timeStart, NULL);
		myPktNum = getPktNum();
		if (myPktNum - prevPktNum > M)
		{
			fprintf(stderr, "Buffer overflow!\n");
			exit(EXIT_FAILURE);
		}

		for (p = prevPktNum; p < myPktNum; p++)
		{
			if (sIRbyte[p%M]==137)
			{
				endProgram();
			}
		}


		/* Get next state, choose action based on it */
		p = (myPktNum + M - 1) % M;
		sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
		aprime = actionChooser(sprime);
		takeAction(aprime);
		ensureTransmitted();

	    /* Prepare for next loop */
	    s = sprime;
	    a = aprime;
	    pthread_mutex_lock( &lastActionMutex );
	    lastAction = aprime;
	    pthread_mutex_unlock( &lastActionMutex );
	    prevPktNum = myPktNum;
	}
	return 0;
}



/* void endProgram()
 * Performs tasks related to program termination.
 * Ensures that the csp3() thread is terminated, stops the Create and sets it
 * to "passive" mode, then disconnects from the serial port.
 */
void endProgram()
{
	int count=0;
	ubyte bytes[10];
	/* Cancel csp3()'s thread */
	pthread_mutex_lock( &endFlagMutex);
	terminationFlag = TRUE;
	pthread_mutex_unlock( &endFlagMutex);
	pthread_join(tid,NULL);
	fprintf(stderr,"\n[DEBUG] Threads successfully joined\n");
	/* Stop the robot */
	driveWheels(0, 0);
	ensureTransmitted();
	fprintf(stderr,"[DEBUG] driveWheels() stop successfully sent\n");
	/* Pause stream, return robot to passive mode */
	bytes[count++] = 150;
	bytes[count++] = 0;
	bytes[count++] = 128;
	sendBytesToRobot(bytes,count);
	ensureTransmitted();
	fflush(NULL);  // Flush all open streams
	usleep(20000); // Give time for commands to be sent/received
	fprintf(stderr,"[DEBUG] endProgram() complete \n");
	exit(EXIT_SUCCESS);
}

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

void sendBytesToRobot(ubyte* bytes, int numBytes) {
  int ret;
  pthread_mutex_lock( &serialMutex );
  if ((ret=write(fd, bytes, numBytes))==-1) {
    fprintf(stderr, "Problem with write(): %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
  pthread_mutex_unlock( &serialMutex );
}

void ensureTransmitted() {
  int ret;
  pthread_mutex_lock( &serialMutex );
  if ((ret=tcdrain(fd))==-1) {
	fprintf(stderr, "Problem with tcdrain(): %s\n", strerror(errno));
	exit(EXIT_FAILURE);
  }
  pthread_mutex_unlock( &serialMutex );
}

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

// expects (19) (SENSOR_SIZE-3) (SENSOR_CLIFF_LEFT) () () ... (checksum)
int checkPacket() {
  int i, sum;
  if (packet[0]==19 &&
      packet[1]==B-3 &&
      packet[2]==SENSOR_CLIFF_LEFT &&
      packet[5]==SENSOR_CLIFF_FRONT_LEFT &&
      packet[8]==SENSOR_CLIFF_FRONT_RIGHT &&
      packet[11]==SENSOR_CLIFF_RIGHT &&
      packet[14]==SENSOR_DISTANCE &&
      packet[17]==SENSOR_IRBYTE) {
    sum = 0;
    for (i = 0; i < B; i++) sum += packet[i];
    if ((sum & 0xFF) == 0) return 1;
  }
  return 0;
}

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


int getPktNum() {
  int myPktNum;
  pthread_mutex_lock( &pktNumMutex );
  myPktNum = pktNum;
  pthread_mutex_unlock( &pktNumMutex );
  return myPktNum;
}

void takeAction(int action) {
    switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(-SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, -SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    default : printf("Bad action\n");
    }
}


int actionChooser(int s)
{
	int choice;
	int myPktNum, p;

	myPktNum = getPktNum(); //TODO Simplify this, if possible
	p = (myPktNum + M - 1) % M;


	pthread_mutex_lock(&resetPhaseMutex);
	if (resetPhase == 0)
	{
		choice = 0;
		resetPhase = 1;
	}
	else if (resetPhase == 1)
	{
		choice = 1;
		resetPhase = 0;
	}
	else
	{
		choice = 1;
		resetPhase = 1;
	}
	pthread_mutex_unlock(&resetPhaseMutex);
    return choice;
}

void csp3(void *arg)
{
	int errorCode, numBytesRead, i;
	ubyte bytes[B];
	int numBytesPreviouslyRead = 0;
	struct timeval timeout;
	fd_set readfs;

	gettimeofday(&lastPktTime, NULL);
	FD_SET(fd, &readfs);

	/* Reading loop */
	while (TRUE)
	{
		pthread_mutex_lock( &endFlagMutex);
		if (terminationFlag == TRUE) break;
		pthread_mutex_unlock( &endFlagMutex);
		timeout.tv_sec = 2;
		timeout.tv_usec = 0;
		errorCode = select(fd+1, &readfs, NULL, NULL, &timeout);
		if (errorCode==0)
		{
			fprintf(stderr,"Timed out at select()\n");
		}
		else if (errorCode==-1)
		{
			fprintf(stderr, "Problem with select(): %s\n", strerror(errno));
			//endProgram();
			exit(EXIT_FAILURE);
		}

		numBytesRead = read(fd, &bytes, B-numBytesPreviouslyRead);
		if (numBytesRead==-1)
		{
			fprintf(stderr, "Problem with read(): %s\n", strerror(errno));
			//endProgram(); //TODO See if it is possible to implement this instead of just exit()
			exit(EXIT_FAILURE);

		}
		else
		{
			for (i = 0; i < numBytesRead; i++) packet[numBytesPreviouslyRead+i] = bytes[i];
			numBytesPreviouslyRead += numBytesRead;
			if (numBytesPreviouslyRead==B) {  //packet complete!
				if (checkPacket())
				{
					extractPacket();
					reflexes();
					ensureTransmitted();
					pthread_mutex_lock( &pktNumMutex );
					pktNum++;
					pthread_mutex_unlock( &pktNumMutex );
					numBytesPreviouslyRead = 0;
				}
				else
				{
					printf("misaligned packet.\n");
					//TODO Does this have a range error?
					for (i = 1; i<B; i++) packet[i-1] = packet[i];
					numBytesPreviouslyRead--;
				}
			}
		}
	}
}


