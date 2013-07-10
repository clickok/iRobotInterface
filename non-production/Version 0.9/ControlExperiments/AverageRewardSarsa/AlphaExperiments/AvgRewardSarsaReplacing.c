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
#define SPEED 300

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
int  epsilonGreedy(double Q[16][4], int s, double epsilon);
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
	int maxIterations = 1200;               // Limit for number of iterations
	double Q[16][4];						// State-Action value array
	double e[16][4];						// Eligibility trace array
	double alpha = 0.1;						// Stepsize (alpha) parameter
	double alphaR = 0.001;                  // Average Reward Stepsize
	double lambda = 0.9;					// Trace decay parameter
	double epsilon = 0.01;                  // Exploration parameter
	int timestep = 100000;                  // Timestep in microseconds
	int a, aprime;                          // Action
	int s, sprime;                          // State
	int reward;                             // Reward
	double avgReward = 0;					// Average Reward
	int i, j;								// Iterator variables
	double delta;							// Update
	ubyte bytes[2];         				// Robot command array
	int rewardReport = 0;					// Reward tracking (for song)
	struct timeval timeBegin;               // Control loop start time
	struct timeval timeStart, timeEnd;      // Timing related
	long computationTime; 					// Timing related
	char * logName = NULL;                  // Name of log file
	char * portName = NULL;                 // Name of serial port
	char * robotName = NULL;                // Name of robot
	char * batteryName = NULL;              // Name of battery
	char * microworldName = NULL;           // Name of microworld
	char strbuf[1000];                      // String buffer for use w/ logging

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
			{"logname",         required_argument,   0, 'f'},
			{"alpha",           required_argument,   0, 'a'},
			{"epsilon",         required_argument,   0, 'e'},
			{"lambda",          required_argument,   0, 'l'},
			{"timestep",        required_argument,   0, 't'},
			{"microworldname",  required_argument,   0, 'm'},
			{"batteryname",     required_argument,   0, 'b'},
			{"robotname",       required_argument,   0, 'r'},
			{"help",            no_argument,         0, 'h'},
			{0, 0, 0, 0}
		};
		int c;
		int option_index = 0;

		c = getopt_long(argc, argv, "p:f:a:r:m:b:t:",long_options, &option_index);
		/* Detect end of options */
		if (c == -1) break;

		/* Set options based on command line arguments */
		switch(c)
		{
		case 'p':
			portName = optarg;
			break;
		case 'f':
			logName = optarg;
			break;
		case 'm':
			microworldName = optarg;
			break;
		case 'b':
			batteryName = optarg;
			break;
		case 'r':
			robotName = optarg;
			break;
		case 't':
			timestep = strtol(optarg, (char **) NULL, 10);
			break;
		case 'a':
			alpha = strtod(optarg, NULL);
			if ((alpha > 2) || (alpha < 0))
			{
				fprintf(stderr,"ERROR: Invalid alpha. Choose an alpha within [0,2]\n");
				exit(EXIT_FAILURE);
			}
			break;
		case '?':
			fprintf(stderr,"ERROR: Unknown command line argument\n");
			exit(EXIT_FAILURE);
		}
	}



	/* ************************************************************************
	 *                         Set Up Log File
	 *************************************************************************/

	/* Get time in the form of a time_t value */

	time_t rawtime;
	struct tm * timeinfo;

	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	int t_sec   = (*timeinfo).tm_sec;
	int t_min   = (*timeinfo).tm_min;
	int t_hour  = (*timeinfo).tm_hour;
	int t_day   = (*timeinfo).tm_mday;
	int t_month = (*timeinfo).tm_mon;
	int t_year  = (*timeinfo).tm_year + 1900;


	/* Open log file (or use stdout if unspecified) */
	if (logName != NULL)
	{
		printf("Opening log file with name: %s\n",logName);
		logFile = fopen(logName,"w"); // Open log file
	}
	else
	{
		/* Name the log file with the current date & time in file name */
		strftime(strbuf,80,"logSarsa-%Y-%b-%d-%H-%M-%S.txt",timeinfo);
		printf("Creating log file with name: %s\n",strbuf);
		logFile = fopen(strbuf,"w");
	}
	if (logFile == NULL)          // Ensure log file opened properly
	{
		perror("Failed to open log file");
	}

	fprintf(stderr,"[DEBUG] Current local time and date: %s", asctime (timeinfo) );

	/* ********************** Write start of log file ************************/
	/* Date and time of run */
	fprintf(logFile,"#Year=%d Month=%d Day=%d Hour=%d Minute=%d Second=%d\n",t_year,t_month,t_day,t_hour,t_min,t_sec);
	/* Robot, battery and microworld used */
	fprintf(logFile,"#RobotUsed=%s BatteryUsed=%s Microworld=%s\n",
					robotName,batteryName,microworldName);
	/* Cliff Thresholds */
	fprintf(logFile,"#CliffHighValue=%d "
					"CliffLeftThreshold=%d "
					"CliffFrontLeftThreshold=%d "
					"CliffFrontRightThreshold=%d "
					"CliffRightThreshold=%d\n",
					cliffHighValue,
					cliffThresholds[0],
					cliffThresholds[1],
					cliffThresholds[2],
					cliffThresholds[3]);
	fprintf(logFile,"#Algorithm=AverageRewardSarsaReplacing\n");
	fprintf(logFile,"#Alpha=%lf Lambda=%lf Epsilon=%lf Alpha-R=%lf Timestep=%d\n",
					alpha,lambda,epsilon,alphaR,timestep);

	fprintf(logFile,"#Iteration Timestamp Reward  AverageReward\n");
	fflush(logFile);



	/* ************************************************************************
	 *                Set up resources used by program
	 * ***********************************************************************/

	/* Initialize mutex locks */
	pthread_mutex_init(&pktNumMutex, NULL);
	pthread_mutex_init(&serialMutex, NULL);
	pthread_mutex_init(&lastActionMutex, NULL);
	
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

	/* Initialize state-action values and eligibility trace */
	for (i = 0; i < 16; i++)
	{
		for (j = 0; j < 4; j++)
		{
			Q[i][j] = 5 + 0.001*( rand()/((double) RAND_MAX) - 0.5);
			e[i][j] = 0;
		}
	}

	/* Get time just before control loop starts */
	gettimeofday(&timeBegin, NULL);

	/* Get first state-action pair */
	gettimeofday(&timeStart, NULL);
	myPktNum = getPktNum();
	p = (myPktNum + M - 1) % M;
	s = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
	a = epsilonGreedy(Q, s, epsilon);
	takeAction(a);
	ensureTransmitted();
	prevPktNum = myPktNum;

	/* ************************************************************************
	 * Control loop
	 * ***********************************************************************/
	while (TRUE)
	{
		printf("Iteration number: %6d\n",++iteration);
		if (iteration > maxIterations)
		{
			gettimeofday(&timeEnd, NULL);
			fprintf(stderr,"[DEBUG] Maximum iterations reached\n");
			fprintf(stderr,"[DEBUG] Total seconds taken: %lf\n",
							(double)(timeEnd.tv_sec  - timeBegin.tv_sec)
							+((double)(timeEnd.tv_usec - timeBegin.tv_usec))/1000000);
			fprintf(logFile,"#TotalTime=%lf\n",
							(double)(timeEnd.tv_sec  - timeBegin.tv_sec)
							+((double)(timeEnd.tv_usec - timeBegin.tv_usec))/1000000);
			endProgram();
		}
		gettimeofday(&timeEnd, NULL);
		computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
						+ (timeEnd.tv_usec-timeStart.tv_usec);
		printf("Time for iteration (in microseconds): %ld\n", computationTime);
		usleep(timestep - computationTime);
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
			/*printf("deltaT: %f cliff sensors: %u(%u) %u(%u) %u(%u) %u(%u) distance: %hd\n",
					sDeltaT[p%M],
					sCliffL[p%M],sCliffLB[p%M],sCliffFL[p%M],sCliffFLB[p%M],
					sCliffFR[p%M],sCliffFRB[p%M],sCliffR[p%M],sCliffRB[p%M],
					(short) sDistance[p%M]);
					*/

			if (sIRbyte[p%M]==137)
			{
				endProgram();
			}
		}


		/* Sing a song upon accumulating enough reward */
		rewardReport += reward;
		if (rewardReport>SONG_REWARD_THRESHOLD)
		{
			bytes[0] = 141;
			bytes[1] = 0;
			sendBytesToRobot(bytes, 2);
			rewardReport = 0;
		}

		/* Get next state, choose action based on it */
		p = (myPktNum + M - 1) % M;
		sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
		aprime = epsilonGreedy(Q, sprime, epsilon);
		takeAction(aprime);
		ensureTransmitted();

		/* Update action values and  eligibility trace */
		delta = reward - avgReward + Q[sprime][aprime] - Q[s][a];

	    // TODO Make sure this is printing in the correct place
	    //printf("s a r s' a':%d %d %d %d %d\n", s, a, reward, sprime, aprime);
	    //printf("average reward: %f\n", avgReward);
	    //printf("delta: %f\n", delta);

		/* Replacing traces */
		e[s][a] = 1;
	    for (i = 0; i < 16; i++)
	    {
	    	//printf("Action values for state %d: %f %f %f %f\n",i, Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
	    	//printf("Eligibility traces for state %d: %f %f %f %f\n", i, e[i][0], e[i][1], e[i][2], e[i][3]);
	    	for (j = 0; j < 4; j++)
	    	{
	    		Q[i][j] = Q[i][j] + (alpha * delta * e[i][j]);
	    		e[i][j] = lambda*e[i][j];
	    		avgReward += alphaR*delta;
	    	}
	    }

		/* Log reward, timestep, and iteration */
		fprintf(logFile,"%5d %d.%d %d %6.12lf\n",
						iteration,
						(int)timeStart.tv_sec,
						(int)timeStart.tv_usec,
						reward,
						avgReward);

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


int epsilonGreedy(double Q[16][4], int s, double epsilon)
{
	int max, i;
	int myPktNum, p;
	int firstAction, lastAction;

	myPktNum = getPktNum();
	p = (myPktNum + M - 1) % M;
	firstAction = sCliffFLB[p] || sCliffFRB[p];
	if (sCliffLB[p] || sCliffRB[p])
	{
		lastAction = 2;
	}
	else
	{
		lastAction = 3;
	}

	if (rand()/((double)RAND_MAX+1) < epsilon)
	{
		//TODO What is going on here?
		return firstAction + rand()%(lastAction + 1 - firstAction);
	}
	else
	{
		max = lastAction;
		for (i = firstAction; i < lastAction; i++)
		{
			if (Q[s][i] > Q[s][max]) max = i;
		}
    return max;
  }
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


