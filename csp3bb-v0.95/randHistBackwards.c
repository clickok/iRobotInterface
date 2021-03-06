/* AvgRewardSarsaReplacing.c
   Author: Rupam
   Modified by Brendan for experiments with modification of parameters.

   Modified further to log communication with the robots for testing
   purposes.
 */

#include <errno.h>
#include <fcntl.h>
#include <float.h>
#include <getopt.h>
#include <limits.h>
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

/*****************************************************************************
 *                        iRobot Create Parameters
 *****************************************************************************/

/* The speed at which the Create drives its wheels */
#define SPEED_1 100
#define SPEED_2 300

/* The reward threshold for the Create to "sing"   */
#define SONG_REWARD_THRESHOLD  100

 /*****************************************************************************
 *                        State Space Parameters
 *****************************************************************************/

#define S_DEPTH   2
#define N_STATES  4096
#define N_ACTS    4

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
pthread_mutex_t pktNumMutex, serialMutex, lastActionMutex, endFlagMutex, logFileMutex; // locks
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
int  epsilonGreedy(double Q[N_STATES][N_ACTS], int s, double epsilon);
void takeAction(int action);
void driveWheels(int left, int right);
void sendBytesToRobot(ubyte* bytes, int numBytes);
void ensureTransmitted();
int  getPktNum();
void endProgram();

void array2dPrint(size_t s1, size_t s2, double *array);

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
	int maxIterations = INT_MAX;            // Limit for number of iterations
	int history[S_DEPTH] = {0};             // Observation history
	int hSum, hIndex;						// For computing state via history
	double hRand = 0.9;						// Odds of including obsv in history
	int hOffset = 0;						// For keeping track of history array
	double Q[N_STATES][N_ACTS];				// State-Action value array
	double e[N_STATES][N_ACTS];				// Eligibility trace array
	double alpha = 0.2;						// Stepsize (alpha) parameter
	double beta = 0.002;                    // Average Reward Stepsize
	double lambda = 0.9;					// Trace decay parameter
	double epsilon = 0.01;                  // Exploration parameter
	int timestep = 100000;                  // Timestep in microseconds
	int obsv;                               // Observation
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
	long computationTime = 0; 				// Timing related
	char * portName = NULL;                 // Name of serial port

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
			{"alpha",           required_argument,   0, 'a'},
			{"beta",            required_argument,   0, 'b'},
			{"epsilon",         required_argument,   0, 'e'},
			{"lambda",          required_argument,   0, 'l'},
			{"timestep",        required_argument,   0, 't'},
			{"iterations",      required_argument,   0, 'i'},
			{"chance", 			required_argument,   0, 'c'},
			{"help",            no_argument,         0, 'h'},
			{0, 0, 0, 0}
		};
		int c;
		int option_index = 0;

		//TODO Alphabetize the command line arguments
		c = getopt_long(argc, argv, "p:a:b:t:i:e:l:c:",long_options, &option_index);
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
		case 'a':
			alpha = strtod(optarg, NULL);
			if ((alpha > 2) || (alpha < 0))
			{
				fprintf(stderr,"ERROR: Invalid alpha. Choose an alpha within [0,2]\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'c':
			hRand = strtod(optarg, NULL);
			if ((hRand < 0) || (hRand > 1))
			{
				fprintf(stderr, "ERROR: Invalid value for chance, choose within [0,1]\n");
				exit(EXIT_FAILURE);
			}
		case 'b':
			beta = strtod(optarg, NULL);
			if ((beta > 1 ) || (beta < 0))
			{
				fprintf(stderr, "ERROR: Invalid beta. Choose a beta within [0,1]\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'e':
			epsilon = strtod(optarg, NULL);
			if ((epsilon > 1) || (epsilon < 0))
			{
				fprintf(stderr,"ERROR: Invalid epsilon. Choose an epsilon within [0,1]\n");
				exit(EXIT_FAILURE);
			}
			break;
		case 'i':
			maxIterations = strtol(optarg, (char **) NULL, 10);
			break;
		case 'l':
			lambda = strtod(optarg, NULL);
			if ((lambda > 1) || (lambda < 0))
			{
				fprintf(stderr,"ERROR: Invalid lambda. Choose a lambda within [0,1]\n");
				exit(EXIT_FAILURE);
			}
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
	pthread_mutex_init(&logFileMutex, NULL);
	
	/* Install signal handler */
	struct sigaction act;                   // The new sigaction
	struct sigaction oldact;                // To preserve old sigaction
	act.sa_handler = endProgram;            // Set endProgram() as sig handler
	sigemptyset(&act.sa_mask);              // Empty act's signal set
	act.sa_flags = 0;                       // Set act's flags to 0
	sigaction(SIGINT, &act, &oldact);       // Set act to respond to SIGINT


	/* Set up serial port and CSP3 thread, begin receiving data */

	if (portName != NULL) setupSerialPort(portName);
	usleep(20000); // wait for at least one packet to have arrived

	if (0 != pthread_create(&tid, NULL, (void *) &csp3, NULL))
	{
		perror("Could not create thread");
		exit(EXIT_FAILURE);
	}

	/* Initialize state-action values and eligibility trace */
	for (i = 0; i < N_STATES; i++)
	{
		for (j = 0; j < N_ACTS; j++)
		{
			Q[i][j] = 5 + 0.00001*( rand()/((double) RAND_MAX) - 0.5);
			e[i][j] = 0;
		}
	}

	/* Get time just before control loop starts */
	gettimeofday(&timeBegin, NULL);

	/* Get first state-action pair */
	gettimeofday(&timeStart, NULL);
	myPktNum = getPktNum();
	p = (myPktNum + M - 1) % M;
	obsv = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
	s = obsv;
	a = epsilonGreedy(Q, s, epsilon);
	takeAction(a);
	ensureTransmitted();
	prevPktNum = myPktNum;

	/*  ----------------------- Print Setup Information --------------------- */
	printf("alpha: %lf beta: %lf \t epsilon: %lf \t lambda: %lf \n",
	 	   alpha, beta, epsilon, lambda);

	/* ************************************************************************
	 * Control loop
	 * ***********************************************************************/
	while (TRUE)
	{
		// Check if maximum number of iterations has been reached
		if (iteration > maxIterations)
		{
			gettimeofday(&timeEnd, NULL);
			fprintf(stderr,"[DEBUG] Maximum iterations reached\n");
			fprintf(stderr,"[DEBUG] Total seconds taken: %lf\n",
							(double)(timeEnd.tv_sec  - timeBegin.tv_sec)
							+((double)(timeEnd.tv_usec - timeBegin.tv_usec))/1000000);
			fprintf(stderr,"[DEBUG] Final PktNum Value: %d\n",getPktNum());
			pthread_mutex_lock(&logFileMutex);
			// REMOVED
			pthread_mutex_unlock(&logFileMutex);
			endProgram();
		}

		// Print iteration data every so often to indicate progress
		if (((iteration % 1) == 0))
		{
			printf("Iteration number: %6d\n",iteration);
			printf("Time for iteration (in microseconds): %ld\n", computationTime);
			printf("action: %d\n", a);
		}
		++iteration;

		// for (i = 0; i < N_STATES; i++)
		// {
		// 	for (j = 0; j < N_ACTS; j++)
		// 	{
		// 		printf("%2.6lf ", Q[i][j]);
		// 	}
		// 	printf("\n");
		// }

		gettimeofday(&timeEnd, NULL);

		/*------------------- Code is not timed within! ------------------*/
		computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
						+ (timeEnd.tv_usec-timeStart.tv_usec);

		//TODO Currently experiencing timing "drift", investigate fixes
		if ((timestep - computationTime) < 0)
		{
			fprintf(stderr,"[ERROR]: Computation time exceeded timestep: "
					"Iteration = %d, "
					"Time = %ld.%ld, "
					"computationTime = %ld\n",
					iteration, (long int)timeEnd.tv_sec, (long int)timeEnd.tv_usec,computationTime);

			//TODO How best to handle situation where timestep has exceeded computationTime?
		}
		else
		{
			usleep(timestep - computationTime);
		}
		/*--------------------- End Non-Timed Block ----------------------*/
		// Begin timing AFTER usleep()
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
			reward -= sDistance[p%M];
			if (sIRbyte[p%M]==137)
			{
				endProgram();
			}
		}
		// Weaken the negativity associated with going forward
		if (reward < 0)
		{
			reward = -1;
		}


		/* Apply "bonuses" for other sensory data */
		//If one of the front sensors is off, but not both
		// p = (myPktNum + M - 1) % M;
		// if ((sCliffFLB[p] || sCliffFRB[p]) && !(sCliffFLB[p] && sCliffFRB[p]))
		// {
		// 	reward += 5;
		// 	printf("Sensor bonus\n");
		// }

		// Punish the robot for having most sensors off the world
		if ((sCliffLB[p] && sCliffRB[p]) && (sCliffFLB[p] || sCliffFRB[p]))
		{
			reward -= 20;
			printf("Not on world!\n");
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


		/*********************************************************************
								History and State Update
		**********************************************************************/

		/* Get next state, choose action based on it */
		p = (myPktNum + M - 1) % M;
		obsv = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
		
		if ( (((double)rand()) / ((double)RAND_MAX) ) < hRand)
		{
			history[(hOffset % S_DEPTH)] = obsv;
			hOffset += 1;
		}


		printf("Observation: %d\n", obsv);
		printf("hOffset: %d\n", hOffset);
		hSum = 0;
		for (i=0; i< S_DEPTH; i++)
		{
			hIndex = (hOffset - i + S_DEPTH) % S_DEPTH;
			printf("history[%d] = %d --> %d\n", hIndex, history[hIndex], ((history[hIndex]) << (4*(i+1))));
			hSum   += ((history[hIndex]) << (4*(i+1)));
		}
		printf("hSum: %d\n", hSum);

		hSum  += obsv;
		sprime = (hSum % N_STATES);
		aprime = epsilonGreedy(Q, sprime, epsilon);

		takeAction(aprime);
		ensureTransmitted();

		/* Update action values and  eligibility trace */
		delta = reward - avgReward + Q[sprime][aprime] - Q[s][a];
		avgReward += beta * delta;
		
		// DEBUG: Print the update algorithm's values
		printf("S: %d\t A: %d\t R: %d\t S': %d\t A': %d\n", s, a, reward, sprime, aprime);
		printf("delta: %lf \t avgReward: %lf\n", delta, avgReward);
		
		/* Replacing traces */
		e[s][a] = 1;
	    for (i = 0; i < N_STATES; i++)
	    {
	    	for (j = 0; j < N_ACTS; j++)
	    	{
	    		Q[i][j] = Q[i][j] + (alpha * delta * e[i][j]);
	    		e[i][j] = lambda*e[i][j];
	    	}
	    }

		/* Log reward, timestep, and iteration to file*/
	    pthread_mutex_lock(&logFileMutex);
	    // Removed
		pthread_mutex_unlock(&logFileMutex);

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


void reflexes() 
{
 	int a;
	int p = pktNum % M;
	pthread_mutex_lock( &lastActionMutex );
	a = lastAction;
	pthread_mutex_unlock( &lastActionMutex );
	// Determine if robot is trying to go off in such a way that NO 
	// sensors would be left behind...

	if ((((a % 4) ==0) && (sCliffFLB[p] && sCliffFRB[p])) ||   // attempt to go forward over cliff
       (((a % 4) ==3)  && (sCliffLB[p]  && sCliffRB[p])))      // attempt to go backward over cliff
    {
    	driveWheels(0, 0);                            // then interrupt motion
  	}

  	// Consider adding something to protect it if wheel drop sensors activate

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
    case 0  : driveWheels( SPEED_1,  SPEED_1);  break;   // forward
    case 1  : driveWheels( -SPEED_1, SPEED_1);  break;   // left
    case 2  : driveWheels( SPEED_1, -SPEED_1);  break;   // right
    case 3  : driveWheels( -SPEED_1, -SPEED_1); break;   // backward

    case 4  : driveWheels( SPEED_2,  SPEED_2);  break;   // 2nd set of actions
    case 5  : driveWheels( -SPEED_2, SPEED_2);  break; 
    case 6  : driveWheels( SPEED_2,  -SPEED_2); break;
    case 7  : driveWheels( -SPEED_2, -SPEED_2); break;

    case -1 : driveWheels(0, 0);                break;   // Stop
    default : printf("Bad action\n");
    }
}


int epsilonGreedy(double Q[N_STATES][N_ACTS], int s, double epsilon)
{
	int max, i;
	int myPktNum, p;
	int offBack, offFront; // Determine if off the back or front

	// Take a random action with probability epsilon
	if (rand()/((double)RAND_MAX+1) < epsilon)
	{
		return (rand() % N_ACTS);
	}

	myPktNum = getPktNum();
	p        = (myPktNum + M - 1) % M;
	offFront = (sCliffFLB[p] || sCliffFRB[p]);
	offBack  = (sCliffLB[p] || sCliffRB[p]);
	
	max = lastAction;
	for (i = 0; i < N_ACTS; i++)
	{
		// Avoid considering forbidden actions
		if (offFront && (i % 4 == 0)) continue;
		if (offBack  && (i % 4 == 3)) continue;
		if (Q[s][i] > Q[s][max]) max = i;
	}
    return max;
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
					fprintf(stderr,"Misaligned packet\n");
					for (i = 1; i<B; i++) packet[i-1] = packet[i];
					numBytesPreviouslyRead--;
				}
			}
		}
	}
}


