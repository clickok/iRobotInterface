/*
Implementation of SARSA algorithm, but with modifications to allow 
the robot to follow the edge going backwards.
*/

#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/select.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <float.h>
#include <signal.h>

#define FALSE 0
#define TRUE 1

typedef unsigned char ubyte;

//------------------------------------------------------------------
// ---------------          Create codes           -----------------
//------------------------------------------------------------------

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

//------------------------------------------------------------------
// ---------          Global Names and Variables           ---------
//------------------------------------------------------------------
#define SPEED 300
unsigned int pktNum = 0;      // Number of the packet currently being constructed by csp3
pthread_mutex_t pktNumMutex, actionMutex, rewardMusicMutex; // locks
int action = 0;           // current action selected by agent (initially forward)
struct timeval lastPktTime;   // time of last packet
int rewardMusic = 0;
int fd = 0;                   // file descriptor for serial port
#define B 20                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

//sensory arrays:
#define M 1000
unsigned short  sCliffL[M], sCliffR[M], sCliffFL[M], sCliffFR[M]; // small pos integers
ubyte  sCliffLB[M], sCliffRB[M], sCliffFLB[M], sCliffFRB[M];      // binary 1/0
short  sDistance[M];          // wheel rotation counts (small integers, pos/neg)
double sDeltaT[M];            // in milliseconds
ubyte sIRbyte[M];             // Infrared byte e.g. remote
ubyte sDrive[M];              // Drive command in {0, 1, 2, 3, 4}

int cliffThresholds[4];       // left, front left, front right, right
int cliffHighValue;           // binary value taken if threshold exceeded
//------------------------------------------------------------------

void setupSerialPort(char serialPortName[]);
void* csp3(void *arg);void loadCliffThresholds();
void takeAction(int action);
int epsilonGreedy(double Q[16][4], int s, double epsilon);
void endProgram();
void driveWheels(int left, int right);
void sendBytesToRobot(ubyte* bytes, int numBytes);
void ensureTransmitted();
int getPktNum();

int main(int argc, char *argv[]) {
  pthread_t tid;
  int t_err;
  unsigned int prevPktNum;
  unsigned int myPktNum;
  int p, pn;
  double Q[16][4], e[16][4];
  double stepsize = 0.1, lambda = 0.9, gamma = 0.98, epsilon = 0.01;
  int a, aprime;
  int s, sprime;
  int reward;
  int i, j;
  double delta;
  int rewardReport;
  struct timeval timeStart, timeEnd, incrementBy;
  long computationTime;
  struct sigaction act;
  struct sigaction oldact;

  act.sa_handler = endProgram;
  sigemptyset(&act.sa_mask);
  act.sa_flags = 0;
  sigaction(SIGINT, &act, &oldact);
  if (argc < 2) {
    fprintf(stderr, "Portname argument required -- something like /dev/tty.usbserial\n");
    return 0;
  }
  loadCliffThresholds();
  srand(0);
  pthread_mutex_init(&pktNumMutex, NULL);
  pthread_mutex_init(&actionMutex, NULL);
  pthread_mutex_init(&rewardMusicMutex, NULL);

  setupSerialPort(argv[1]);
  usleep(20000); // wait for at least one packet to have arrived
  t_err = pthread_create(&tid, NULL, &csp3, NULL);
  if (t_err!=0) {
    fprintf(stderr, "\ncan't create thread: [%s]", strerror(t_err));
    exit(EXIT_FAILURE);
  }
  prevPktNum = 0;

  // initialize Q
  for (i = 0; i < 16; i++)
    for (j = 0; j < 4; j++) {
      Q[i][j] = 5 + 0.001*( rand()/((double) RAND_MAX) - 0.5);
      e[i][j] = 0;
    }
  gettimeofday(&timeStart, NULL);
  myPktNum = getPktNum();
  p = (myPktNum + M - 1) % M;
  s = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
  a = epsilonGreedy(Q, s, epsilon);
  pthread_mutex_lock( &actionMutex );
  action = a; // sets up action to be taken by csp thread
  pthread_mutex_unlock( &actionMutex );  
  prevPktNum = myPktNum;
  rewardReport = 0;
  while (TRUE) { // main agent loop
    gettimeofday(&timeEnd, NULL);
    computationTime = (timeEnd.tv_sec-timeStart.tv_sec)*1000000
      + (timeEnd.tv_usec-timeStart.tv_usec);
    printf("Time for iteration (in microseconds): %ld\n", computationTime);
    if (100000 - computationTime > 0) usleep(100000 - computationTime);
    else printf("This iteration took too long!\n\n");
    incrementBy.tv_sec = 0;
    incrementBy.tv_usec = 100000;
    timeradd(&timeStart, &incrementBy, &timeStart);
    myPktNum = getPktNum();
    if (myPktNum - prevPktNum > M) {
      fprintf(stderr, "Buffer overflow!\n");
      exit(EXIT_FAILURE);
    }
    reward = 0;
    for (pn = prevPktNum; pn < myPktNum; pn++) {
      p = pn % M;
      reward -= sDistance[p];
      printf("deltaT: %f cliff sensors: %u(%u) %u(%u) %u(%u) %u(%u) distance: %hd\n",
	     sDeltaT[p],
	     sCliffL[p],sCliffLB[p],sCliffFL[p],sCliffFLB[p],
	     sCliffFR[p],sCliffFRB[p],sCliffR[p],sCliffRB[p],
	     (short) sDistance[p]);
      if (sIRbyte[p]==137) endProgram(); // quit on remote pause
    }
    rewardReport += reward;
    if (rewardReport > 50) {
      pthread_mutex_lock( &rewardMusicMutex );
      rewardMusic = 1;
      pthread_mutex_unlock( &rewardMusicMutex );    
      rewardReport -= 50;
    }
    p = (myPktNum - 1) % M;
    sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
    aprime = epsilonGreedy(Q, sprime, epsilon);
    pthread_mutex_lock( &actionMutex );
    action = aprime; // sets up action to be taken by csp thread
    pthread_mutex_unlock( &actionMutex );    
    delta = reward + gamma*Q[sprime][aprime] - Q[s][a];
    for (j = 0; j < 4; j++)
      e[s][j] = 0;
    e[s][a] = 1;
    printf("s a r s' a':%d %d %d %d %d\n", s, a, reward, sprime, aprime);
    for (i = 0; i < 16; i++) {
      printf("Action values for state %d: %f %f %f %f\n",i, Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
      printf("Eligibility traces for state %d: %f %f %f %f\n", i, e[i][0], e[i][1], e[i][2], e[i][3]);
      for (j = 0; j < 4; j++) {
        Q[i][j] = Q[i][j] + stepsize*delta*e[i][j];
        e[i][j] = gamma*lambda*e[i][j];
      }
    }
    s = sprime;
    a = aprime;
    prevPktNum = myPktNum;
  }
  return 0;
}

int getPktNum() {
  int myPktNum;
  pthread_mutex_lock( &pktNumMutex );
  myPktNum = pktNum;
  pthread_mutex_unlock( &pktNumMutex );
  return myPktNum;  
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

int epsilonGreedy(double Q[16][4], int s, double epsilon)
{
  int max, i, p;
  int firstAction, lastAction;

  p = (getPktNum() + M - 1) % M;
  firstAction = sCliffFLB[p] || sCliffFRB[p];
  if (sCliffLB[p] || sCliffRB[p]) lastAction = 2;
  else lastAction = 3;
  if (rand()/((double)RAND_MAX+1) < epsilon) {
    printf("random action\n\n");
    return firstAction + rand()%(lastAction + 1 - firstAction);
  } else {
    max = lastAction;
    for (i = firstAction; i < lastAction; i++)
      if (Q[s][i] > Q[s][max])
        max = i;
    return max;
  }
}

void takeAction(int action) {
    switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(-SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, -SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    case 4  : driveWheels(0, 0); break;            // stop
    default : printf("Bad action\n");
    }
}

void endProgram() {
  ubyte bytes[2];
  printf("Ending Program\n");
  driveWheels(0, 0);
  // pause streaming
  bytes[0] = CREATE_STREAM_PAUSE;
  bytes[1] = 0;
  sendBytesToRobot(bytes, 2);
  tcdrain(fd);
  exit(EXIT_SUCCESS);
}

void sendBytesToRobot(ubyte* bytes, int numBytes) {
  int ret;
  if ((ret=write(fd, bytes, numBytes))==-1) {
    fprintf(stderr, "Problem with write(): %s\n", strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void ensureTransmitted() {
  int ret;
  if ((ret=tcdrain(fd))==-1) {
	fprintf(stderr, "Problem with tcdrain(): %s\n", strerror(errno));
	exit(EXIT_FAILURE);
  }
}

#define MAX(a,b) (a > b?a:b)
#define MIN(a,b) (a < b?a:b)

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

void reflexes();

void* csp3(void *arg) {
  int errorCode, numBytesRead, i;
  ubyte bytes[B];
  int numBytesPreviouslyRead = 0;
  struct timeval timeout;
  fd_set readfs;

  gettimeofday(&lastPktTime, NULL);
  FD_SET(fd, &readfs);

  while (TRUE) {
    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    errorCode = select(fd+1, &readfs, NULL, NULL, &timeout);
    if (errorCode==0) {
      printf("Timed out at select()\n");
    } else if (errorCode==-1) {
      fprintf(stderr, "Problem with select(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    }
    numBytesRead = read(fd, &bytes, B-numBytesPreviouslyRead);
    if (numBytesRead==-1) {
      fprintf(stderr, "Problem with read(): %s\n", strerror(errno));
      exit(EXIT_FAILURE);
    } else {
      for (i = 0; i < numBytesRead; i++) packet[numBytesPreviouslyRead+i] = bytes[i];
      numBytesPreviouslyRead += numBytesRead;
      if (numBytesPreviouslyRead==B) {  //packet complete!
	if (checkPacket()) {
	  extractPacket();
	  reflexes();
	  ensureTransmitted();
	  pthread_mutex_lock( &pktNumMutex );
	  pktNum++;
	  pthread_mutex_unlock( &pktNumMutex );
	  numBytesPreviouslyRead = 0;
	} else {
	  printf("misaligned packet.\n");
	  for (i = 1; i<B; i++) packet[i-1] = packet[i];
	  numBytesPreviouslyRead--;
	}
      }
    }
  }
  return NULL;
}

void reflexes() {
  int p = pktNum%M;
  int myRewardMusic;
  pthread_mutex_lock( &actionMutex );
  sDrive[p] = action;
  pthread_mutex_unlock( &actionMutex );
  if ((sDrive[p]==0 && (sCliffFLB[p] || sCliffFRB[p])) || // if forward over cliff
      (sDrive[p]==3 && (sCliffLB[p] || sCliffRB[p])))    // or backward over cliff
    sDrive[p] = 4;                            // then stop instead
  takeAction(sDrive[p]);

  ubyte bytes[2];
  ubyte frontbit = sCliffFLB[p] || sCliffFRB[p];
  ubyte ledbits = (sCliffLB[p] << 2) | (frontbit << 1) | sCliffRB[p];
  bytes[0] = CREATE_DIGITAL_OUTS;
  bytes[1] = ledbits;
  sendBytesToRobot(bytes, 2);

  pthread_mutex_lock( &rewardMusicMutex );
  myRewardMusic = rewardMusic;
  pthread_mutex_unlock( &rewardMusicMutex );

  if (myRewardMusic == 1) {
    bytes[0] = 141;
    bytes[1] = 0;
    sendBytesToRobot(bytes, 2);
    pthread_mutex_lock( &rewardMusicMutex );
    rewardMusic = 0;
    pthread_mutex_unlock( &rewardMusicMutex );    
  }
}



