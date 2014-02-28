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
#define CHECK_BIT(var, pos) (((var) & (1 <<(pos))) == 0)
#define MAX(a,b) (a > b?a:b)
#define MIN(a,b) (a < b?a:b)

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
#define SPEED 50
#define TURN_SPEED -50
#define sDepth 10
#define aDepth 10

// If you modify this, you may have to ensure thread safety
int policyMode = 0;           // The current mode of the robot's policy
int policyStep = 0;           // The policy's current step in its execution

unsigned int pktNum = 0;      // Number of the packet currently being constructed by csp3
pthread_mutex_t pktNumMutex, actionMutex, rewardMusicMutex; // locks
int action = 0;           // current action selected by agent (initially forward)
struct timeval lastPktTime;   // time of last packet
int rewardMusic = 0;
int fd = 0;                   // file descriptor for serial port
#define B 20                  // number of bytes in a packet
ubyte packet[B];              // packet is constructed here

// state & action history

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
int customPolicy(int s);
void endProgram();
void driveWheels(int left, int right);
void sendBytesToRobot(ubyte* bytes, int numBytes);
void ensureTransmitted();
int getPktNum();
void reflexes();


// My additional functions
int customPolicy(int s);
int randomAction(int defaultAction, double randProb);
int lastGoodState(int state, int curPkt);
int shouldSwitch(int curPkt);
void printLastPackets(int n);

int main(int argc, char *argv[]) {
  pthread_t tid;
  int t_err;
  
  unsigned int prevPktNum;
  unsigned int myPktNum;
  int p, pn;
  

  double Q[16][4], e[16][4];
  double stepsize = 0.1, lambda = 0.9, gamma = 0.98;
  int a, aprime;
  int s, sprime;
  int reward;
  
  int i, j;
  int iterCount = 0;
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
  //a = epsilonGreedy(Q, s, epsilon);

  // Perform action according to custom policy instead
  a = customPolicy(s); 

  pthread_mutex_lock( &actionMutex );
  action = a; // sets up action to be taken by csp thread
  pthread_mutex_unlock( &actionMutex ); 

  prevPktNum = myPktNum;
  rewardReport = 0;
  while (TRUE) { // main agent loop
    iterCount++;
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
    printf("prevPktNum: %5d \t myPktNum: %5d\n", prevPktNum, myPktNum);
    for (pn = prevPktNum; pn < myPktNum; pn++) 
    {
        p = pn % M;
        reward -= sDistance[p];
        printf("packet: %5d deltaT: %f cliff sensors: %u(%u) %u(%u) %u(%u) %u(%u) distance: %hd\n",
        p,
	       sDeltaT[p],
  	     sCliffL[p],sCliffLB[p],sCliffFL[p],sCliffFLB[p],
  	     sCliffFR[p],sCliffFRB[p],sCliffR[p],sCliffRB[p],
  	     (short) sDistance[p]);

        if (sIRbyte[p]==137) endProgram(); // quit on remote pause
    }
    // Modify negative reward (so it's not gigantic) for going forwards
    if (reward < 5)
    {
      reward = -1;
    }
    // Add a negative reward for being off the edge
    reward -= 30 * (sCliffLB[p] | sCliffFLB[p]) + (sCliffFRB[p] | sCliffRB[p]);
    printf("Cliff Sensors: %u \t %u \t %u \t %u\n", sCliffLB[p], sCliffFLB[p], sCliffFRB[p], sCliffRB[p]);

    rewardReport += reward;
    if (rewardReport > 50) {
      pthread_mutex_lock( &rewardMusicMutex );
      rewardMusic = 1;
      pthread_mutex_unlock( &rewardMusicMutex );    
      rewardReport -= 50;
    }

    p = (myPktNum - 1) % M;
    sprime = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];
    //aprime = epsilonGreedy(Q, sprime, epsilon);
    aprime = customPolicy(sprime);

    pthread_mutex_lock( &actionMutex );
    action = aprime; // sets up action to be taken by csp thread
    pthread_mutex_unlock( &actionMutex );    

    delta = reward + gamma*Q[sprime][aprime] - Q[s][a];
    
    for (j = 0; j < 4; j++)
      e[s][j] = 0;
    e[s][a] = 1;
    printf("s a r s' a': %d \t %d \t %d \t %d \t %d\n", s, a, reward, sprime, aprime);
    for (i = 0; i < 16; i++) {
      //printf("Action values for state %d: %f %f %f %f\n",i, Q[i][0], Q[i][1], Q[i][2], Q[i][3]);
      //printf("Eligibility traces for state %d: %f %f %f %f\n", i, e[i][0], e[i][1], e[i][2], e[i][3]);
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


int randomAction(int defaultAction, double randProb)
{
  if ((rand() / (double)(RAND_MAX)) < randProb)
  {
    return (rand() % 4);
  }
  else
  {
    return defaultAction;
  }
  fprintf(stderr, "Error: randomAction() reached end of function unexpectedly\n");
  return -1;
}

int customPolicy(int s)
{

  // Store the values for the bumpers, whether on (1) or off (0)
  int LB_ON, FLB_ON, FRB_ON, RB_ON;
  int LB_OFF, FLB_OFF, FRB_OFF, RB_OFF;

  // Have a custom action to return after going through the policy
  int customAction;
  int FORWARD = 0, LEFT = 1, RIGHT = 2, BACKWARD = 3, STOP =4;

  // For determining how far back a certain state was
  int searchDepth;
  int onWorldState = 0;

  // Determine whether the bumpers are "on" the allowed terrain or "off"
  // State information might be out of date when passed to this file, so use
  // the newest packet instead?
  // Should this bother with getting the packet number at all?
  // Possibly would prefer to have state information in terms of a struct...
  int p;
  p = ((getPktNum() + M - 1) % M);

  int tmp = 0;
  tmp = (sCliffLB[p]<<3) | (sCliffFLB[p]<<2) | (sCliffFRB[p]<<1) | sCliffRB[p];

  LB_ON   = CHECK_BIT(tmp, 3);
  FLB_ON  = CHECK_BIT(tmp, 2);
  FRB_ON  = CHECK_BIT(tmp, 1);
  RB_ON   = CHECK_BIT(tmp, 0);

  LB_OFF  = (LB_ON  == FALSE);
  FLB_OFF = (FLB_ON == FALSE);
  FRB_OFF = (FRB_ON == FALSE);
  RB_OFF  = (RB_ON  == FALSE);

  // The actual code  ---------------------------------------------------------------- //
  printf("policyMode: %4d \t policyStep: %4d \n", policyMode, policyStep);
  printf("currentState: %d\n", tmp);
  printf("ON:  \tLB: %d \t FLB: %d \t FRB: %d \t RB: %d \n", LB_ON, FLB_ON, FRB_ON, RB_ON);
  printf("OFF: \tLB: %d \t FLB: %d \t FRB: %d \t RB: %d \n", LB_OFF, FLB_OFF, FRB_OFF, RB_OFF);
  

  // If in line following mode
  if (policyMode == 0)
  {
    if (FLB_ON && FRB_OFF)
    {
      customAction = BACKWARD;
    }
    else if (FLB_OFF && FRB_OFF)
    {
      if (policyStep < 2)
      {
        customAction = RIGHT;
        policyStep ++;
      }
      else
      {
        searchDepth = lastGoodState(onWorldState, p);
        printf("lastGoodState was at packet: %d\n", searchDepth);
        customAction = STOP;
      }
    }
    else if (FLB_ON && FRB_ON)
    {
      customAction = LEFT;
    }
    else
    {
      customAction = STOP;
    }
  }
  else if (policyMode == 1) // Attempt to find the world again
  {
    // NEED TO KNOW when the last good state was, what the history has been
    // up until that point, in order to calculate an inverse
    customAction = STOP;
  }
  else if (policyMode == 2) // The turning phase of the policy
  {
    customAction == STOP;

  }

  printf("customAction: %d\n", customAction);
  return customAction;
}

int lastGoodState(int state, int curPkt)
{
  int i;
  int tmpState;
  for(i = curPkt; i > 0; i--)
  {
    // Figure out what the state was at that point in time
    tmpState = ((sCliffLB[i]<<3) | (sCliffFLB[i]<<2) | (sCliffFRB[i]<<1) | (sCliffRB[i]));
    if (tmpState == state)
    {
      return i;
    }
  }
  // If cannot find the last good state, return -1
  fprintf(stderr, "lastGoodState() could not find state %d\n before packet:%d", state, curPkt);
  return -1;
}

int shouldSwitch(int curPkt)
{
  // int i;
  policyMode = 1;
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

void takeAction(int action) {
    /*switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(-SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, -SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    case 4  : driveWheels(0, 0); break;            // stop
    default : printf("Bad action\n");
    }
*/
    switch (action) {
    case 0  : driveWheels(SPEED, SPEED); break;    // forward
    case 1  : driveWheels(TURN_SPEED, SPEED); break;   // left
    case 2  : driveWheels(SPEED, TURN_SPEED); break;   // right
    case 3  : driveWheels(-SPEED, -SPEED); break;  // backward
    case 4  : driveWheels(0, 0); break;            // stop
    default : printf("Bad action\n");
    }
}

void printLastPackets(n)
{
  int i, p;
  p = getPktNum();
  for(i=0; i<n; i++)
  {
    printf("%7d\t %u %u %u %u %u %u %u %u", 
      (p -i), 
      sCliffL[p],sCliffLB[p],sCliffFL[p],sCliffFLB[p],
      sCliffFR[p],sCliffFRB[p],sCliffR[p],sCliffRB[p]);
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


void * csp3(void *arg) 
{
    int errorCode, numBytesRead, i;
    ubyte bytes[B];
    int numBytesPreviouslyRead = 0;
    struct timeval timeout;
    fd_set readfs;

    gettimeofday(&lastPktTime, NULL);
    FD_SET(fd, &readfs);

    while (TRUE) 
    {
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
      if (numBytesPreviouslyRead==B)  //packet complete!
      {
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
            for (i = 1; i<B; i++) packet[i-1] = packet[i];
            {
              numBytesPreviouslyRead--;
            }
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
  // If forward over the cliff, stop instead
  /*
  if ((sDrive[p]==0 && (sCliffFLB[p] || sCliffFRB[p]))) // if forward over cliff
  {
      sDrive[p] = 4;
  }
  else if (sDrive[p]==3 && (sCliffFLB[p] || sCliffFRB[p])) // or backward over cliff
  {
      sDrive[p] = 4;
  }
  else if (sDrive[p]==3 && (sCliffLB[p] || sCliffRB[p])) // or backward over cliff
  {
      sDrive[p] = 4;
  }*/


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



