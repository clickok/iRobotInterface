/******************************************************************************
 * calibrate.c
 * Created on: Jun 10, 2013
 * Author: BB
 *
 * A currently NON-FUNCTIONAL calibration program for the iRobot Create's
 * sensors, in order to ensure that the robot can distingush between the
 * different regions it might be placed in.
 * 
 * The "call-and-response" code isn't working, and it isn't clear why this is
 * the case. A similar program by Rich (using the "stream" mode) does work,
 * so this code is only kept for reference.
 * 
 *****************************************************************************/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#define BUFFER_LENGTH 40
#define NUM_SAMPLES   1
#define DELAY_SEND 1000

int connectDevice(char * device);
int disconnectDevice(int fd);
int askForSensors(int fd, int * array);
int sendByte(int fd, unsigned char byte);

static int fd;

int main(int argc, char * argv[])
{
	int i,ret;
	int sensors[10];
	char line[BUFFER_LENGTH];
	char cmd[8];
	char * p;
	/* Commence awesome, manually-entered explanation! */
	printf("******************** iRobot Sensor Calibration **********************\n");
	printf( "* How this works:                                                   *\n"
			"* 1) The program will attempt to connect to the device specified by *\n"
			"*    the command line argument... which you should've entered...    *\n"
			"* 2) Place Robot in the 'inner' region. The program will read the   *\n"
			"*    infrared sensor values. When prompted, move the robot (while   *\n"
			"*    keeping it in the same region) so it can collect an average.   *\n"
			"* 3) When prompted, move the robot to the 'outer' region, and       *\n"
			"*    follow the same process as in Step (1).                        *\n"
			"* 4) The program will then compute an average for the two regions   *\n"
			"*    (allowing for some margin to account for variability) and save *\n"
			"*    the result as 'calibration.dat', or tell you if it's not       *\n"
			"*    possible to distinguish the two regions.                       *\n"
			"*********************************************************************\n"
			);
	printf("\nSTEP 1)\n"); /* Cut a hole in a box */
	fd = connectDevice(argv[1]);
	if (fd > -1) printf("Connected to device with file descriptor: %d\n",fd);
	printf("STEP 2)\n");
	printf("Place Robot in region one.\n");
	for(i=0;i<NUM_SAMPLES;++i)
	{
		printf("Collecting sample %d of %d\n",(i+1),NUM_SAMPLES);
		printf("When ready to sample, press <return>\n");
		p = fgets(line,BUFFER_LENGTH,stdin);
		ret = askForSensors(fd,sensors);
	}

	printf("STEP 3)\n");
	printf("Place Robot in region two.\n");
	for(i=0;i<NUM_SAMPLES;++i)
	{
		printf("Collecting sample %d of %d\n",(i+1),NUM_SAMPLES);
		printf("When ready to sample, press <return>\n");
		p = fgets(line,BUFFER_LENGTH,stdin);

	}

	printf("STEP 4)\n");
	cmd[0] = 150;
	cmd[1] = 0;
	//write(fd,cmd,2);

	disconnectDevice(fd);
	printf("Disconnected from device. File descriptor is now: %d\n",fd);
	return 0;
}


int connectDevice(char * device)
{
	int fd;
	struct termios tty, tty_check;
    if(device == NULL) return -1;

    /* Try to open the device */
    if((fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1)
    {
    	perror("Failed to connect to device");
        return -1;
    }
    /* Set communication flags */
    tcgetattr(fd, &tty);
    tcgetattr(fd, &tty_check);
    tty.c_iflag     = IGNBRK | IGNPAR;
    tty.c_lflag     = 0;
    tty.c_oflag     = 0;
    tty.c_cflag     = CREAD | CS8 | CLOCAL;
    cfsetispeed(&tty, B57600);
    cfsetospeed(&tty, B57600);
    tcsetattr(fd, TCSANOW, &tty);
    /* Flush I/O buffers */
    //tcflush(fd, TCIOFLUSH);
    return fd;
}


int disconnectDevice(int fd)
{

    /* Check if not connected */
    if(fd < 0) {
        return -1;
    }
    if(close(fd) == -1) {
    	perror("Failed to close connection to device");
        return -1;
    }
    fd = -1;
    return 0;
}

int askForSensors(int fd, int * array)
{
	int i;
	int res,e;
	unsigned char buf;
	unsigned char cmd[8];
	int count = 0;
//	cmd[0]=148;
//	cmd[1]=4;
//	cmd[2]=28;
//	cmd[3]=29;
//	cmd[4]=30;
//	cmd[5]=31;
	cmd[count++] = 131;
	cmd[count++] = 139;
	cmd[count++] = 10;
	cmd[count++] = 0;
	cmd[count++] = 0;
	for (i = 0; i < count; ++i)
	{
		res = write(fd, cmd + i, 1);
		//e = tcdrain(fd);
		usleep(10000);
		printf("res = %d\n",res);
	}
	for (i = 0; i < 9; i++)
	{
		res = read(fd,&buf,1);
		//array[i] = (int) buf;
		printf("res = %d val = %d\n",res,(int) buf);
	}
	return 0;
}

int sendByte(int fd, unsigned char byte) {
	int res;
	res = (write(fd, &byte, 1) == 1 ? 0 : -1);
    return res;
}
