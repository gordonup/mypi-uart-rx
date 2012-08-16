/*
 * c.c
 *
 *  Created on: 10/ago/2012
 *      Author: Utente
 */

/* Hello World program */

#include <stdio.h>
#include "uart.h"
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/types.h>
#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

int main() {
	int fd = 0;
	int read_output;
	fd = open_serial_port_3();
	read_output = read_serial_port_3(fd);
	printf("N bytes %d", read_output);
	return 0;
}

/*
 int main(int argc, char ** argv) {
 int fd;
 // Open the Port. We want read/write, no "controlling tty" status, and open it no matter what state DCD is in
 fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
 if (fd == -1) {
 perror("open_port: Unable to open /dev/ttyAMA0 - ");
 return(-1);
 }

 struct termios options;
 tcgetattr(fd, &options);
 cfsetispeed(&options, B9600);
 cfsetospeed(&options, B9600);
 tcsetattr(fd, TCSANOW, &options);

 // Turn off blocking for reads, use (fd, F_SETFL, FNDELAY) if you want that
 fcntl(fd, F_SETFL, 0);

 // Write to the port
 int n = write(fd,"Hello",6);
 if (n < 0) {
 perror("Write failed - ");
 return -1;
 }

 // Read up to 255 characters from the port if they are there
 char buf[256];
 usleep(1000);
 n = read(fd, (void*)buf, 255);
 if (n < 0) {
 perror("Read failed - ");
 return -1;
 } else if (n == 0) printf("No data on port\n");
 else {
 buf[n] = '\0';
 printf("%i bytes read : %s", n, buf);
 }

 // Don't forget to clean up
 close(fd);
 return 0;
 }
 */
