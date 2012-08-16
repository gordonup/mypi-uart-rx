#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
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
#include "uart.h"
#include <errno.h>
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
#include <stdlib.h>
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

#define DL_PACKET_SIZE  1

int open_serial_port_1() {
	int fd;
	fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	return fd;
}
void set_serial_port_1(int fd) {
	struct termios options;
	/*
	 * Get the current options for the port...
	 */
	tcgetattr(fd, &options);
	/*
	 * Set the baud rates to 19200...
	 */
	cfsetispeed(&options, B9600);
	cfsetospeed(&options, B9600);
	/*
	 * Enable the receiver and set local mode...
	 */
	options.c_cflag |= (CLOCAL | CREAD);
	// CHARACTER SIZE
	options.c_cflag &= ~CSIZE; /* Mask the character size bits */
	options.c_cflag |= CS8; /* Select 8 data bits */
	// NO PARITY
	options.c_cflag &= ~PARENB;
	options.c_cflag &= ~CSTOPB;
	// DISABLE HW FLOW CONTROL
	options.c_cflag &= ~CRTSCTS;
	// CANONICAL/RAW INPUT
	options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	// DISABLE SW FLOW CONTROL
	options.c_iflag &= ~(IXON | IXOFF | IXANY);
	// DELAYS
	options.c_cc[VMIN] = 1;
	options.c_cc[VTIME] = 10;
	/*
	 * Set the new options for the port...
	 */
	tcsetattr(fd, TCSANOW, &options);
}
void read_serial_port_1(int fd, char *buf) {
	int read_output = 0;
	int bytes;
	int num = 0;
	ioctl(fd, FIONREAD, &bytes);
	printf("Queue: %d", bytes);
	tcflush(fd, TCIOFLUSH);
	ioctl(fd, FIONREAD, &bytes);
	printf("***** Queue update: %d\n", bytes);
	getchar();
	while (DL_PACKET_SIZE - num > 0) {
		sleep(1);
		ioctl(fd, FIONREAD, &bytes);
		printf("*** WAITING: *** %d", bytes);
		read_output = read(fd, buf, sizeof(buf));
		if (read_output != -1) {
			printf("*** CHAR: *** %s", buf);
			getchar();
			num += read_output;
			continue;
		}
		// BLOCKING BEHAVIOUR
		// fcntl(fd, F_SETFL, 0);
		printf("Read status: %d\n", read_output);
		printf("Error message: %s\n", strerror(errno));
	}
}

int close_serial_port_1(int fd) {
	int closing_result;
	closing_result = close(fd);
	return closing_result;
}

struct serial_port_unix {
	int fd; // Serial port file descriptor
	struct termios termios_backup; // Terminal info before using the port
	struct termios termios_new; // Terminal info during the transaction
};
typedef void *serial_port;
#  define INVALID_SERIAL_PORT (void*)(~1)
#  define CLAIMED_SERIAL_PORT (void*)(~2)
// Work-around to claim uart interface using the c_iflag (software input processing) from the termios struct
#  define CCLAIMED 0x80000000
#define UART_DATA( X ) ((struct serial_port_unix *) X)
#ifndef MAX
#define MAX( a, b ) ( ((a) > (b)) ? (a) : (b) )
#endif

#ifndef MIN
#define MIN( a, b ) ( ((a) < (b)) ? (a) : (b) )
#endif

serial_port open_serial_port_2(const char *pcPortName) {
	struct serial_port_unix *sp = (struct serial_port_unix *) malloc(
			sizeof(struct serial_port_unix));
	if (sp == 0)
		return INVALID_SERIAL_PORT;

	sp->fd = open(pcPortName, O_RDWR | O_NOCTTY | O_NONBLOCK);
	perror("jj");
	if (sp->fd == -1) {
		close_serial_port_2(sp, 0);
		return INVALID_SERIAL_PORT;
	}

	if (tcgetattr(sp->fd, &sp->termios_backup) == -1) {
		close_serial_port_2(sp, 0);
		return INVALID_SERIAL_PORT;
	}
	// Make sure the port is not claimed already
	if (sp->termios_backup.c_iflag & CCLAIMED) {
		close_serial_port_2(sp, 0);
		return CLAIMED_SERIAL_PORT;
	}
	// Copy the old terminal info struct
	sp->termios_new = sp->termios_backup;

	sp->termios_new.c_cflag = CS8 | CLOCAL | CREAD;
	sp->termios_new.c_iflag = CCLAIMED | IGNPAR;
	sp->termios_new.c_oflag = 0;
	sp->termios_new.c_lflag = 0;
	// MINE
	sp->termios_new.c_cflag &= ~CRTSCTS;
	//sp->termios_new.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	sp->termios_new.c_iflag &= ~(IXON | IXOFF | IXANY);

	sp->termios_new.c_cc[VMIN] = 0; // block until n bytes are received
	sp->termios_new.c_cc[VTIME] = 0; // block until a timer expires (n * 100 mSec.)

	if (tcsetattr(sp->fd, TCSANOW, &sp->termios_new) == -1) {
		close_serial_port_2(sp, 1);
		return INVALID_SERIAL_PORT;
	}
	return sp;

}

void set_serial_port_2(serial_port sp, const uint32_t uiPortSpeed) {
	printf("Serial port speed requested to be set to %d bauds.", uiPortSpeed);

	// Portability note: on some systems, B9600 != 9600 so we have to do
	// uint32_t <=> speed_t associations by hand.
	speed_t stPortSpeed = B9600;
	switch (uiPortSpeed) {
	case 9600:
		stPortSpeed = B9600;
		break;
	case 19200:
		stPortSpeed = B19200;
		break;
	case 38400:
		stPortSpeed = B38400;
		break;
#  ifdef B57600
	case 57600:
		stPortSpeed = B57600;
		break;
#  endif
#  ifdef B115200
	case 115200:
		stPortSpeed = B115200;
		break;
#  endif
#  ifdef B230400
	case 230400:
		stPortSpeed = B230400;
		break;
#  endif
#  ifdef B460800
	case 460800:
		stPortSpeed = B460800;
		break;
#  endif
	default:
		printf(
				"Unable to set serial port speed to %d bauds. Speed value must be one of those defined in termios(3).",
				uiPortSpeed);
		return;
	};

	// Set port speed (Input and Output)
	cfsetispeed(&(UART_DATA(sp)->termios_new), stPortSpeed);
	cfsetospeed(&(UART_DATA(sp)->termios_new), stPortSpeed);
	if (tcsetattr(UART_DATA(sp)->fd, TCSADRAIN, &(UART_DATA(sp)->termios_new))
			== -1) {
		printf("%s", "Unable to apply new speed settings.");
	}
}

int read_serial_port_2(serial_port sp, uint8_t *pbtRx, const size_t szRx,
		void *abort_p, int timeout) {
	int iAbortFd = abort_p ? *((int *) abort_p) : 0;
	int received_bytes_count = 0;
	int available_bytes_count = 0;
	const int expected_bytes_count = (int) szRx;
	int res;
	fd_set rfds;
	do {
		select:
		// Reset file descriptor
		FD_ZERO(&rfds);
		FD_SET(UART_DATA(sp)->fd, &rfds);

		if (iAbortFd) {
			FD_SET(iAbortFd, &rfds);
		}

		struct timeval timeout_tv;
		if (timeout > 0) {
			timeout_tv.tv_sec = (timeout / 1000);
			timeout_tv.tv_usec = ((timeout % 1000) * 1000);
		}

		res = select(MAX(UART_DATA(sp)->fd, iAbortFd) + 1, &rfds, NULL, NULL,
				timeout ? &timeout_tv : NULL);

		if ((res < 0) && (EINTR == errno)) {
			// The system call was interupted by a signal and a signal handler was
			// run.  Restart the interupted system call.
			goto select;
		}

		// Read error
		if (res < 0) {
			printf("Error: %s", strerror(errno));
			return -1;
		}
		// Read time-out
		if (res == 0) {
			printf("%s", "Timeout!");
			return -1;
		}

		if (FD_ISSET(iAbortFd, &rfds)) {
			// Abort requested
			printf("%s", "Abort!");
			close(iAbortFd);
			return -1;
		}

		// Retrieve the count of the incoming bytes
		res = ioctl(UART_DATA(sp)->fd, FIONREAD, &available_bytes_count);
		if (res != 0) {
			return -1;
		}
		// There is something available, read the data
		res =
				read(UART_DATA(sp)->fd, pbtRx + received_bytes_count,
						MIN(available_bytes_count, (expected_bytes_count - received_bytes_count)));
		// Stop if the OS has some troubles reading the data
		if (res <= 0) {
			return -1;
		}
		received_bytes_count += res;

	} while (expected_bytes_count > received_bytes_count);
	printf("RX", pbtRx, szRx);
	return 0;
}

void flush_serial_port_2(serial_port sp) {
	// This line seems to produce absolutely no effect on my system (GNU/Linux 2.6.35)
	tcflush(UART_DATA(sp)->fd, TCIFLUSH);
	// So, I wrote this byte-eater
	// Retrieve the count of the incoming bytes
	int available_bytes_count = 0;
	int res;
	res = ioctl(UART_DATA(sp)->fd, FIONREAD, &available_bytes_count);
	if (res != 0) {
		return;
	}
	if (available_bytes_count == 0) {
		return;
	}
	char *rx = (char *) malloc(available_bytes_count);
	// There is something available, read the data
	res = read(UART_DATA(sp)->fd, rx, available_bytes_count);
	printf("%d bytes have eatten.", available_bytes_count);
	free(rx);
}

void close_serial_port_2(const serial_port sp, const bool restore_termios) {
	if (UART_DATA(sp)->fd >= 0) {
		if (restore_termios)
			tcsetattr(UART_DATA(sp)->fd, TCSANOW,
					&UART_DATA(sp)->termios_backup);
		close(UART_DATA(sp)->fd);
	}
	free(sp);
}

int open_serial_port_3() {
	int fd;
	fd = open("/dev/ttyAMA0", O_RDWR);
	return fd;
}
void set_serial_port_3(int fd) {
}

int read_serial_port_3(int fd){
	int read_output=-1;
	void *buf;
	do{
	read_output = read(fd, buf, sizeof(buf));
	} while (read_output <0);
	printf("Buffer %s", (char *)buf);
	return read_output;
}
