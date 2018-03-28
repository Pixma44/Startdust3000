/*
 * SerialHandler.cpp
 *
 *  Created on: 3 févr. 2018
 *      Author: maxip
 */

#include "SerialHandler.h"

using namespace std;

SerialHandler::SerialHandler() {
	// TODO Auto-generated constructor stub
	uart0_filestream = -1;

	if(setup()<0){
	cout << "Serial setup failed." << endl;
	}
	else {
		if (config()<0){
			cout << "Serial config failed" << endl;
		}
	}

}

SerialHandler::~SerialHandler() {
	// TODO Auto-generated destructor stub
}

/**
 * @brief Open the fd of the uart device (ttyS0)
 *
 * @return 0 if ok, -1 if not.
 */
int SerialHandler::setup() {

		//OPEN THE UART
		//The flags (defined in fcntl.h):
		//	Access modes (use 1 of these):
		//		O_RDONLY - Open for reading only.
		//		O_RDWR - Open for reading and writing.
		//		O_WRONLY - Open for writing only.
		//
		//	O_NDELAY / O_NONBLOCK (same function) - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
		//											if there is no input immediately available (instead of blocking). Likewise, write requests can also return
		//											immediately with a failure status if the output can't be written immediately.
		//
		//	O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
		uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY );		//Open in non blocking read/write mode
		if (uart0_filestream == -1)
		{
			//ERROR - CAN'T OPEN SERIAL PORT
			printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
			return -1;
		}
		return 0;

}

/**
 * @brief Configuration for uart device
 *
 * @return 0 if ok, -1 if not.
 */
	int SerialHandler::config(int baud){

		struct termios options;
		speed_t speed;

		//CONFIGURE THE UART
		//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
		//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
		//	CSIZE:- CS5, CS6, CS7, CS8
		//	CLOCAL - Ignore modem status lines
		//	CREAD - Enable receiver
		//	IGNPAR = Ignore characters with parity errors
		//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
		//	PARENB - Parity enable
		//	PARODD - Odd parity (else even)
		switch (baud)
		{
		case 1200:
			speed = B1200;
			break;
		case 9600:
			speed = B9600;
			break;
		case 19200:
			speed = B19200;
			break;
		case 38400:
			speed = B38400;
			break;
		case 57600:
			speed = B57600;
			break;
		case 115200:
			speed = B115200;
			break;
		case 230400:
			speed = B230400;
			break;
		default:
			close(uart0_filestream);
			cout << "Baudrate invalid" <<endl;
			return -1;
		}

		tcgetattr(uart0_filestream, &options);
		if (cfsetispeed(&options, speed) || cfsetospeed(&options, speed))
		{
			close(uart0_filestream);
			return -1;
		}
		cfmakeraw(&options);

		options.c_cflag |= (CLOCAL | CREAD );
		//Data Size 8;
		options.c_cflag &= ~CSIZE;
		options.c_cflag |= CS8;
		// No Parity
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~PARODD;
		options.c_iflag &= ~(INPCK | ISTRIP);
		// Stop Bit One
		options.c_cflag &= ~CSTOPB;
		// No hardware flow control
		options.c_cflag &= ~CRTSCTS;
		// No software flow control
		options.c_iflag &= ~(IXON | IXOFF | IXANY);
		// Raw input
		options.c_iflag &= ~(BRKINT | ICRNL);
		options.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ISIG);
		// Raw output
		options.c_oflag &= ~OPOST;
		// No wait time
		options.c_cc[VMIN]  = 0;
		options.c_cc[VTIME] = 0;
		if(tcsetattr(uart0_filestream, TCSANOW, &options)){
			close(uart0_filestream);
			return -1;
		}
		return 0;
	}

int SerialHandler::writeData(const uint8_t* p_data, int len){
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, p_data, len);	//Filestream, bytes to write, number of bytes to write
		if (count < 0)
		{
			printf("UART TX error\n");
			return -1;
		}
	}
	else {
		printf("Filestream busy \n");
		return -1;
	}
	return 0;
	/**/
}
	int SerialHandler::readData(uint8_t* buffer, int len){
		int _timeout = 50;
		fd_set fds;
		struct timeval tv;
		int numread=0;
		int retval=0;
		if (uart0_filestream == -1)
		{
			return -1;
		}
		while (numread < len){
			FD_ZERO(&fds);
			FD_SET(uart0_filestream, &fds);
			tv.tv_sec  = _timeout / 1000;
			tv.tv_usec = (_timeout % 1000) * 1000;

			retval = select(uart0_filestream + 1, &fds, NULL, NULL, &tv);

			if (retval < 0)
			{
				return -1;
			}
			else if (retval == 0)
			{
				return numread;
			}
			else if (FD_ISSET(uart0_filestream, &fds))
			{
				retval = ::read(uart0_filestream, (uint8_t*) buffer + numread, len - numread);
				if (retval < 0)
					return -1;
				numread += retval;
			}
		}
		return numread;
	}
	void SerialHandler::rFlush(){
		tcflush(uart0_filestream,TCIFLUSH);
	}
	int SerialHandler::get()
	{
		uint8_t byte;

		if (uart0_filestream == -1)
			return -1;

		if (readData(&byte, 1) != 1)
			return -1;

		return byte;
	}

	/**
	 * @brief n : keep the CPU running during approximatively the amount of millisecond given as parameter
	 * @param : millisecond, as an integer : indicate delay duration
	 */
	int SerialHandler::data_availlable() {
		int bytes=0;
		ioctl(uart0_filestream, FIONREAD, &bytes);
		return bytes;
	}
	void SerialHandler::delay(int millisecond)
	{
		clock_t ticks1, ticks2;
		ticks1=clock();
		ticks2=ticks1;
		while((ticks2/CLOCKS_PER_SEC-ticks1/CLOCKS_PER_SEC)<(millisecond/1000))
				ticks2=clock();
	}

