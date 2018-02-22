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
		uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY );		//Open in non blocking read/write mode
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
int SerialHandler::config(){

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
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B1152000 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	return tcsetattr(uart0_filestream, TCSANOW, &options);
}

int SerialHandler::writeData(string p_data){
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, p_data.c_str(), p_data.size()+1);		//Filestream, bytes to write, number of bytes to write
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
}
string SerialHandler::readData(){

		if (uart0_filestream != -1)
		{
			// Read up to 255 characters from the port if they are there
			unsigned char rx_buffer[256];
			int rx_length = read(uart0_filestream,(void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
			if (rx_length < 0)
			{
				//An error occured (will occur if there are no bytes)
				printf("No data received (rx_lenght <0 \n");
			}
			else if (rx_length == 0)
			{
				//No data waiting
			}
			else
			{
				//Bytes received
				std::string data ((char*)rx_buffer);
				return data;
			}
		}
		return "";
}
/**
 * @brief n : keep the CPU running during approximatively the amount of millisecond given as parameter
 * @param : millisecond, as an integer : indicate delay duration
 */
void SerialHandler::delay(int millisecond)
{
	clock_t ticks1, ticks2;
	ticks1=clock();
	ticks2=ticks1;
	while((ticks2/CLOCKS_PER_SEC-ticks1/CLOCKS_PER_SEC)<(millisecond/1000))
			ticks2=clock();
}
