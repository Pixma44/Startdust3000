/*
 * SerialHandler.cpp
 *
 *  Created on: 3 févr. 2018
 *      Author: maxip
 */

#include "SerialHandler.h"

using namespace std;

  /********************/
 /*Public function : */
/********************/

/**
* @brief Class constructor
*
* @
*/
SerialHandler::SerialHandler(int baud, bool debug) {
	uart0_filestream = -1;
	setDebug(debug);
	if(setup()<0){
	cout << "Serial setup failed." << endl;
	}
	else {
		if (config(baud)<0){
			cout << "Serial configuration failed" << endl;
		}
	}
}
/**
* @brief Class destructor
*
* @
*/
SerialHandler::~SerialHandler() {
// TODO Auto-generated destructor stub
}
/*********/
//Main tools :

/**
* @brief : Set() allows to read p_len bytes from the p_offset position into arduino's memory map over serial, and push it into p_buffer[].
*
* @return 0 if ok, -1 if not.
*/
int SerialHandler::get(int p_offset, int p_len, uint8_t p_buffer[]) {
	// Prevent from writing out of the memory map and/or to much data.
	if ((p_offset+p_len < MAXOFFSET && p_offset >0) && p_len < 250 ) {
		// Build the header with the information provided.
		Header header=headerBuilder(READ, p_offset, p_len);
		uint8_t buffer[5];
		//setting the header
		buffer[0]=header.AddrDev[0];
		buffer[1]=header.AddrDev[1];
		buffer[2]=header.AddrMem[0];
		buffer[3]=header.AddrMem[1];
		buffer[4]=header.Count;
		if(writeData(buffer,5)<0) return -1; //sending request
		if(readData(p_buffer,(p_len+1)) >p_len){
			if(p_buffer[0]== 0xFC) { //request as been proceed by the Arduino
				return 0;
			}
			if (p_buffer[0] == 0xFF) {
				printf("Arduino device report an error \n");
				return -1;
			}
			else return -1;
		}
		return -1;
	}
	else {
		printf("offset or length incorrect \n");
		return -1;
	}
}
/**
* @brief : Set() allows to write p_len bytes from data[] at the p_offset position into arduino's memory map over serial.
*
* @return 0 if ok, -1 if not.
*/
int SerialHandler::set(int p_offset, int p_len, uint8_t data[]) {
	// Prevent from writing out of the memory map and/or to much data.
	if ((p_offset+p_len < MAXOFFSET && p_offset >0) && p_len < 250 ) {
		// Build the header with the information provided.
		Header header=headerBuilder(WRITE, p_offset, p_len);
		uint8_t buffer[255];
		//setting the header
		buffer[0]=header.AddrDev[0];
		buffer[1]=header.AddrDev[1];
		buffer[2]=header.AddrMem[0];
		buffer[3]=header.AddrMem[1];
		buffer[4]=header.Count;
		// filling the buffer with data
		for (int i = 0; i<p_len; i++) {
			buffer[i+5]=data[i];
		}
		if(writeData(buffer,p_len+5)==0) { //sending data
			if(readData(&buffer[0],1)>0) { // look if there is an answer
				printf("buffer : %x \n",buffer[0]); //debug
				if(buffer[0]==0xFB) return 0; // If true, arduino have received the data corectly
				else {
					printf("Transmission failed \n");
					return -1;
				}
			}
			else { //mean readData get no data or failed
				printf("No answer from Arduino \n");
				return -1;
			}
		}
		printf("Write Failed \n");
		return -1;
	}
	else
	{
		printf("offset or length incorrect \n");
		return -1;
	}
}
//*********
//tools :

/**
* @brief : flush the hardware read buffer.
*
* @return 0 if ok, -1 if not.
*/
int SerialHandler::rFlush(){
	return tcflush(uart0_filestream,TCIFLUSH);
}
/**
* @brief : flush the hardware write buffer.
*
* @return 0 if ok, -1 if not.
*/
int SerialHandler::wFlush(){
	return tcflush(uart0_filestream,TCOFLUSH);
}
/**
* @brief : check if there is some bytes available to be read.
*
* @return the number of bytes available.
*/
int SerialHandler::dataAvailable() {
	int bytes=0;
	ioctl(uart0_filestream, FIONREAD, &bytes);
	return bytes;
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
//*********
//Debug tool :
/**
* @brief enable debug message.
*
*
*/
void SerialHandler::setDebug(bool p_debug){
	this->debug=p_debug;
}

  /*********************/
 /*Private function : */
/*********************/

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
	/*CONFIGURE THE UART
	//The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
	//	Baud rate:- B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200, B230400, B460800, B500000, B576000, B921600, B1000000, B1152000, B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
	//	CSIZE:- CS5, CS6, CS7, CS8
	//	CLOCAL - Ignore modem status lines
	//	CREAD - Enable receiver
	//	IGNPAR = Ignore characters with parity errors
	//	ICRNL - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
	//	PARENB - Parity enable
	//	PARODD - Odd parity (else even)
	 */
	struct termios options;
	speed_t speed;
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
	options.c_cflag &= ~CSIZE; //Data Size 8;
	options.c_cflag |= CS8;
	options.c_cflag &= ~PARENB; // No Parity
	options.c_cflag &= ~PARODD;
	options.c_iflag &= ~(INPCK | ISTRIP);
	options.c_cflag &= ~CSTOPB; // Stop Bit One
	options.c_cflag &= ~CRTSCTS; // No hardware flow control
	options.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	options.c_iflag &= ~(BRKINT | ICRNL); // Raw input
	options.c_lflag &= ~(ICANON | IEXTEN | ECHO | ECHOE | ISIG);
	options.c_oflag &= ~OPOST; // Raw output
	options.c_cc[VMIN]  = 0; // No wait time
	options.c_cc[VTIME] = 0;
	if(tcsetattr(uart0_filestream, TCSANOW, &options)){
		close(uart0_filestream);
		return -1;
	}
	return 0;
}
/**
* @brief : write 'len' bytes from p_data into uart.
*
* @return 0 if ok, -1 if not.
*/
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
}
/**
* @brief : read 'len' bytes from uart and put it into 'buffer'.
*
* @return 0 if ok, -1 if not.
*/
int SerialHandler::readData(uint8_t* buffer, int len){
	int _timeout = 100; //based on empiric test.
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
/**
* @brief : build an appropriate header based on given parameters.
*
* @return 0 if ok, -1 if not.
*/
SerialHandler::Header SerialHandler::headerBuilder(uint8_t IO, short p_offset, int p_length, short p_AddrMem) {
	Header header;
	header.AddrDev[0]=IO; //device address MSB
	header.AddrDev[1]=p_offset; //device address LSB
	header.AddrMem[0]= (p_AddrMem >> 8) & 0xff; //memory pointer address MSB
	header.AddrMem[1]= p_AddrMem & 0xff; // memory pointer address LSB
	header.Count=p_length; // Number of data
	return header;
}
/**
* @brief : Build an Header from a received buffer.
*
* @return 0 if ok, -1 if not.
*/
SerialHandler::Header SerialHandler::headerReader(uint8_t* buffer) {
    Header header;
	header.AddrDev[0]=buffer[0];
    header.AddrDev[1]=buffer[1];
    header.AddrMem[0]=buffer[2];
    header.AddrMem[1]=buffer[3];
    header.Count = buffer[4];
    return header;
}
/**
* @brief : Structured print of and header
*
* @return 0 if ok, -1 if not.
*/
void SerialHandler::headerPrinter (SerialHandler::Header header) {
	printf("AddrDev = %x,%x \n",header.AddrDev[0],header.AddrDev[1]);
	printf("AddrMem = %x,%x \n",header.AddrMem[0],header.AddrMem[1]);
	printf("Count = %d \n",header.Count);
}




