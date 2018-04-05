/*
 * SerialHandler.h
 *
 *  Created on: 3 févr. 2018
 *      Author: maxip
 */

#ifndef SRC_SERIALHANDLER_H_
#define SRC_SERIALHANDLER_H_

#include <iostream>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <time.h>  			//used for mydelay
#include <sys/ioctl.h>
#define HEADERSIZE	5
#define WRITE	0xF0
#define READ	0xF1
#define MAXOFFSET 255

class SerialHandler {

public:
	SerialHandler(int baud=115200, bool debug = false);
	virtual ~SerialHandler();

	//Main tools :
	int get(int offset, int length, uint8_t buffer[]);
	int set(int offset, int lenght, uint8_t data[]);

	//tools :
	int rFlush();
	int wFlush();
	int dataAvailable();
	void delay(int millisecond);
	//Debug tool :
	void setDebug(bool p_debug);

private:
	// used to store the file descriptor.
	int uart0_filestream;

	struct Header {
		uint8_t AddrDev[2];
		uint8_t AddrMem[2];
		uint8_t Count;
	};
	bool debug = false;
	int setup();
	int config(int baud);
	int readData(uint8_t* buffer, int len);
	int writeData(const uint8_t *p_data, int len);
	Header headerBuilder(uint8_t IO, short p_offset, int p_length, short p_AddrMem=0x5555);
	Header headerReader(uint8_t* buffer);
	void headerPrinter (SerialHandler::Header header);
};

#endif /* SERIALHANDLER_H_ */
