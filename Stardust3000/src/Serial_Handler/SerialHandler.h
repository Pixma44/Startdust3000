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

class SerialHandler {

public:
	SerialHandler();
	virtual ~SerialHandler();
	void delay(int millisecond);
	int readData(uint8_t* buffer, int len);
	int get();
	int writeData(const uint8_t *p_data, int len);
	int data_availlable();
	void serialTest_1();
	void rFlush();
	struct WR_header {
		uint8_t AddrDev[2];
		uint8_t AddrMem[2];
		uint8_t Count;
	};

private:
	int uart0_filestream;
	int setup();
	int config(int baud=115200);

};

#endif /* SERIALHANDLER_H_ */
