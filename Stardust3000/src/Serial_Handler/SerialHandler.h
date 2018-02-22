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


class SerialHandler {
public:
	SerialHandler();
	virtual ~SerialHandler();
	void delay(int millisecond);
	std::string readData();
	int writeData(std::string p_data);
private:

	int uart0_filestream;
	int setup();
	int config();

};

#endif /* SERIALHANDLER_H_ */
