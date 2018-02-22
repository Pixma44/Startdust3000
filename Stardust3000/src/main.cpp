/*
 * main.cpp
 *
 *  Created on: 22 févr. 2018
 *      Author: maxip
 */


#include <iostream>
#include "Serial_Handler/SerialHandler.h"

using namespace std;

int main() {

	SerialHandler serial;
	cout << "Hello world" <<endl;
	clock_t click = clock();
	serial.writeData("Hello pti rpi");
	cout << serial.readData() << endl;
	cout << (clock()-click) <<endl;
}


