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
    string data ="";
	while(1) {

		serial.delay(1000);
		if (serial.data_availlable()!=0){
			data=serial.readData();
			cout << data << endl;
			if(data == "PUSHED") {
				cout << "PUSHED received" << endl;
				serial.writeData("ON");

			}
			else if (data=="NOTPUSHED") {
				cout << "NOTPUSHED received" << endl;
				serial.writeData("OFF");
			}
		}
		else {
			cout << "no data" <<endl;
		}
	}
}
