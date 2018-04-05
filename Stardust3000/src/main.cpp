/*
 * main.cpp
 *
 *  Created on: 22 févr. 2018
 *      Author: maxip
 */


#include <iostream>
#include "Serial_Handler/SerialHandler.h"
#include <bits/stdc++.h>
#include <string>

using namespace std;

int main() {
	SerialHandler serial;
	short offset;
	string message;
	do {
		cout << "your data : " << endl;
		cin >> message;
		cin.ignore();
		cout << "Offset : " << endl;
		cin >> offset;
		int n = message.length()+1;
		uint8_t data[n];
		strcpy((char*)(data), message.c_str());
		uint8_t data_received[n];
		if(serial.set(offset,n,data) <0) {
			break;
		}
		if(serial.get(offset,n,data_received) <0){
			break;
		}
		for (int i=0;i<n;i++) {
		if (i<1) {
			printf("%d : [%x]\n",i,data_received[i]);
		}
		else printf("%d : [%c]\n",i,data_received[i]);
		}
	}while(message != "exit");
}
