/*
 * main.cpp
 *
 *  Created on: 22 févr. 2018
 *      Author: maxip
 */


#include <iostream>
#include "Serial_Handler/SerialHandler.h"
#include "MemMap/Memmap.h"
#include <vector>
#define HEADER_SIZE	5
#define WRITE	0xF0
#define READ	0xF1
using namespace std;

void header_reader(uint8_t* buffer, SerialHandler::WR_header* header) {
    header->AddrMem[0]=buffer[0];
    header->AddrMem[1]=buffer[1];
    header->AddrDev[0]=buffer[2];
    header->AddrDev[1]=buffer[3];
    header->Count = buffer[4];
}
void header_printer (SerialHandler::WR_header header) {
	printf("AddrMem = %x,%x \n",header.AddrMem[0],header.AddrMem[1]);
	printf("AddrDev = %x,%x \n",header.AddrDev[0],header.AddrDev[1]);
	printf("Count = %d \n",header.Count);
}


int main() {
	SerialHandler serial;


	while(1){
		uint8_t rByte[255];
		uint8_t wByte[6]={' ',' ',' ',' ',' '};
		SerialHandler::WR_header header;
		SerialHandler::WR_header header_received;
		cin >> wByte;
		int data_size = sizeof(wByte);
		data_size+=HEADER_SIZE;
	    header.AddrMem[0]=WRITE;
	    header.AddrMem[1]=66;
	    header.AddrDev[0]=67;
	    header.AddrDev[1]=68;
	    header.Count=data_size;
	    unsigned char buffer[255];
	    buffer[0]=header.AddrMem[0];
	    buffer[1]=header.AddrMem[1];
	    buffer[2]=header.AddrDev[0];
	    buffer[3]=header.AddrDev[1];
	    buffer[4]=header.Count;
	    for (int i=0;i<6;i++) {
	    	buffer[i+5]=wByte[i];
	    }
		if(serial.writeData(buffer,11)>=0){
			cout << "write OK" <<endl;
		}
		else break;
		//serial.rFlush();
		int result = serial.readData(rByte,12);
		cout << "result : " << result << endl;
		header_reader(rByte,&header_received);
		header_printer(header_received);
		cout << "le message : ";
		for (int i=5; i<result;i++){
			printf("%c", rByte[i]);
			rByte[i]=' ';
		}
		serial.rFlush();
		cout << endl;
	}
}

