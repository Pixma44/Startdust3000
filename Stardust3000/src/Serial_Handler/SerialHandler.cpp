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
		uart0_filestream = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY );		//Open in non blocking read/write mode
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
	cfsetspeed(&options, B9600);
	cfmakeraw(&options);
	options.c_cflag &= ~CSTOPB;
	options.c_cflag |= CLOCAL;
	options.c_cflag |= CREAD;
	options.c_cc[VTIME]=0;
	options.c_cc[VMIN]=1;
	return tcsetattr(uart0_filestream, TCSANOW, &options);
}

int SerialHandler::writeData(string p_data){
	if (uart0_filestream != -1)
	{
		int count = write(uart0_filestream, p_data.c_str(), p_data.size()+1);	//Filestream, bytes to write, number of bytes to write
		tcdrain(uart0_filestream);
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
			//tcflush(uart0_filestream, TCIFLUSH);
			unsigned char rx_buffer[255]="";
			std::string data="";
			int rx_length=0;
			rx_length = read(uart0_filestream,(void*)rx_buffer, 255);		//Filestream, buffer to store in, number of bytes to read (max)
			if (rx_length < 0)
			{
				//An error occured (will occur if there are no bytes)
				printf("Error during reception (rx_lenght =%d ) \n", rx_length);
			}
			else
			{
				return data=((char*)rx_buffer);
			}
		}
		return "no data";
}
//string SerialHandler::readData(){
//
//		if (uart0_filestream != -1)
//		{
//			// Read up to 255 characters from the port if they are there
//			//tcflush(uart0_filestream, TCIFLUSH);
//			unsigned char rx_buffer[255];
//			std::string data;
//			int rx_length=0;
//			do {
//				rx_length = read(uart0_filestream,(void*)rx_buffer, 9);		//Filestream, buffer to store in, number of bytes to read (max)
//				if (rx_length < 0)
//				{
//					//An error occured (will occur if there are no bytes)
//					printf("Error during reception (rx_lenght =%d ) \n", rx_length);
//					break;
//				}
//				else
//				{
//					//Bytes received
//					//printf("%s",rx_buffer);
//					data+= ((char*)rx_buffer);
//				}
//			}while (rx_length >0);
//			return data;
//		}
//		return "no data";
//}
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
void SerialHandler::serialTest_1(){
	string data ="";
	while(1) {
		this->delay(1000);
		if (this->data_availlable()!=0){
			data=this->readData();
			cout << data << endl;
			if(data == "PUSHED") {
				cout << "PUSHED received" << endl;
				this->writeData("ON");
			}
			else if (data=="NOTPUSHED") {
				cout << "NOTPUSHED received" << endl;
				this->writeData("OFF");
			}
		}
		else {
			cout << "no data" <<endl;
		}
	}
	/*arduino code :
	#include <Wire.h>
	#include <SPI.h>
	#include <variant.h>
	#include <bootloaders/boot.h>
	// here we just say hello, and return to boot for next update.

	int val = 0;
	int state=LOW;
	String ordre="";
	void setup() {
		 // put your setup code here, to run once:
	 	 P_COM3.serial.begin(9600);
	 	 pinMode (P_COM4.Pin.P4, INPUT_PULLUP);
	 	 pinMode (P_COM4.Pin.P5, OUTPUT);

	}

	void loop() {
		// put your main code here, to run repeatedly:
		//P_COM3.serial.print("Hello world 4 ! ! );
		ordre = P_COM3.serial.readString();
		if(ordre == "ON"){
			digitalWrite(P_COM4.Pin.P5, HIGH);
		}
		else if (ordre == "OFF") {
			digitalWrite (P_COM4.Pin.P5, LOW);
		}
		if(digitalRead(P_COM4.Pin.P4)==LOW ){
			P_COM3.serial.print("PUSHED");
		}
		else {
			P_COM3.serial.print("NOTPUSHED");
		}
		delay(500);
		//}
		//String data=P_COM3.serial.readString();
		//P_COM3.serial.print(data);
	}
	*/
}
