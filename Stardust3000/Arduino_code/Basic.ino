#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>
/** here we just say hello, and return to boot for next update.
*/
#define HEADERSIZE  5
#define WRITE 0xF0
#define READ  0xF1
#define MAXOFFSET 250

int num,bsend,i;
uint8_t memmap[250];
struct WR_header {
  uint8_t AddrDev[2];
  uint8_t AddrMem[2];
  uint8_t Count;
};
uint8_t WriteOK = 0xFB;
int incomingByte=0;
byte buff_header[5];
byte data[250];
void setup() {
// put your setup code here, to run once:
  pinMode(P_COM4.Pin.P5, OUTPUT);
  pinMode(P_COM4.Pin.P4, INPUT);
  P_COM3.serial.begin(115200);
}

void loop() {
  if(digitalRead(P_COM4.Pin.P4)==HIGH) {
    num =((num*11/3)+1)%5000;
    if(bsend==0){
      P_COM3.serial.print(num);
      bsend=1;
    }
  }
  else{
      i++;
      if (i>10){
        i=0;
        bsend=0;
      }
  }
  digitalWrite(P_COM4.Pin.P5, HIGH);
  if(P_COM3.serial.available() >=5) {
    P_COM3.serial.readBytes(buff_header,5);
    int len = buff_header[4];
    if(buff_header[0]==WRITE){
      digitalWrite(P_COM4.Pin.P5, LOW);
      if(P_COM3.serial.readBytes(&memmap[buff_header[1]],len)==len) {
        //data received ok
        P_COM3.serial.write(0xFB); // indicate transaction is ok
      }
      else {
        P_COM3.serial.write(0xFF); // send error to rpi
      }
    }
    else if (buff_header[0] == READ) {
      uint8_t* buffer = (uint8_t*)malloc(sizeof(uint8_t)*(len+1));
      buffer[0]=0xFC; // indicate an answer to a get request;
      int offset = buff_header[1];
      for (int i=0;i < len; i++){
        buffer[i+1] = memmap[offset+i];
      }
      P_COM3.serial.Print::write(buffer,len+1);
    }
    else
    {
      P_COM3.serial.write(0xFF);
    }
  }
}
