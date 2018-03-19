
#include <Wire.h>
#include <SPI.h>
#include <variant.h>
#include <bootloaders/boot.h>
/** here we just say hello, and return to boot for next update.
*/
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
