#include <wiringPi.h>
#include <wiringShift.h>
int dataPin = 23; //define three pins
int latchPin = 24;
int clockPin = 25;
int a[10]={
    252,96,218,242,102,182,190,224,254,246}; 
int x;
int main()
{
  wiringPiSetup();
 
  {
  pinMode(latchPin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(dataPin,OUTPUT); //three pins as output
  }
  
  while(1)
  { 
  for(x=0; x<10 ;x++ )                        //calculate counting function
  {
    digitalWrite(latchPin,LOW);
    shiftOut(dataPin,clockPin,MSBFIRST,a[x]);     //display array a[x]
    digitalWrite(latchPin,HIGH);
    delay(1000);
  }
  }	
}
