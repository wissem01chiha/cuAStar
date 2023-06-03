#include <wiringPi.h>
int main()
{
  wiringPiSetup();
  char val;
  {
    pinMode(1,INPUT);
    pinMode(2,OUTPUT);
  }
  
  while(1)
  { 
   val=digitalRead(1);
   if(val==1)//check if the button is pressed, if yes, turn on the LED
   digitalWrite(2,LOW);
   else
   digitalWrite(2,HIGH);
  }	
}
