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
   if(val==1)
   digitalWrite(2,LOW);
   else
   digitalWrite(2,HIGH);
  }	
}
