#include <wiringPi.h>
int main()
{
  wiringPiSetup();

  {
        pinMode(1,OUTPUT);
  }
  
  while(1)
  { 
        digitalWrite(1,HIGH);
        delay(1000);
        digitalWrite(1,LOW);
        delay(1000);	  
  }
	
}