#include <wiringPi.h>
#include <stdio.h>

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
  {
   printf("Somebody is in this area!\n");
   digitalWrite(2,HIGH);
      delay(100);

  }

   else
  {
   printf("No one!\n");
   digitalWrite(2,LOW);
   delay(100);
  }
  }	
}
