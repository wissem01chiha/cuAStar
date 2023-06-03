#include <wiringPi.h>
int main()
{
  wiringPiSetup();
  char i;
  char j;
  for(i=1;i<4;i++)
  {
        pinMode(i,OUTPUT);
  }
  
  while(1)
  { 
   digitalWrite(1, HIGH);//// turn on blue LED
   delay(5000);// wait 5 seconds
   digitalWrite(1, LOW); // turn off blue LED
   for(j=0;j<3;j++) // blinks for 3 times
   {
   delay(500);// wait 0.5 second
   digitalWrite(2, HIGH);// turn on yellow LED
   delay(500);// wait 0.5 second
   digitalWrite(2, LOW);// turn off yellow LED
   } 
   delay(500);// wait 0.5 second
   digitalWrite(3, HIGH);// turn on red LED
   delay(5000);// wait 5 second
   digitalWrite(3, LOW);// turn off red LED
   } 
}
