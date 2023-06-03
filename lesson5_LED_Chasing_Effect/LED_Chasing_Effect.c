#include <wiringPi.h>
int main()
{
  wiringPiSetup();
  char i;
  for(i=1;i<4;i++)
  {
    pinMode(i,OUTPUT);
  }
  
  while(1)
  {  
    for (i=1;i<4;i ++) 
   {
    digitalWrite(i, LOW);// set I/O pins as ¡°low¡±
     delay(200);        // delay
   }
   for (i=1;i<4;i ++) 
   {
     digitalWrite(i, HIGH);// set I/O pins as ¡°high¡±
     delay(200);       // delay
   } 
  }    
}
