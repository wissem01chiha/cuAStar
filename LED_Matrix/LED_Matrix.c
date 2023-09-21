#include <wiringPi.h>
int main()
{
  wiringPiSetup();
  char i;
  char j;
  for(i=0;i<8;i++)
  {
    pinMode(i,OUTPUT);
  }
    for(j=21;j<29;j++)
  {
    pinMode(j,OUTPUT);
  }
  while(1)
  {  
    for(i=0;i<8;i++)
  {
     digitalWrite(i, HIGH);// set I/O pins as ¡°high¡±
      delay(200);       // delay
  }
     for(j=21;j<29;j++)
  {
     digitalWrite(j, LOW);// set I/O pins as ¡°low¡±
  }
    for(i=0;i<8;i++)
  {
     digitalWrite(i, LOW);// set I/O pins as ¡°high¡±
     delay(200);       // delay
  }
    for(i=0;i<8;i++)
  {
     digitalWrite(i, HIGH);// set I/O pins as ¡°high¡±
     delay(200);       // delay
  }
     for(j=21;j<29;j++)
  {
     digitalWrite(j,HIGH);// set I/O pins as ¡°high¡±
     delay(200);       // delay
  }
     for(j=21;j<29;j++)
  {
     digitalWrite(j, LOW);// set I/O pins as ¡°low¡±
     delay(200);       // delay
  }
    for(i=0;i<8;i++)
  {
     digitalWrite(i, LOW);// set I/O pins as ¡°low¡±
     delay(200);       // delay
  }
  }    
}
