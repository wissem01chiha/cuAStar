#include <wiringPi.h>
int main()
{
  wiringPiSetup();
  char i;
  char j;

  {
    pinMode(1,OUTPUT);
  }
  
  while(1)
   { 
    for(i=0;i<80;i++)// output a frequency sound
   { digitalWrite(1,HIGH);// sound
     delay(1);//delay1ms 
     digitalWrite(1,LOW);//not sound
     delay(1);//ms delay 
    } 
   for(j=0;j<100;j++)// output a frequency sound
    { digitalWrite(1,HIGH);// sound
      delay(2);
      digitalWrite(1,LOW);//not sound
      delay(2);//2ms delay 
     }
   } 	  
  }

