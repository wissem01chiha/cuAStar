#include <wiringPi.h>
#include <pcf8591.h>
#include <stdio.h>

#define Address 0x48
#define BASE 64
#define A0 BASE+0
#define A1 BASE+1
#define A2 BASE+2
#define A3 BASE+3

int main(void)
{
      unsigned char value;
	wiringPiSetup();
      pinMode(1,OUTPUT);
	pcf8591Setup(BASE,Address);
        
	while(1)
	{
        value=analogRead(A0);              
        printf("A0:%d\n",value);
        delay(100);
        if(value>200)
        {
         digitalWrite(1,HIGH);
         delay(100);
        }
        else
        {
         digitalWrite(1,LOW);
        }	
	}
     }
