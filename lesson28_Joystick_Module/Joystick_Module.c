#include <wiringPi.h>
#include <pcf8591.h>
#include <stdio.h>

#define Address 0x48
#define BASE 64
#define A0 BASE+0
#define A1 BASE+1
#define A2 BASE+2
#define A3 BASE+3
char dat;

int main(void)
{
        unsigned char value;
	wiringPiSetup();
        pinMode(1,INPUT);
	pcf8591Setup(BASE,Address);
	while(1)
	{
               value=analogRead(A0);              
               printf("X:%d    ",value);
               value=analogRead(A1);              
               printf("Y:%d    ",value);
               dat=digitalRead(1);
               if(dat==HIGH)
                  printf("DO:%d\n",dat);
               if(dat==LOW)
                  printf("DO:%d\n",dat);
               delay(100);

               // analogWrite(BASE,value++);
               // printf("AOUT:%d\n",value++);
               // delay(50);
	}
}
