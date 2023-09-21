#include <wiringPi.h>
int main()
{
	wiringPiSetup();
	pinMode(1,OUTPUT);
	int i;
	for(;;)
	{
		
		for(i=0;i<50;i++)            
		{
		digitalWrite(1,HIGH);
		delayMicroseconds(1000);
		digitalWrite(1,LOW);
                delay(19);	
		}
		
		delay(1000);
		
		for(i=0;i<50;i++)         
		{
		digitalWrite(1,HIGH);
		delayMicroseconds(2000);
		digitalWrite(1,LOW);
	        delay(18);	
		}
                delay(1000);	
	}
	return 0;
}
