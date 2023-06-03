#include <wiringPi.h>
#include <stdio.h>
#include <sys/time.h>

#define Trig    5
#define Echo    4

void ultraInit(void)
{
	pinMode(Echo, INPUT);
	pinMode(Trig, OUTPUT);
}

float disMeasure(void)
{
	struct timeval tv1;
	struct timeval tv2;
	long start, stop;
	float dis;

	digitalWrite(Trig, LOW);
	delayMicroseconds(2);

	digitalWrite(Trig, HIGH);
	delayMicroseconds(10);      	
    digitalWrite(Trig, LOW);
	
	while(!(digitalRead(Echo) == 1));
	gettimeofday(&tv1, NULL);           

	while(!(digitalRead(Echo) == 0));
	gettimeofday(&tv2, NULL);           

	start = tv1.tv_sec * 1000000 + tv1.tv_usec;   
	stop  = tv2.tv_sec * 1000000 + tv2.tv_usec;

	dis = (float)(stop - start) / 1000000 * 34000 / 2;  

	return dis;
}

int main(void)
{
	float dis;

	if(wiringPiSetup() == -1){ //when initialize wiring failed,print messageto screen
		printf("setup wiringPi failed !");
		return 1; 
	}

	ultraInit();
	
	while(1){
		dis = disMeasure();
		printf("distance = %0.2f cm\n",dis);
		delay(500);
	}

	return 0;
}
