 #include <stdio.h>
 #include <stdlib.h>
 #include <stdint.h>
 #include <wiringPi.h>
 
 #define LED 1

int main(void)
{
   int bright;
    printf("Raspberry Pi wiringPi PWM test program\n");
    if (wiringPiSetup() == -1)
     {
        printf("GPIO setup error!\n");
         exit(1);
    }
    pinMode(LED,PWM_OUTPUT);
     while(1)
    {
     for (bright = 0; bright < 1024; ++bright)
         {
            pwmWrite(LED,bright);
            printf("bright:%d\n",bright);
             delay(3);
        }
    for (bright = 1023; bright >= 0; --bright)
        {
             pwmWrite(LED,bright);
             printf("bright:%d\n",bright);
             delay(3);
         }
       }
     return 0;
 }