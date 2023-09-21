#include <wiringPi.h>
#include <wiringShift.h>
int dataPin = 23; //define three pins
int latchPin = 24;
int clockPin = 25;
int a[8]={
1,2,4,8,16,32,64,128};   //定义功能数组，数组依次为数码管得定义
int b[8]={
254,253,251,247,239,223,191,127};   //定义功能数组，数组依次为数码管得定义
int c[16]={
1,3,7,15,31,63,127,255,1,2,4,8,16,32,64,128};   //定义功能数组，数组依次为数码管得定义
int d[16]={
254,253,251,247,239,223,191,127,1,3,7,15,31,63,127,255};   //定义功能数组，数组依次为数码管得定义
int x;
int y;
int z;
int main()
{
  wiringPiSetup();
 
  {
  pinMode(latchPin,OUTPUT);
  pinMode(clockPin,OUTPUT);
  pinMode(dataPin,OUTPUT); //three pins as output
  }
  
  while(1)
  { 
   for(x=0; x<8 ;x++ )                        //顺数功能循环
   for(y=0; y<8 ;y++ )                        //顺数功能循环
   {
    digitalWrite(latchPin,LOW);
    shiftOut(dataPin,clockPin,MSBFIRST,a[x]);     //显示数组a[x]
    shiftOut(dataPin,clockPin,MSBFIRST,b[y]);     //显示数组b[y]
    digitalWrite(latchPin,HIGH);
    delay(80);
   }
    for(z=0; z<16 ;z++ )                        //顺数功能循环
  {
    digitalWrite(latchPin,LOW);
    shiftOut(dataPin,clockPin,MSBFIRST,c[z]);     //显示数组c[z]
    shiftOut(dataPin,clockPin,MSBFIRST,d[z]);     //显示数组d[z]
    digitalWrite(latchPin,HIGH);
    delay(80);
  }  
 }	
}

