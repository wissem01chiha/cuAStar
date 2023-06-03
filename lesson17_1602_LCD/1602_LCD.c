#include <wiringPi.h>

int main()
{

//int RS=21,RW=22,EN=23;
//int DB0=3,DB1=4,DB2=5,DB3=6,DB4=7,DB5=8,DB6=9,DB7=10;
int DB0=0,DB1=1,DB2=2,DB3=3,DB4=4,DB5=5,DB6=6,DB7=7;
int RS=21,RW=22,EN=23;
//int i;
int i;

wiringPiSetup();

    
   
  //Serial.begin(9600);
      pinMode(RS,OUTPUT);
      pinMode(RW,OUTPUT);
      pinMode(EN,OUTPUT);
      pinMode(DB0,OUTPUT);
      pinMode(DB1,OUTPUT);
      pinMode(DB2,OUTPUT);
      pinMode(DB3,OUTPUT);
      pinMode(DB4,OUTPUT);
      pinMode(DB5,OUTPUT);
      pinMode(DB6,OUTPUT);
      pinMode(DB7,OUTPUT);       
     
  
  digitalWrite(RS,HIGH);
  digitalWrite(RW,LOW);
  digitalWrite(RS,LOW);
  digitalWrite(EN,LOW);
  delay(1);
   //digitalWrite(DB0,0);
   //digitalWrite(DB1,0);
   digitalWrite(DB2,0);
   digitalWrite(DB3,1);
   digitalWrite(DB4,1);
   digitalWrite(DB5,1);
   digitalWrite(DB6,0);
   digitalWrite(DB7,0);
  digitalWrite(EN,HIGH);
  
  //Serial.println("zzl");
   
   delay(1);
   digitalWrite(EN,LOW);
   digitalWrite(RS,HIGH);
   
   delay(5);
  /************************************/
  digitalWrite(RS,HIGH);
  digitalWrite(RW,LOW);
  digitalWrite(RS,LOW);
  digitalWrite(EN,LOW);
  delay(1);
   digitalWrite(DB0,LOW);
   digitalWrite(DB1,LOW);
   digitalWrite(DB2,HIGH);
   digitalWrite(DB3,HIGH);
   for(i=4;i<8;i++)
   { digitalWrite(i,LOW); }  
  digitalWrite(EN,HIGH);
  

   
   delay(1);
   digitalWrite(EN,LOW);
   digitalWrite(RS,HIGH);
   
   delay(5);
  
  
  
  /**************************************/
  digitalWrite(RS,HIGH);
  digitalWrite(RW,LOW);
  digitalWrite(RS,LOW);
  digitalWrite(EN,LOW);
  delay(1);
   digitalWrite(DB0,LOW);
   digitalWrite(DB1,HIGH);
   digitalWrite(DB2,HIGH);
   digitalWrite(DB3,LOW);
   for(i=4;i<8;i++)
   { digitalWrite(i,LOW); }  
  digitalWrite(EN,HIGH);
  

   
   delay(1);
   digitalWrite(EN,LOW);
   digitalWrite(RS,HIGH);
   
   delay(5);
   /*******************************************/
   
  digitalWrite(RS,HIGH);
  digitalWrite(RW,LOW);
  digitalWrite(RS,LOW);
  digitalWrite(EN,LOW);
  delay(1);
   digitalWrite(DB0,HIGH);
   for(i=1;i<8;i++)
   { digitalWrite(i,LOW); }  
  digitalWrite(EN,HIGH);
  

   
   delay(1);
   digitalWrite(EN,LOW);
   digitalWrite(RS,HIGH);
   
   delay(5);
   /*************************************************/
 // put your setup code here, to run once:



for (;;) 
{
  digitalWrite(RS,LOW);
  digitalWrite(RW,LOW);
  digitalWrite(RS,HIGH);
  digitalWrite(EN,LOW);
  delay(1);
     for(i=0;i<4;i++)
    {digitalWrite(i,LOW);}
    digitalWrite(DB4,HIGH);
    digitalWrite(DB5,HIGH);
    digitalWrite(DB6,LOW);
    digitalWrite(DB7,LOW);
  digitalWrite(EN,HIGH);
  

  
   
   delay(1);
   digitalWrite(EN,LOW);
   digitalWrite(RS,LOW);
   delay(100);
   //while(1);
   //delay(5000);
  // put your main code here, to run repeatedly:

}
return 0;
}

