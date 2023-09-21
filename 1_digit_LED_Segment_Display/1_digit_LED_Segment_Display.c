#include <wiringPi.h>
int a=27;//GPIO16
int b=26;//GPIO12
int c=23;//GPIO13
int d=24;//GPIO19
int e=25;//GPIO26
int f=28;// GPIO20
int g=29;//GPIO21
int dp=22;//GPIO6
int i; 
void digital_0()//0
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,HIGH);
digitalWrite(f,HIGH);
digitalWrite(g,LOW);
digitalWrite(dp,LOW);
}
void digital_1()//1
{
digitalWrite(a,LOW);
digitalWrite(b,HIGH); 
digitalWrite(c,HIGH); 
digitalWrite(d,LOW);
digitalWrite(e,LOW); 
digitalWrite(f,LOW);
digitalWrite(g,LOW);
digitalWrite(dp,LOW); 
}
void digital_2()//2
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,LOW);
digitalWrite(d,HIGH);
digitalWrite(e,HIGH);
digitalWrite(f,LOW);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
}
void digital_3()//3
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,LOW);
digitalWrite(f,LOW);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
}
void digital_4()//4
{
digitalWrite(a,LOW);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,LOW);
digitalWrite(e,LOW);
digitalWrite(f,HIGH);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
}
void digital_5()//5
{
digitalWrite(a,HIGH);
digitalWrite(b,LOW);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,LOW);
digitalWrite(f,HIGH);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
digitalWrite(e,LOW);
}
void digital_6()//6
{
digitalWrite(a,HIGH);
digitalWrite(b,LOW);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,HIGH);
digitalWrite(f,HIGH);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);

}
void digital_7()//7
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,LOW);
digitalWrite(e,LOW);
digitalWrite(f,LOW);
digitalWrite(g,LOW);
digitalWrite(dp,LOW);
}
void digital_8()//8
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,HIGH);
digitalWrite(f,HIGH);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
}
void digital_9()//9
{
digitalWrite(a,HIGH);
digitalWrite(b,HIGH);
digitalWrite(c,HIGH);
digitalWrite(d,HIGH);
digitalWrite(e,LOW);
digitalWrite(f,HIGH);
digitalWrite(g,HIGH);
digitalWrite(dp,LOW);
}
int main()
{
 wiringPiSetup();
{
for(i=22;i<=29;i++)
pinMode(i,OUTPUT); 
 }
 while(1)
  { 
  digital_0();//0
  delay(1000);
  digital_1();//1
  delay(1000);
  digital_2();//2
  delay(1000); 
  digital_3();//3
  delay(1000); 
  digital_4();//4
  delay(1000); 
  digital_5();//5
  delay(1000); 
  digital_6();//6
  delay(1000); 
  digital_7();//7
  delay(1000); 
  digital_8();//8
  delay(1000);
  digital_9();//9
  delay(1000);
  } 
}