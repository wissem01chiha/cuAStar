#include <wiringPi.h>
#include <pcf8591.h>
#include <stdio.h>

#define Address 0x48         //pcf8591器件地址
#define BASE 64
#define A0 BASE+0           //A0端口输入地址
#define A1 BASE+1           //A1端口输入地址
#define A2 BASE+2           //A2端口输入地址
#define A3 BASE+3           //A3端口输入地址

int main(void)
{
    unsigned char value;
	wiringPiSetup();
	pcf8591Setup(BASE,Address);        //配置pcf8591
	
	while(1)
	{
               value=analogRead(A0);     // 读取A0端口的值          
               printf("A0:%d\n",value);  // 终端打印出A0端口的值
               delay(100);	
	}
}
