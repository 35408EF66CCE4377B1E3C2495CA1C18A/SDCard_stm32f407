#include  <stdio.h>
#include "usart.h"

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout; 
FILE __stdin;
FILE __stderr;
//定义_sys_exit()以避免使用半主机模式    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 

void _ttywrch(int ch)
{
	ch = ch;
}


//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
	return 0;
}
#endif

