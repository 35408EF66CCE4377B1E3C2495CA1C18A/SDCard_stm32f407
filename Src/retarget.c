#include  <stdio.h>
#include "usart.h"

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout; 
FILE __stdin;
FILE __stderr;
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
int _sys_exit(int x) 
{ 
	x = x; 
	return 0;
} 

void _ttywrch(int ch)
{
	ch = ch;
}


//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 10);
	return 0;
}
#endif

