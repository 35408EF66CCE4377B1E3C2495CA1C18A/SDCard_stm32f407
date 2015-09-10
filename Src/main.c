/**
******************************************************************************
* File Name          : main.c
* Description        : Main program body
******************************************************************************
*
* COPYRIGHT(c) 2015 STMicroelectronics
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of STMicroelectronics nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "fatfs.h"
#include "rtc.h"
#include "sdio.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "includes.h"
#include "socket.h"
#include "wizchip_conf.h"
#include "loopback.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/////////////////////////////////////////////////
// Semaphore Used To Indicate Socket Interrupt //
/////////////////////////////////////////////////
OS_SEM semSOCKET_1;


/////////////////////////////////////////
// SOCKET NUMBER DEFINION for Examples //
/////////////////////////////////////////
#define SOCK_TCPS        0
#define SOCK_UDPS        1

////////////////////////////////////////////////
// Shared Buffer Definition for LOOPBACK TEST //
////////////////////////////////////////////////
#define DATA_BUF_SIZE   2048
uint8_t gDATABUF[DATA_BUF_SIZE];

///////////////////////////////////
// Memory for SDIO data exchange //
///////////////////////////////////
OS_MEM mem_for_sd;
CPU_INT08U mem_for_sd_storage[2][512];

///////////////////////////////////
// Default Network Configuration //
///////////////////////////////////
wiz_NetInfo gWIZNETINFO =
{
	.mac = {0x00, 0x08, 0xdc,0x00, 0xab, 0xcd},
	.ip = {192, 168, 1, 110},
	.sn = {255,255,255,0},
	.gw = {192, 168, 1, 1},
	.dns = {0,0,0,0},
	.dhcp = NETINFO_STATIC
};

wiz_NetInfo gWIZNETINFO_Check;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
void network_init(void);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/******************** Start Task  ********************/
#define START_TASK_PRIO		3
#define START_STK_SIZE 		128
OS_TCB StartTaskTCB;
CPU_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *p_arg);

/******************** LED0 Task ********************/
#define LED0_TASK_PRIO		4
#define LED0_STK_SIZE 		256
OS_TCB Led0TaskTCB;
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

/******************** LED1 Task ********************/
#define LED1_TASK_PRIO		5
#define LED1_STK_SIZE 		2048
OS_TCB Led1TaskTCB;
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
void led1_task(void *p_arg);



/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	OS_ERR err;
	CPU_SR_ALLOC();


	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_RTC_Init();
	MX_SDIO_SD_Init();
	MX_SPI2_Init();
	MX_USART1_UART_Init();
	MX_FATFS_Init();

	/* USER CODE BEGIN 2 */
	OSInit(&err);



	/* Create the memory for SDIO */
	/*
	OSMemCreate((OS_MEM*)&mem_for_sd,
	(CPU_CHAR*)"SDIO_MEM",
	(void*)&mem_for_sd_storage[0][0],
	(OS_MEM_QTY)2,
	(OS_MEM_SIZE)512,
	(OS_ERR*)&err);
	if(err != OS_ERR_NONE)
	{
	printf("Init memories for SDIO error, err:%d", err);
}
	*/

	OS_CRITICAL_ENTER();
	OSTaskCreate((OS_TCB*)		&StartTaskTCB,
				 (CPU_CHAR*)	"start task",
				 (OS_TASK_PTR)	start_task,
				 (void*)		0,
				 (OS_PRIO)		START_TASK_PRIO,
				 (CPU_STK*)		&START_TASK_STK[0],
				 (CPU_STK_SIZE)	START_STK_SIZE / 10,
				 (CPU_STK_SIZE)	START_STK_SIZE,
				 (OS_MSG_QTY)	0,
				 (OS_TICK)		0,
				 (void*)		0,
				 (OS_OPT)	OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR*)	&err);
	OS_CRITICAL_EXIT();
	OSStart(&err);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	__PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
		|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	OSIntEnter();
	OSTimeTick();
	OSIntExit();
}


/* Start user tasks */
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
	OSStatTaskCPUUsageInit(&err);
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
	CPU_IntDisMeasMaxCurReset();
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN

	// 浣胯兘鏃堕棿鐗囪疆杞皟搴﹀姛鑳�?,鏃堕棿鐗囬暱搴︿�?1涓郴缁熸椂閽熻妭鎷嶏紝鏃�1*5=5ms
	OSSchedRoundRobinCfg(DEF_ENABLED, 1, &err);
#endif

	// Enter the critical segment
	OS_CRITICAL_ENTER();

	// Create user task - LED0
	OSTaskCreate(	(OS_TCB *)		&Led0TaskTCB,
				 (CPU_CHAR*)		"led0 task",
				 (OS_TASK_PTR)		led0_task,
				 (void*)			0,
				 (OS_PRIO)			LED0_TASK_PRIO,
				 (CPU_STK*)			&LED0_TASK_STK[0],
				 (CPU_STK_SIZE)		LED0_STK_SIZE / 10,
				 (CPU_STK_SIZE)		LED0_STK_SIZE,
				 (OS_MSG_QTY)		0,
				 (OS_TICK)			0,
				 (void*)			0,
				 (OS_OPT)			OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR*)			&err);

	// Create user task - LED1
	OSTaskCreate(	(OS_TCB *)		&Led1TaskTCB,
				 (CPU_CHAR*)		"led1 task",
				 (OS_TASK_PTR)		led1_task,
				 (void*)			0,
				 (OS_PRIO)			LED1_TASK_PRIO,
				 (CPU_STK*)			&LED1_TASK_STK[0],
				 (CPU_STK_SIZE)		LED1_STK_SIZE/10,
				 (CPU_STK_SIZE)		LED1_STK_SIZE,
				 (OS_MSG_QTY)		0,
				 (OS_TICK)			0,
				 (void*)			0,
				 (OS_OPT)			OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR,
				 (OS_ERR*)			&err);

	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);

	// Exit the critical segment
	OS_CRITICAL_EXIT();
}


//led1浠诲姟鍑芥暟
void led0_task(void *p_arg)
{
	uint8_t memsize[2][8] = {{2,2,2,2,2,2,2,2},{2,2,2,2,2,2,2,2}};
	intr_kind sirm = IK_SOCK_1;
	uint8_t tmp;
	int32_t ret = 0;
	OS_ERR err;

	p_arg = p_arg;

	printf("\r\n+++++++++++++++++++++++++\r\n");
	printf("Start W5500 thread!\r\n");


	/* Create the semaphore for Socket 1 interrupt */
	OSSemCreate(&semSOCKET_1, "SEM SOCKET 1", 0, &err);
	if(err != OS_ERR_NONE)
	{
		printf("Semaphore SOCKET 1 Initialized fail. Err %d\r\n", err);
		while(1);
	}

	reg_wizchip_spi_cbfunc(0x0, 0x0);
	reg_wizchip_spiburst_cbfunc(0x0, 0x0);

	/* WIZCHIP SOCKET Buffer initialize */
	if(ctlwizchip(CW_INIT_WIZCHIP,(void*)memsize) == -1)
	{
		printf("WIZCHIP Initialized fail.\r\n");
		while(1);
	}

	/* WIZCHIP SOCKET interrupt mask initialize */
	if(ctlwizchip(CW_SET_INTRMASK, &sirm))
	{
		printf("WIZCHIP Socket Interrupt Mask Initialized fail.\r\n");
		while(1);
	}
	else
	{
		sirm = (intr_kind)0x0;
		ctlwizchip(CW_GET_INTRMASK, &sirm);
		printf("IMR: %d\r\n", sirm);
	}

	/* Set SnIMR */
	setSn_IMR(SOCK_UDPS, SIK_RECEIVED);

	/* PHY link status check */
	tmp = 0;
	do
	{
		if(tmp == 0)
			printf("Wait for PHY Link...\r\n");
		tmp = 1;
		if(ctlwizchip(CW_GET_PHYLINK, (void*)&ret) == -1)
			printf("Unknown PHY Link stauts.\r\n");

		OSTimeDly(5, OS_OPT_TIME_DLY, &err);
	}while(ret == PHY_LINK_OFF);

	/* Network initialization */
	network_init();

	/* W5500 Registers initializition */
	//iinchip_init();

	/* get the value from VERSION register */
	//w5500_reg_val = getVersion();
#ifdef __DEBUG_INFO__
	//printf("Chip Version:%d\r\n", w5500_reg_val);
#endif

	//config_w5500_ip();

	/* Open UDP port */

	if((ret = socket(SOCK_UDPS, Sn_MR_UDP, 3000, 0x00)) < 0)
	{
		printf("Open UDP SOCKET ERROR : %ld\r\n", ret);
	}
	else
	{
		printf("UDP Opened, udp port(3000)\r\n");
	}

	while(1)
	{
		//printf("Wait for the data...\r\n");
		OSSemPend(&semSOCKET_1, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
		//printf("Start the loopback process...\r\n");

		/* Who raised the interrupt? */
		ctlwizchip(CW_GET_INTERRUPT, &sirm);
		if((sirm & (IK_WOL | IK_PPPOE_TERMINATED | IK_IP_CONFLICT | IK_DEST_UNREACH)) != 0)	// Common Interrupt
		{
			sirm = (IK_WOL | IK_PPPOE_TERMINATED | IK_IP_CONFLICT | IK_DEST_UNREACH);
			ctlwizchip(CW_CLR_INTERRUPT, &sirm);
		}
		else // Socket Interrupt
		{
			/* Which Socket raised the interrupt? */
			if(sirm & IK_SOCK_1)
			{
				/* What kinds of interrupt had been raised? */
				ret = getSn_IR(SOCK_UDPS);
				if(ret & SIK_RECEIVED)
				{
					// Get and send data from RX buffer
					if( (ret = loopback_udps(SOCK_UDPS, gDATABUF, 3000)) < 0)
					{
						printf("Receive Data from UDP SOCKET ERROR : %ld\r\n", ret);
					}

					setSn_IR(SOCK_UDPS, SIK_RECEIVED); // Clear the Received Interrupt
				}

				sirm = IK_SOCK_1;
				ctlwizchip(CW_CLR_INTERRUPT, &sirm); // Clear the IR bit in the Common Interrupt Register
			}
		}

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_RESET);
		OSTimeDly(10, OS_OPT_TIME_DLY, &err);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, GPIO_PIN_SET);
		//OSTimeDly(1, OS_OPT_TIME_DLY, &err);
	}
}

//led1 user task
void led1_task(void *p_arg)
{
	OS_ERR err;

	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;

	/*
	CPU_INT08U *sd_mem;
	HAL_SD_CardInfoTypedef sdcard_info;
	HAL_SD_ErrorTypedef sd_ret;
	*/

	uint32_t thread_cnt = 0;
	uint32_t wbytes;
	uint8_t wtext[] = "abcabc12345";
	FATFS sddisk_fatfs;
	FIL file;
	FRESULT ret;

	p_arg = p_arg;


	/************** SDIO basic function test **************

	sd_mem = OSMemGet((OS_MEM*)&mem_for_sd, &err);
	memset(sd_mem, 0xcc, 512);

	if(err == OS_ERR_NONE)
	{
	sd_ret = HAL_SD_Init(&hsd, &sdcard_info);
	if(sd_ret == SD_OK)
	{
	printf("Start SDIO dma transmission\r\n");
	HAL_SD_ReadBlocks(&hsd, (uint32_t*)sd_mem, 0, 512, 1);
}
		else
	{
	printf("Init SD Card Error, %d\r\n", sd_ret);
}
}

	******************************************************/

	/************** FATFS function test **************/
	sprintf((char*)wtext, "%d", thread_cnt);
	if((ret = f_mount(&sddisk_fatfs, (const TCHAR*)SD_Path, 0)) == FR_OK)
	{
		if((ret = f_open(&file, "test.txt", FA_OPEN_EXISTING | FA_WRITE)) == FR_OK)
		{
			if((ret = f_write(&file, wtext, sizeof(wtext),  &wbytes)) == FR_OK)
			{
				if((ret = f_close(&file)) == FR_OK)
				{
					printf("SD Card thread cnt = %ld\r\n", thread_cnt);
				}
				else
				{
					printf(" ** f_close() err: %d\r\n", ret);
				}

			}
			else
			{
				printf(" ** f_write() err: %d\r\n", ret);
			}
		}
		else
		{
			printf(" ** f_open() err: %d\r\n", ret);
		}
	}
	else
	{
		printf(" ** f_mount() err: %d\r\n", ret);
	}



	thread_cnt++;
	/***************************************************/
	while(1)
	{
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &date, RTC_FORMAT_BIN);

		printf("\r\n>>>>> %04d-%02d-%02d %02d:%02d:%02d.%d\r\n",
			   date.Year,
			   date.Month,
			   date.Date,
			   time.Hours,
			   time.Minutes,
			   time.Seconds,
			   time.SubSeconds);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);
		OSTimeDly(10, OS_OPT_TIME_DLY, &err);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);
		OSTimeDly(990, OS_OPT_TIME_DLY, &err);


	}
}

/////////////////////////////////////////////////////////////
// Intialize the network information to be used in WIZCHIP //
/////////////////////////////////////////////////////////////
void network_init(void)
{
	uint8_t tmpstr[6];

	ctlnetwork(CN_SET_NETINFO, (void*)&gWIZNETINFO);

	memset(&gWIZNETINFO_Check, 0x55, sizeof(wiz_NetInfo));

	ctlnetwork(CN_GET_NETINFO, (void*)&gWIZNETINFO_Check);

	// Display Network Information
	ctlwizchip(CW_GET_ID,(void*)tmpstr);
	printf("\r\n=== %s NET CONF ===\r\n",(char*)tmpstr);
	printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
		   gWIZNETINFO_Check.mac[0],
		   gWIZNETINFO_Check.mac[1],
		   gWIZNETINFO_Check.mac[2],
		   gWIZNETINFO_Check.mac[3],
		   gWIZNETINFO_Check.mac[4],
		   gWIZNETINFO_Check.mac[5]);

	printf("SIP: %d.%d.%d.%d\r\n",
		   gWIZNETINFO_Check.ip[0],
		   gWIZNETINFO_Check.ip[1],
		   gWIZNETINFO_Check.ip[2],
		   gWIZNETINFO_Check.ip[3]);

	printf("GAR: %d.%d.%d.%d\r\n",
		   gWIZNETINFO_Check.gw[0],
		   gWIZNETINFO_Check.gw[1],
		   gWIZNETINFO_Check.gw[2],
		   gWIZNETINFO_Check.gw[3]);

	printf("SUB: %d.%d.%d.%d\r\n",
		   gWIZNETINFO_Check.sn[0],
		   gWIZNETINFO_Check.sn[1],
		   gWIZNETINFO_Check.sn[2],
		   gWIZNETINFO_Check.sn[3]);

	printf("DNS: %d.%d.%d.%d\r\n",
		   gWIZNETINFO_Check.dns[0],
		   gWIZNETINFO_Check.dns[1],
		   gWIZNETINFO_Check.dns[2],
		   gWIZNETINFO_Check.dns[3]);
	printf("======================\r\n");
}
/////////////////////////////////////////////////////////////

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	OS_ERR err;

	OSIntEnter();

	if(GPIO_Pin == GPIO_PIN_4)
	{

		// TODO Socket Interrupt
		OSSemPost(&semSOCKET_1, OS_OPT_POST_1, &err);
	}

	OSIntExit();
}



/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
* @brief Reports the name of the source file and the source line number
* where the assert_param error has occurred.
* @param file: pointer to the source file name
* @param line: assert_param error line source number
* @retval None
*/
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
