/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "g711.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_AT_COMAND //IF NON DEFINED YOU CAN USE UART TO RECORD!!!
#define conect_to_AP "AT+CWJAP=\"360WiFi\",\"amkhalyawa\"\r\n"
//#define conect_to_AP "AT+CWJAP=\"HUAWEI-q36B\",\"48575443D9E41A95\"\r\n"
//48575443D9E41A95
#define serverPort 1121
//#define server_Ip "192.168.100.65"
#define server_Ip "192.168.1.176"
#define stream_pac_size 1024//1024
#define DEBUG 1
#define ESP01_RESET()              LL_GPIO_ResetOutputPin(ESP01_RESET_GPIO_Port,ESP01_RESET_Pin);
#define ESP01_NORESET()            LL_GPIO_SetOutputPin(ESP01_RESET_GPIO_Port,ESP01_RESET_Pin);
//USART_InitStruct.BaudRate = baud; must be in MX_USART1_UART_Init();
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char rx_buf[RX_BUF_SIZE]="";
volatile uint16_t rx_pos=0;
uint16_t adc_buffer[2][stream_pac_size/2];
uint8_t g711_buffer[stream_pac_size/2];
const char uartFastbaud [] =     "AT+UART_CUR=2000000,8,1,0,0\r\n" ;
const char at_test[] =           "AT\r\n";
const char echoOff[] =           "ATE0\r\n";
const char station_mode [] =     "AT+CWMODE=1\r\n"; //station mode
const char conect_AP_status [] = "AT+CWJAP?\r\n";
const char get_IP [] =           "AT+CIFSR\r\n";
const char NoMultConn[] =        "AT+CIPMUX=0\r\n";
uint8_t ready_buf=0;
uint8_t send_buf=1;
char sendbuf[50];
uint32_t baud=115200;
uint32_t i;
volatile uint32_t currentTimeMs=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
//void MX_USART1_UART_Init(uint32_t baudrate);
void usart_send_byte(char chr);
void usart_send_string(const char *str);
void clear_rx_buf(void);
void connect_to_server(void);
uint8_t check_answer(char *str,uint16_t wait);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//==================================SWO_ini======================================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA           0x01000000

struct __FILE { int handle;};
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) {
   if (DEMCR & TRCENA) {

while (ITM_Port32(0) == 0){};
    ITM_Port8(0) = ch;
  }
  return(ch);
}
char *crcOK;


//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=================================================================================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_AFIO);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled 
  */
  LL_GPIO_AF_Remap_SWJ_NOJTAG();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  LL_SYSTICK_EnableIT();
	//LL_USART_EnableIT_RXNE(USART1);//in uart ini
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

#if defined(USE_AT_COMAND)
	LL_mDelay(500);
	ESP01_RESET();
	LL_mDelay(200);
	ESP01_NORESET();
	LL_mDelay(3000);//delite startup log
	clear_rx_buf();
	while(1){
			if(DEBUG)printf("while ESP01 module ready\r\n");	
			usart_send_string(at_test);	
			if (check_answer("OK",500))break;
			}clear_rx_buf();
		//if(DEBUG)printf("rx_buf %s\r\n",rx_buf);
		connect_to_server();
#endif /* USE_AT_COMAND */
//    baud = 2000000;			
//    MX_USART1_UART_Init();		
//========================================DMA_CHANNEL_1=======================================================================//			
LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, stream_pac_size);
LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_1,(uint32_t)&ADC1->DR,(uint32_t)&adc_buffer[0],LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_1);
LL_DMA_EnableIT_HT(DMA1,LL_DMA_CHANNEL_1);
LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
//========================================DMA_CHANNEL_4======================================================================//					
LL_USART_EnableDMAReq_TX(USART1);						
//LL_DMA_ConfigAddresses(DMA1,LL_DMA_CHANNEL_4,(uint32_t)g711_buffer,(uint32_t)&USART1->DR,LL_DMA_DIRECTION_MEMORY_TO_PERIPH);				
LL_DMA_EnableIT_TC(DMA1,LL_DMA_CHANNEL_4);
LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_4,(uint32_t)&USART1->DR);			
LL_DMA_SetMemoryAddress(DMA1,LL_DMA_CHANNEL_4,(uint32_t)g711_buffer);	
LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, stream_pac_size/2);		
//LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_4);			
//============================================ADC1============================================================================//				
LL_ADC_Enable(ADC1);
LL_ADC_REG_StartConversionExtTrig(ADC1,LL_ADC_REG_TRIG_EXT_RISING);
//============================================TIM3============================================================================//	
LL_TIM_EnableUpdateEvent(TIM3);
LL_TIM_EnableIT_UPDATE(TIM3);
LL_TIM_EnableCounter(TIM3);		
//============================================================================================================================//	
ESP01_NORESET();
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		
		if(ready_buf!=send_buf)	{
		   //if(DEBUG)printf("start tx %d ms\r\n",currentTimeMs);		
					 //for(i=0;i<sream_pac_size/2;i++){
					 // usart_send_byte(linear2alaw(adc_buffer[ready_buf][i]));}
			     //printf(" end tx %d ms\r\n",currentTimeMs);
			if(DEBUG)printf("start linear2alaw %d ms\r\n",currentTimeMs);		
			for(i=0;i<stream_pac_size/2;i++)g711_buffer[i]=linear2alaw(adc_buffer[ready_buf][i]);
			if(DEBUG)printf("end linear2alaw %d ms\r\n",currentTimeMs);
			LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_4, stream_pac_size/2);	
			LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_4);
		  send_buf=ready_buf;
	  }
					
		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
    Error_Handler();  
  }
  LL_RCC_HSE_Enable();

   /* Wait till HSE is ready */
  while(LL_RCC_HSE_IsReady() != 1)
  {
    
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(72000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(72000000);
  LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration  
  PB0   ------> ADC1_IN8 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 DMA Init */
  
  /* ADC1 Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
  ADC_InitStruct.SequencersScanMode = LL_ADC_SEQ_SCAN_DISABLE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM3_TRGO;
  ADC_REG_InitStruct.SequencerLength = 1;
  ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  /** Configure Regular Channel 
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_8);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_8, LL_ADC_SAMPLINGTIME_1CYCLE_5);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  LL_TIM_InitTypeDef TIM_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM3);

  /* TIM3 interrupt Init */
  NVIC_SetPriority(TIM3_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  TIM_InitStruct.Prescaler = 0;
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 8999;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  LL_TIM_Init(TIM3, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM3);
  LL_TIM_SetClockSource(TIM3, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM3, LL_TIM_TRGO_UPDATE);
  LL_TIM_DisableMasterSlaveMode(TIM3);
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  LL_USART_InitTypeDef USART_InitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Peripheral clock enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
  /**USART1 GPIO Configuration  
  PA9   ------> USART1_TX
  PA10   ------> USART1_RX 
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_9;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_10;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USART1 DMA Init */
  
  /* USART1_TX Init */
  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_4, LL_DMA_DIRECTION_MEMORY_TO_PERIPH);

  LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MODE_NORMAL);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_PDATAALIGN_BYTE);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_4, LL_DMA_MDATAALIGN_BYTE);

  /* USART1 interrupt Init */
  NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(USART1_IRQn);

  /* USER CODE BEGIN USART1_Init 1 */
  LL_USART_EnableIT_RXNE(USART1);
  /* USER CODE END USART1_Init 1 */
  USART_InitStruct.BaudRate = baud;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  LL_USART_Init(USART1, &USART_InitStruct);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_Enable(USART1);
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Channel4_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);

  /**/
  LL_GPIO_ResetOutputPin(ESP01_RESET_GPIO_Port, ESP01_RESET_Pin);

  /**/
  GPIO_InitStruct.Pin = ESP01_RESET_Pin;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_OPENDRAIN;
  LL_GPIO_Init(ESP01_RESET_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=================================usart1_send_byte==============================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void usart_send_byte (char chr){
	      USART1->DR = chr;
	       while (!LL_USART_IsActiveFlag_TC(USART1));
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//=================================usart1_send_string============================//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
void usart_send_string(const char* str) {
	//clear_rx_buf();
  while(*str){	
		usart_send_byte(*str++);
	}
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
//==============================check_answer================================================//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
uint8_t check_answer(char *str,uint16_t wait){
uint32_t time_Ms_Now = currentTimeMs;
char *istr = NULL;
	while(istr==NULL && ((currentTimeMs-time_Ms_Now)< wait)){ 
			istr=strstr(rx_buf,str);
		} 
		if(istr!=NULL){
			if(DEBUG)printf("find %s;\r\n",str);
			clear_rx_buf();
			return 1;	
		}else return 0;
	}
//=========================================================================//
//-----------------------------clear_rx_buf--------------------------------//
//=========================================================================//
void clear_rx_buf(void){
 for(;rx_pos!=0;rx_pos--)rx_buf[rx_pos]=0xFF;
// for(i=RX_BUF_SIZE;i!=0;i--)
//	rx_buf[i]='\0';
}
//=========================================================================================//
//+++++++++++++++++++++++configure ESP and connect to the server++++++++++++++++++++++++++++//
//=========================================================================================//
#if defined(USE_AT_COMAND)
void connect_to_server(void){
uint8_t step=0;
	
while(1){
//This at command checked at !!!!	
	switch(step){
		case(0):
			if(DEBUG)printf("step 0: send uartFastbaud tx\r\n");
			usart_send_string(uartFastbaud);//2000000 DCMI_CaptureRate_1of2_Frame;4608000 DCMI_CaptureRate_All_Frame;
			if (check_answer("OK",1000)){
				baud = 2000000;
				LL_USART_DeInit(USART1);
				MX_USART1_UART_Init();//2000000 DCMI_CaptureRate_1of2_Frame;4608000 DCMI_CaptureRate_All_Frame;
				step++;
			}
		break;
		case(1):
			if(DEBUG)printf("step 1: send_string(echoOff)\r\n");
			usart_send_string(echoOff);
			if (check_answer("OK",1000))step++;
		break;
		case(2):
			if(DEBUG)printf("step 2: send_string(station_mode)\r\n");
			usart_send_string(station_mode);
			if (check_answer("OK",1000))step++;
		break;
		case(3):
			if(DEBUG)printf("step 3: send_string(conect_to_AP)\r\n");	
		  usart_send_string(conect_to_AP);
			if (check_answer("WIFI CONNECTED",5000))step++;
		break;
		case(4):			
			if(DEBUG)printf("step 4: send_string(conect_to_AP_status)\r\n");	
			usart_send_string(conect_AP_status);
		  if (check_answer("OK",5000))step++;
		break;
		case(5):
			if(DEBUG)printf("step 5: send_string(get_IP)\r\n");
			usart_send_string(get_IP);
		  if (check_answer("OK",1000))step++;
		break;
		case(6):
			if(DEBUG)printf("step 6: send_string(NoMultConn)\r\n");
			usart_send_string(NoMultConn);
		  if (check_answer("OK",1000))step++;
		break;
		case(7):
			 sprintf(sendbuf,"%s%s%s%d%s","AT+CIPSTART=\"TCP\",\"",server_Ip,"\",",serverPort,"\r\n");	
			 if(DEBUG)printf("step 7: connect to the server tx: %s\r\n",sendbuf);
			 usart_send_string(sendbuf);          //connect to the server 
			 if(check_answer("CONNECT",3000))step++;
		break;	
  case(8):
    if(DEBUG)printf("step 8: enable transparent transmission mode tx\r\n");
	  usart_send_string("AT+CIPMODE=1\r\n");//enable transparent transmission mode
	  if(check_answer("OK",1000))step++;
	  usart_send_string("AT+CIPSEND\r\n");//stat transmitt
  break;
	}
LL_mDelay(500);
if (step==9)break;
		LL_mDelay(200);
 }
}
#endif /* USE_AT_COMAND */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
