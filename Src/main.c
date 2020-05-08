//****************************************************************************************************
//������������ ����������
//****************************************************************************************************
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cdisplaynokia5110.h"
#include "cdisplaystandardlibrary.h"
#include "ff.h"
#include "string.h"
#include "sd.h"

#include <math.h>
#include <stdlib.h>

//****************************************************************************************************
//����������������
//****************************************************************************************************

#define DRIVE_OUTPUT_GPIO_PIN_INDEX		GPIO_PIN_14
#define DRIVE_OUTPUT_GPIO_INDEX				GPIOC

#define DRIVE_OUTPUT_GPIO_PIN_TRACK0	GPIO_PIN_13
#define DRIVE_OUTPUT_GPIO_TRACK0			GPIOC

//#define DRIVE_OUTPUT_GPIO_PIN_CHANGE	GPIO_PIN_2
//#define DRIVE_OUTPUT_GPIO_CHANGE			GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_DIR			GPIO_PIN_3
#define DRIVE_OUTPUT_GPIO_DIR					GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_READY		GPIO_PIN_2
#define DRIVE_OUTPUT_GPIO_READY				GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_STEP		GPIO_PIN_0
#define DRIVE_OUTPUT_GPIO_STEP				GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_SIDE1		GPIO_PIN_1
#define DRIVE_OUTPUT_GPIO_SIDE1				GPIOA

#define MAX_TRACK 79

#define TRACK_SIZE 12800

//****************************************************************************************************
//���������
//****************************************************************************************************

//****************************************************************************************************
//���������� ����������
//****************************************************************************************************
SPI_HandleTypeDef hspi2;//SPI
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

CDisplayNokia5110 cDisplay;//�������
IDisplay *iDisplay_Ptr=&cDisplay;//��������� �� �������
CDisplayStandardLibrary cDisplayStandardLibrary(iDisplay_Ptr,true);//����������� ���������� �������
static FATFS FatFS; 
static FIL File;

volatile bool Update=true;
volatile uint8_t Track=0;//����� �����
volatile uint8_t Side=0;//����� �������

static uint8_t TrackBuffer[TRACK_SIZE];

//****************************************************************************************************
//��������� �������
//****************************************************************************************************

static void RCC_Init(void);//������������� RCC
static void SPI2_Init(void);//������������� SPI2
static void DMA_Init(void);//������������� DMA
static void GPIO_Init(void);//������������� ������

void FindSD(void);//����� � ������������� SD-�����

void DRIVE_INDEX_One(void);//���������� Index � 1
void DRIVE_INDEX_Zero(void);//���������� Index � 0

void DRIVE_TRACK0_One(void);//���������� Track0 � 1
void DRIVE_TRACK0_Zero(void);//���������� Track0 � 0

void DRIVE_READY_One(void);//���������� Ready � 1
void DRIVE_READY_Zero(void);//���������� Ready � 0

bool DRIVE_DIR_Get(void);//�������� �������� Dir
bool DRIVE_SIDE1_Get(void);//�������� �������� Side1
bool DRIVE_STEP_Get(void);//�������� �������� Step

void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin);//���������� �������� ����������

//****************************************************************************************************
//���������� �������
//****************************************************************************************************

//----------------------------------------------------------------------------------------------------
//������� ������� ���������
//----------------------------------------------------------------------------------------------------
int main(void)
{ 	
 HAL_Init();
 RCC_Init();
 GPIO_Init();
 DMA_Init();
 SPI2_Init();
 iDisplay_Ptr->Init();
 cDisplayStandardLibrary.Clear(IDisplay::COLOR_WHITE);
	
 FindSD();
	
 if (f_open(&File,"Dizzy-6.MFM",FA_READ)!=FR_OK)
 {
  cDisplayStandardLibrary.Print("��� �����!",IDisplay::COLOR_BLACK);	 
	while(1);
 }
 cDisplayStandardLibrary.Print("���� ������",IDisplay::COLOR_BLACK);	  
 
 Update=true;
 Track=0;//����� �����
 Side=0;//����� �������  
 
 DRIVE_INDEX_One();
 DRIVE_READY_Zero(); 
 DRIVE_TRACK0_Zero();
 
 uint8_t last_side=0;
 uint32_t counter=0;
 //��������� ����� ������ � SPI
 memset(TrackBuffer,0,TRACK_SIZE);
 HAL_SPI_Transmit_DMA(&hspi2,TrackBuffer,TRACK_SIZE);	
 
 size_t track_offset=0;
 
 while(1)
 {
	__disable_irq();
	bool update=Update;
  Update=false;
  uint8_t track=Track;
  __enable_irq();
  uint8_t side;	 
  if (DRIVE_SIDE1_Get()==true) side=0;
                          else side=1;
  if (side!=last_side) update=true;	 
	last_side=side;
	if (update==true)//��������� ���������� ������
	{   		
   memset(TrackBuffer,0,TRACK_SIZE);
	 uint32_t offset=track;
	 offset*=2;
	 offset+=side;
	 offset*=TRACK_SIZE;
	 f_lseek(&File,offset);
	 counter=12800/512;
	 track_offset=0;		
		
	 char str[55];
	 sprintf(str,"T:%i S:%i",track,side);
	 cDisplayStandardLibrary.Print(str,IDisplay::COLOR_BLACK);
		
	 HAL_SPI_DMAStop(&hspi2);	 
	 //HAL_SPI_Transmit_DMA(&hspi2,TrackBuffer,TRACK_SIZE);
	}		
	if (counter>0)
	{
   //������������� ����� � SPI
	 UINT readen;
   if (f_read(&File,TrackBuffer+track_offset,sizeof(uint8_t)*512,&readen)!=FR_OK)
	 {
	  cDisplayStandardLibrary.Print("������ SD",IDisplay::COLOR_BLACK);	 
		while(1);
	 }
	 if (readen!=512)
	 {
	  cDisplayStandardLibrary.Print("�� ������",IDisplay::COLOR_BLACK);	 
		while(1);
	 }
	 counter--;
	 track_offset+=512;
	 if (counter==0) HAL_SPI_Transmit_DMA(&hspi2,TrackBuffer,TRACK_SIZE);
	}	
 }
 f_close(&File); 
}
//----------------------------------------------------------------------------------------------------
//������������� ��������� ����������
//----------------------------------------------------------------------------------------------------
void RCC_Init(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct;
 RCC_ClkInitTypeDef RCC_ClkInitStruct;

 RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState=RCC_HSE_ON;
 RCC_OscInitStruct.HSEPredivValue=RCC_HSE_PREDIV_DIV1;
 RCC_OscInitStruct.HSIState=RCC_HSI_ON;
 RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLMUL=RCC_PLL_MUL16;
 //RCC_OscInitStruct.PLL.PLLMUL=RCC_PLL_MUL9;	
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK)
 {
  _Error_Handler(__FILE__,__LINE__);
 }

 RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV1;

 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!=HAL_OK)
 {
  _Error_Handler(__FILE__,__LINE__);
 }
 HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
 HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
 HAL_NVIC_SetPriority(SysTick_IRQn,0,0); 
}

//----------------------------------------------------------------------------------------------------
//������������� SPI2
//----------------------------------------------------------------------------------------------------
static void SPI2_Init(void)
{
 hspi2.Instance=SPI2;
 hspi2.Init.Mode=SPI_MODE_MASTER;
 hspi2.Init.Direction=SPI_DIRECTION_2LINES;
 hspi2.Init.DataSize=SPI_DATASIZE_8BIT;
 hspi2.Init.CLKPolarity=SPI_POLARITY_LOW;
 hspi2.Init.CLKPhase=SPI_PHASE_1EDGE;
 hspi2.Init.NSS=SPI_NSS_SOFT;
 hspi2.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_128;
 hspi2.Init.FirstBit=SPI_FIRSTBIT_MSB;
 hspi2.Init.TIMode=SPI_TIMODE_DISABLE;
 hspi2.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
 hspi2.Init.CRCPolynomial=10;
 if (HAL_SPI_Init(&hspi2)!=HAL_OK)
 {
  _Error_Handler(__FILE__,__LINE__);
 }
}
//----------------------------------------------------------------------------------------------------
//������������� DMA
//----------------------------------------------------------------------------------------------------
static void DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}


//----------------------------------------------------------------------------------------------------
//������������� ������
//----------------------------------------------------------------------------------------------------
static void GPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct;
	
 /* GPIO Ports Clock Enable */
 __HAL_RCC_GPIOD_CLK_ENABLE();
 __HAL_RCC_GPIOA_CLK_ENABLE();
 __HAL_RCC_GPIOB_CLK_ENABLE();
 __HAL_RCC_GPIOC_CLK_ENABLE();
	
 //������
 GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
 GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_HIGH;
	
 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_INDEX;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_INDEX,&GPIO_InitStruct);
	
 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_TRACK0;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_TRACK0,&GPIO_InitStruct);
	
 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_READY;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_READY,&GPIO_InitStruct);
 //�����
 GPIO_InitStruct.Mode=GPIO_MODE_INPUT;
 GPIO_InitStruct.Pull=GPIO_NOPULL;
 
 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_DIR;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_DIR,&GPIO_InitStruct);

 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_SIDE1;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_SIDE1,&GPIO_InitStruct);

 //������� ����������
 GPIO_InitStruct.Mode=GPIO_MODE_IT_RISING;
 GPIO_InitStruct.Pull=GPIO_NOPULL;

 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_STEP;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_STEP,&GPIO_InitStruct);
 //���������� ����������
 HAL_NVIC_SetPriority(EXTI0_IRQn,0,0);
 HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

//----------------------------------------------------------------------------------------------------
//���������� Index � 1
//----------------------------------------------------------------------------------------------------
void DRIVE_INDEX_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_INDEX,DRIVE_OUTPUT_GPIO_PIN_INDEX,GPIO_PIN_SET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Index � 0
//----------------------------------------------------------------------------------------------------
void DRIVE_INDEX_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_INDEX,DRIVE_OUTPUT_GPIO_PIN_INDEX,GPIO_PIN_RESET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Track0 � 1
//----------------------------------------------------------------------------------------------------
void DRIVE_TRACK0_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_TRACK0,DRIVE_OUTPUT_GPIO_PIN_TRACK0,GPIO_PIN_SET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Track0 � 0
//----------------------------------------------------------------------------------------------------
void DRIVE_TRACK0_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_TRACK0,DRIVE_OUTPUT_GPIO_PIN_TRACK0,GPIO_PIN_RESET);
}
//----------------------------------------------------------------------------------------------------
//���������� Ready � 1
//----------------------------------------------------------------------------------------------------
void DRIVE_READY_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_READY,DRIVE_OUTPUT_GPIO_PIN_READY,GPIO_PIN_SET);
}
//----------------------------------------------------------------------------------------------------
//���������� Ready � 0
//----------------------------------------------------------------------------------------------------
void DRIVE_READY_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_READY,DRIVE_OUTPUT_GPIO_PIN_READY,GPIO_PIN_RESET);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Dir
//----------------------------------------------------------------------------------------------------
bool DRIVE_DIR_Get(void)
{
 if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_DIR,DRIVE_OUTPUT_GPIO_PIN_DIR)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Side1
//----------------------------------------------------------------------------------------------------
bool DRIVE_SIDE1_Get(void)
{
 if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_SIDE1,DRIVE_OUTPUT_GPIO_PIN_SIDE1)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Step
//----------------------------------------------------------------------------------------------------
bool DRIVE_STEP_Get(void)
{
 if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_STEP,DRIVE_OUTPUT_GPIO_PIN_STEP)==GPIO_PIN_SET) return(true);
 return(false);
}

//----------------------------------------------------------------------------------------------------
//���������� �������� ����������
//----------------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
 __disable_irq();	
 if (gpio_pin==DRIVE_OUTPUT_GPIO_PIN_STEP) 
 {
  if (DRIVE_DIR_Get()==false)
	{
   if (Track<MAX_TRACK)
	 {
		Track++;
		Update=true;
	 }
	}
	else
	{
	 if (Track>0)
	 {
		Track--;
		Update=true;
	 }
	}
  if (Track==0) DRIVE_TRACK0_Zero();
           else DRIVE_TRACK0_One(); } 
 __enable_irq();
}

//----------------------------------------------------------------------------------------------------
//���������� ���������� �������� �� DMA
//----------------------------------------------------------------------------------------------------
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
 if(hspi==&hspi2)
 {
  if(hspi2.TxXferCount==0)
  {
   //������ Index 
	 DRIVE_INDEX_Zero();
	 DRIVE_INDEX_One();
	 //��������� ����� ������ � SPI		
	 HAL_SPI_Transmit_DMA(&hspi2,TrackBuffer,TRACK_SIZE);			 
  }
 }
}

//----------------------------------------------------------------------------------------------------
//���������� ���������� ������
//----------------------------------------------------------------------------------------------------
void _Error_Handler(char *file,int line)
{
 while(1);
}
//------------------------------------------------------------------------------------------
//���������������� ������� ��������� �������
//------------------------------------------------------------------------------------------
DWORD get_fattime(void)
{
 uint32_t wYear=2020;
 uint32_t wMonth=5;
 uint32_t wDay=7;
 uint32_t wHour=12;
 uint32_t wMinute=48;
 uint32_t wSecond=00;
	
 DWORD ret=((DWORD)(wYear-1980)<<25);	
 ret|=((DWORD)wMonth<<21);
 ret|=((DWORD)wDay<<16);
 ret|=(WORD)(wHour<<11);
 ret|=(WORD)(wMinute<<5);
 ret|=(WORD)(wSecond>>1);
 return(ret);
}

//------------------------------------------------------------------------------------------
//����� � ������������� SD-�����
//------------------------------------------------------------------------------------------
void FindSD(void)
{ 
 cDisplayStandardLibrary.Print("������������� SD-�����",IDisplay::COLOR_BLACK);
 SD_ANSWER sd_answer=SD_Init();
 if (sd_answer==SD_ANSWER_OK) cDisplayStandardLibrary.Print("SD:������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_ERROR) cDisplayStandardLibrary.Print("SD:�� ������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SPI_ERROR) cDisplayStandardLibrary.Print("SD:��������� spi �� �������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SPI_NOT_SUPPORTED)cDisplayStandardLibrary.Print("SD:spi �� ��������������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_NO_RESPONSE) cDisplayStandardLibrary.Print("SD:��� ������ �� �����",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SIZE_ERROR) cDisplayStandardLibrary.Print("SD:������ ��������� �������",IDisplay::COLOR_BLACK);
 if (sd_answer!=SD_ANSWER_OK) 
 {
  while(1);	 
 }
	
 FRESULT res; 
 res=f_mount(&FatFS,"",1);
 if (res==FR_INVALID_DRIVE) cDisplayStandardLibrary.Print("FR_INVALID_DRIVE",IDisplay::COLOR_BLACK);
 if (res==FR_DISK_ERR) cDisplayStandardLibrary.Print("FR_DISK_ERR",IDisplay::COLOR_BLACK);
 if (res==FR_NOT_READY) cDisplayStandardLibrary.Print("FR_NOT_READY",IDisplay::COLOR_BLACK);
 if (res==FR_NO_FILESYSTEM) cDisplayStandardLibrary.Print("FR_NO_FILESYSTEM",IDisplay::COLOR_BLACK);
 if (res!=FR_OK) 
 {
  while(1);	 
 }
 cDisplayStandardLibrary.Print("�������� ������� �������",IDisplay::COLOR_BLACK);	
}


