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

#define DRIVE_OUTPUT_GPIO_PIN_CHDISK		GPIO_PIN_14
#define DRIVE_OUTPUT_GPIO_CHDISK				GPIOC

#define DRIVE_OUTPUT_GPIO_PIN_TRACK0		GPIO_PIN_13
#define DRIVE_OUTPUT_GPIO_TRACK0				GPIOC

#define DRIVE_OUTPUT_GPIO_PIN_DIR				GPIO_PIN_3
#define DRIVE_OUTPUT_GPIO_DIR						GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_READY			GPIO_PIN_2
#define DRIVE_OUTPUT_GPIO_READY					GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_STEP			GPIO_PIN_0
#define DRIVE_OUTPUT_GPIO_STEP					GPIOA

#define DRIVE_OUTPUT_GPIO_PIN_SIDE1			GPIO_PIN_1
#define DRIVE_OUTPUT_GPIO_SIDE1					GPIOA

#define BUTTON_OUTPUT_GPIO_PIN_UP				GPIO_PIN_10
#define BUTTON_OUTPUT_GPIO_UP						GPIOB

#define BUTTON_OUTPUT_GPIO_PIN_DOWN			GPIO_PIN_4
#define BUTTON_OUTPUT_GPIO_DOWN					GPIOA

#define BUTTON_OUTPUT_GPIO_PIN_SELECT		GPIO_PIN_11
#define BUTTON_OUTPUT_GPIO_SELECT				GPIOB

#define BUTTON_OUTPUT_GPIO_PIN_CENTER		GPIO_PIN_1
#define BUTTON_OUTPUT_GPIO_CENTER				GPIOB

//****************************************************************************************************
//���������
//****************************************************************************************************

static const size_t MAX_TRACK=79;//������������ ����� �������
static const size_t TRACK_SIZE=12800;//������ �������
static const size_t MAX_LEVEL=10;//������������ ������� ����������� ����������
static const size_t MAX_PATH=255;//������������ ����

//****************************************************************************************************
//���������� ����������
//****************************************************************************************************
SPI_HandleTypeDef hspi2;//SPI
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

CDisplayNokia5110 cDisplay;//�������
IDisplay *iDisplay_Ptr=&cDisplay;//��������� �� �������
CDisplayStandardLibrary cDisplayStandardLibrary(iDisplay_Ptr,true);//����������� ���������� �������

volatile uint8_t Track=0;//����� �����

static uint8_t TrackBuffer[TRACK_SIZE];//����� �������
static FATFS FatFS;//�������� �������
static char Path[MAX_PATH];//���� � ����� ��� � ���������

//****************************************************************************************************
//��������� �������
//****************************************************************************************************

static void RCC_Init(void);//������������� RCC
static void SPI2_Init(void);//������������� SPI2
static void DMA_Init(void);//������������� DMA
static void GPIO_Init(void);//������������� ������

static bool OutputFile(const char *filename,const char *short_name);//����� �����
static void FindSD(void);//����� � ������������� SD-�����
static void SelectFile(void);//����� �����
void PathMenu(const char *path);//����� ���� ������ ����� � �����

static void DRIVE_CHDISK_One(void);//���������� Index � 1
static void DRIVE_CHDISK_Zero(void);//���������� Index � 0

static void DRIVE_TRACK0_One(void);//���������� Track0 � 1
static void DRIVE_TRACK0_Zero(void);//���������� Track0 � 0

static void DRIVE_READY_One(void);//���������� Ready � 1
static void DRIVE_READY_Zero(void);//���������� Ready � 0

static bool DRIVE_DIR_Get(void);//�������� �������� Dir
static bool DRIVE_SIDE1_Get(void);//�������� �������� Side1
static bool DRIVE_STEP_Get(void);//�������� �������� Step

static bool BUTTON_GetButtonUpState(void);//�������� ������� ������� �� ������ �����
static bool BUTTON_GetButtonDownState(void);//�������� ������� ������� �� ������ ����
static bool BUTTON_GetButtonSelectState(void);//�������� ������� ������� �� ������ �����
static bool BUTTON_GetButtonCenterState(void);//�������� ������� ������� �� ������ �����

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

 SelectFile(); 	
}
//----------------------------------------------------------------------------------------------------
//������������� ��������� ����������
//----------------------------------------------------------------------------------------------------
static void RCC_Init(void)
{
 RCC_OscInitTypeDef RCC_OscInitStruct;
 RCC_ClkInitTypeDef RCC_ClkInitStruct;

 RCC_OscInitStruct.OscillatorType=RCC_OSCILLATORTYPE_HSE;
 RCC_OscInitStruct.HSEState=RCC_HSE_ON;
 RCC_OscInitStruct.HSEPredivValue=RCC_HSE_PREDIV_DIV1;
 RCC_OscInitStruct.HSIState=RCC_HSI_ON;
 RCC_OscInitStruct.PLL.PLLState=RCC_PLL_ON;
 RCC_OscInitStruct.PLL.PLLSource=RCC_PLLSOURCE_HSE;
 RCC_OscInitStruct.PLL.PLLMUL=RCC_PLL_MUL9;
 if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!=HAL_OK)
 {
  _Error_Handler(__FILE__,__LINE__);
 }

 RCC_ClkInitStruct.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
 RCC_ClkInitStruct.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK;
 RCC_ClkInitStruct.AHBCLKDivider=RCC_SYSCLK_DIV1;
 RCC_ClkInitStruct.APB1CLKDivider=RCC_HCLK_DIV2;
 RCC_ClkInitStruct.APB2CLKDivider=RCC_HCLK_DIV1;

 if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,FLASH_LATENCY_2)!=HAL_OK)
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
 hspi2.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_64;
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
	
 GPIO_InitStruct.Pin=DRIVE_OUTPUT_GPIO_PIN_CHDISK;
 HAL_GPIO_Init(DRIVE_OUTPUT_GPIO_CHDISK,&GPIO_InitStruct);
	
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
 
 GPIO_InitStruct.Pin=BUTTON_OUTPUT_GPIO_PIN_UP;
 HAL_GPIO_Init(BUTTON_OUTPUT_GPIO_UP,&GPIO_InitStruct);
 
 GPIO_InitStruct.Pin=BUTTON_OUTPUT_GPIO_PIN_DOWN;
 HAL_GPIO_Init(BUTTON_OUTPUT_GPIO_DOWN,&GPIO_InitStruct);
 
 GPIO_InitStruct.Pin=BUTTON_OUTPUT_GPIO_PIN_CENTER;
 HAL_GPIO_Init(BUTTON_OUTPUT_GPIO_CENTER,&GPIO_InitStruct);
 
 GPIO_InitStruct.Pin=BUTTON_OUTPUT_GPIO_PIN_SELECT;
 HAL_GPIO_Init(BUTTON_OUTPUT_GPIO_SELECT,&GPIO_InitStruct);
 
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
static void DRIVE_CHDISK_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_CHDISK,DRIVE_OUTPUT_GPIO_PIN_CHDISK,GPIO_PIN_SET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Index � 0
//----------------------------------------------------------------------------------------------------
static void DRIVE_CHDISK_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_CHDISK,DRIVE_OUTPUT_GPIO_PIN_CHDISK,GPIO_PIN_RESET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Track0 � 1
//----------------------------------------------------------------------------------------------------
static void DRIVE_TRACK0_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_TRACK0,DRIVE_OUTPUT_GPIO_PIN_TRACK0,GPIO_PIN_SET);	
}
//----------------------------------------------------------------------------------------------------
//���������� Track0 � 0
//----------------------------------------------------------------------------------------------------
static void DRIVE_TRACK0_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_TRACK0,DRIVE_OUTPUT_GPIO_PIN_TRACK0,GPIO_PIN_RESET);
}
//----------------------------------------------------------------------------------------------------
//���������� Ready � 1
//----------------------------------------------------------------------------------------------------
static void DRIVE_READY_One(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_READY,DRIVE_OUTPUT_GPIO_PIN_READY,GPIO_PIN_SET);
}
//----------------------------------------------------------------------------------------------------
//���������� Ready � 0
//----------------------------------------------------------------------------------------------------
static void DRIVE_READY_Zero(void)
{
 HAL_GPIO_WritePin(DRIVE_OUTPUT_GPIO_READY,DRIVE_OUTPUT_GPIO_PIN_READY,GPIO_PIN_RESET);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Dir
//----------------------------------------------------------------------------------------------------
static bool DRIVE_DIR_Get(void)
{
 if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_DIR,DRIVE_OUTPUT_GPIO_PIN_DIR)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Side1
//----------------------------------------------------------------------------------------------------
static bool DRIVE_SIDE1_Get(void)
{
 //if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_SIDE1,DRIVE_OUTPUT_GPIO_PIN_SIDE1)==GPIO_PIN_SET) return(true);
 if (DRIVE_OUTPUT_GPIO_SIDE1->IDR&DRIVE_OUTPUT_GPIO_PIN_SIDE1) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� �������� Step
//----------------------------------------------------------------------------------------------------
static bool DRIVE_STEP_Get(void)
{
 if (HAL_GPIO_ReadPin(DRIVE_OUTPUT_GPIO_STEP,DRIVE_OUTPUT_GPIO_PIN_STEP)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� ������� ������� �� ������ �����
//----------------------------------------------------------------------------------------------------
static bool BUTTON_GetButtonUpState(void)
{
 if (HAL_GPIO_ReadPin(BUTTON_OUTPUT_GPIO_UP,BUTTON_OUTPUT_GPIO_PIN_UP)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� ������� ������� �� ������ ����
//----------------------------------------------------------------------------------------------------
static bool BUTTON_GetButtonDownState(void)
{
 if (HAL_GPIO_ReadPin(BUTTON_OUTPUT_GPIO_DOWN,BUTTON_OUTPUT_GPIO_PIN_DOWN)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� ������� ������� �� ������ �����
//----------------------------------------------------------------------------------------------------
static bool BUTTON_GetButtonSelectState(void)
{
 if (HAL_GPIO_ReadPin(BUTTON_OUTPUT_GPIO_SELECT,BUTTON_OUTPUT_GPIO_PIN_SELECT)==GPIO_PIN_SET) return(true);
 return(false);
}
//----------------------------------------------------------------------------------------------------
//�������� ������� ������� �� ������ �����
//----------------------------------------------------------------------------------------------------
static bool BUTTON_GetButtonCenterState(void)
{
 //if (HAL_GPIO_ReadPin(BUTTON_OUTPUT_GPIO_CENTER,BUTTON_OUTPUT_GPIO_PIN_CENTER)==GPIO_PIN_SET) return(true);
 if (BUTTON_OUTPUT_GPIO_CENTER->IDR&BUTTON_OUTPUT_GPIO_PIN_CENTER) return(true);
 return(false);
}




//----------------------------------------------------------------------------------------------------
//���������� �������� ����������
//----------------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t gpio_pin)
{
 if (gpio_pin==DRIVE_OUTPUT_GPIO_PIN_STEP) 
 {
  if (DRIVE_DIR_Get()==false)
	{
   if (Track<MAX_TRACK)
	 {
		Track++;
	 }
	}
	else
	{
	 if (Track>0)
	 {
		Track--;
	 }
	}
  if (Track==0) DRIVE_TRACK0_Zero();
           else DRIVE_TRACK0_One(); 
 } 
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
//����� �����
//------------------------------------------------------------------------------------------
static bool OutputFile(const char *filename,const char *short_name)
{
 cDisplayStandardLibrary.Clear(IDisplay::COLOR_WHITE);	
 FIL file;
 if (f_open(&file,filename,FA_READ)!=FR_OK)
 {
  cDisplayStandardLibrary.Print("������",IDisplay::COLOR_BLACK);
  HAL_Delay(1000);
	return(false);
 }
 
 cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*0,"�������",IDisplay::COLOR_BLACK); 
 cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*1,"����",IDisplay::COLOR_BLACK);
 cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*2,short_name,IDisplay::COLOR_BLACK);

 __disable_irq();
 Track=0;//����� �����
 __enable_irq();

 DRIVE_CHDISK_Zero();
 DRIVE_READY_One(); 
 
 HAL_Delay(1000);
 
 DRIVE_CHDISK_One();
 DRIVE_READY_Zero(); 
 DRIVE_TRACK0_Zero();
 
 static const size_t BLOCK_SIZE=512;
 static const size_t BLOCK_AMOUNT=TRACK_SIZE/BLOCK_SIZE;
  
 struct SBlock
 {
  uint8_t Track;
  uint8_t Side;	 
 };
 
 static SBlock sBlock[BLOCK_AMOUNT];
 for(size_t n=0;n<BLOCK_AMOUNT;n++)
 {
	sBlock[n].Track=0xff;
  sBlock[n].Side=3;
 }
 uint8_t block=0;
 //��������� ����� ������ � SPI
 memset(TrackBuffer,0,TRACK_SIZE);
 HAL_SPI_Transmit_DMA(&hspi2,TrackBuffer,TRACK_SIZE);	
 HAL_SPI_DMAPause(&hspi2);
 size_t last_offset=0;
 while(1)
 {
	if (BUTTON_GetButtonCenterState()==true) break;	 
	__disable_irq();
  uint8_t track=Track;
  __enable_irq();
  uint8_t side;	 
  if (DRIVE_SIDE1_Get()==true) side=0;
                          else side=1;
	if (sBlock[block].Side!=side || sBlock[block].Track!=track)//��������� ���������� ������
	{	 	
   HAL_SPI_DMAPause(&hspi2);
		
	 sBlock[block].Side=side;
   sBlock[block].Track=track;
		
   size_t track_offset=block*512;
	 uint32_t offset=track;
	 offset*=2;
	 offset+=side;
	 offset*=TRACK_SIZE;
	 offset+=block*512;
	 if (last_offset!=offset) f_lseek(&file,offset);
	 last_offset=offset+512;
	 UINT readen;
   if (f_read(&file,TrackBuffer+track_offset,sizeof(uint8_t)*512,&readen)!=FR_OK)
	 {
    HAL_SPI_DMAStop(&hspi2);		 
    f_close(&file);		 
    cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*5,"����",IDisplay::COLOR_BLACK);
		HAL_Delay(5000);
		return(false);
	 }
	 if (readen!=512)
	 {
    HAL_SPI_DMAStop(&hspi2);		 
    f_close(&file);
    cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*5,"����",IDisplay::COLOR_BLACK);
    HAL_Delay(5000);
		return(false);
	 }	
	 size_t counter=0;
   for(size_t n=0;n<BLOCK_AMOUNT;n++)
   {
    if ((sBlock[n].Side!=side) || (sBlock[n].Track!=track)) counter++;
   }
	 if (counter!=0) HAL_SPI_DMAPause(&hspi2);
	            else HAL_SPI_DMAResume(&hspi2);
	 
	}
	block++;
	block%=BLOCK_AMOUNT;
 }
 f_close(&file);
 HAL_SPI_DMAStop(&hspi2);
 return(true); 
}


//------------------------------------------------------------------------------------------
//����� � ������������� SD-�����
//------------------------------------------------------------------------------------------
static void FindSD(void)
{
 cDisplayStandardLibrary.Print("����� SD",IDisplay::COLOR_BLACK);
 SD_ANSWER sd_answer=SD_Init();	
 if (sd_answer==SD_ANSWER_OK) cDisplayStandardLibrary.Print("SD ������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_ERROR) cDisplayStandardLibrary.Print("������ SD",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SPI_ERROR) cDisplayStandardLibrary.Print("������ SPI",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SPI_NOT_SUPPORTED)cDisplayStandardLibrary.Print("SPI	���",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_NO_RESPONSE) cDisplayStandardLibrary.Print("��� ������",IDisplay::COLOR_BLACK);
 if (sd_answer==SD_ANSWER_SIZE_ERROR) cDisplayStandardLibrary.Print("������ ?",IDisplay::COLOR_BLACK);
 if (sd_answer!=SD_ANSWER_OK) 
 {
  while(1);	 
 }
}
//------------------------------------------------------------------------------------------
//����� �����
//------------------------------------------------------------------------------------------
static void SelectFile(void)
{
 cDisplayStandardLibrary.Print("����� ��",IDisplay::COLOR_BLACK);
 FRESULT res; 
 res=f_mount(&FatFS,"",1);
 if (res==FR_INVALID_DRIVE) cDisplayStandardLibrary.Print("��� SD",IDisplay::COLOR_BLACK);
 if (res==FR_DISK_ERR) cDisplayStandardLibrary.Print("������ SD",IDisplay::COLOR_BLACK);
 if (res==FR_NOT_READY) cDisplayStandardLibrary.Print("�� ������",IDisplay::COLOR_BLACK);
 if (res==FR_NO_FILESYSTEM) cDisplayStandardLibrary.Print("��� ��",IDisplay::COLOR_BLACK);
 if (res!=FR_OK)
 {
  while(1);	 
 }
 cDisplayStandardLibrary.Print("�� �������",IDisplay::COLOR_BLACK);	 
 HAL_Delay(1000); 
 
 FIL file;
 if (f_open(&file,"Test.MFM",FA_READ)==FR_OK)
 {
  cDisplayStandardLibrary.Clear(IDisplay::COLOR_WHITE);
  UINT readen;
  for(size_t n=0;n<5;n++)
  {
   size_t begin=HAL_GetTick(); 
   f_lseek(&file,n*12800*30);
   f_read(&file,TrackBuffer,sizeof(uint8_t)*TRACK_SIZE,&readen);
   size_t time=HAL_GetTick()-begin;
   char str[25];
   sprintf(str,"%i ��",time);
   cDisplayStandardLibrary.Print(str,IDisplay::COLOR_BLACK);
  }
  f_close(&file); 
	while(1);
 }
 
 PathMenu("/");
}
//------------------------------------------------------------------------------------------
//����� ���� ������ �����
//------------------------------------------------------------------------------------------
void PathMenu(const char *path)
{
 size_t level=0;//������� �����������	
 //��������� � ������� ����� ����� �� �����
 FILINFO fileInfo;
 FRESULT res;
 static DIR dir[MAX_LEVEL];//�������� ���������� �� ������
 static size_t index[MAX_LEVEL];//����� ����� �� ������
 static size_t length_path_name[MAX_LEVEL];//������ ����� �� �������
 index[level]=0;//����� ���������� �����	
 res=f_opendir(&dir[level],Path);
 if (res!=FR_OK) return;//��� ������
 sprintf(Path,"%s",path);
 length_path_name[level]=strlen(Path); 	
 while(1)
 {
  DIR current_dir=dir[level];	 
  //������� ������ � SD-����� (������� ������� ������ ����� ������ �� index/5)
  uint8_t offset=(uint8_t)((index[level]%5UL));
  //���������� �� �������� �������� (index-offset)
  for(uint8_t n=0;n<index[level]-offset;n++)
	{
   res=f_readdir(&current_dir,&fileInfo);
   if (res!=FR_OK) break;
	 if (fileInfo.fname[0]==0) break;
	}
  //�������, ������� � �������� ��������
	cDisplayStandardLibrary.Clear(IDisplay::COLOR_WHITE);
  uint8_t counter=0;
  for(uint8_t n=0;n<5;n++,counter++)
  {
   res=f_readdir(&current_dir,&fileInfo);
   if (res!=FR_OK) break;
	 if (fileInfo.fname[0]==0) break;
   //������ ��� ����� 
   if (n==offset)
	 {
    for(size_t m=0;m<11;m++) cDisplayStandardLibrary.ClearSymbol(CDisplayStandardLibrary::FONT_WIDTH*m,CDisplayStandardLibrary::FONT_HEIGHT*n,IDisplay::COLOR_BLACK);
		cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*n,fileInfo.fname,IDisplay::COLOR_WHITE);//��������� ����
	 }
	 else cDisplayStandardLibrary.PutString(0,CDisplayStandardLibrary::FONT_HEIGHT*n,fileInfo.fname,IDisplay::COLOR_BLACK);
  }
  HAL_Delay(100);
  //��� ������� ������
  while(1)
  {
 	 if (BUTTON_GetButtonUpState()==true)	
 	 {
    if (index[level]>0)
	  {
	   index[level]--;
		 break;
	  }
	 }	 
   if (BUTTON_GetButtonDownState()==true)	
   {
    //������� ������� � ���������� �����		 
    current_dir=dir[level];		 
		size_t n;
    for(n=0;n<=index[level]+1;n++)
	  {
     res=f_readdir(&current_dir,&fileInfo);
     if (res!=FR_OK) break;
	   if (fileInfo.fname[0]==0) break;
		}		
		if (n==index[level]+2) index[level]++;
	  break;
	 }
   if (BUTTON_GetButtonSelectState()==true)	
   {		 
    //���������� �� ���������� ��������
    current_dir=dir[level];
		size_t n;
    for(n=0;n<=index[level];n++)
	  {
     res=f_readdir(&current_dir,&fileInfo);
     if (res!=FR_OK) break;
	   if (fileInfo.fname[0]==0) break;
		}		
		if (n==index[level]+1)
		{
		 if ((fileInfo.fattrib&AM_DIR)==AM_DIR)
		 {
			if (level+1<MAX_LEVEL)
			{
			 if (length_path_name[level]+strlen(fileInfo.fname)+2<MAX_PATH)
			 {
			  if (level==0) sprintf(Path+length_path_name[level],"%s",fileInfo.fname);
                 else sprintf(Path+length_path_name[level],"/%s",fileInfo.fname);
 			  level++;
			  length_path_name[level]=strlen(Path);
			  res=f_opendir(&dir[level],Path);
			  index[level]=0;
				HAL_Delay(300);
			 }
			}
		 }
		 else
		 {
			if (length_path_name[level]+strlen(fileInfo.fname)+2<MAX_PATH)
			{				
 			 if (level==0) sprintf(Path+length_path_name[level],"%s",fileInfo.fname);
                else sprintf(Path+length_path_name[level],"/%s",fileInfo.fname);
			 HAL_Delay(100); 
		   OutputFile(Path,fileInfo.fname);
			 HAL_Delay(500);
			 Path[length_path_name[level]]=0;//���������� ������� ������� ����������			 
			}
		 }
	  }
		break;
	 }
   if (BUTTON_GetButtonCenterState()==true)	
	 {
		if (level>0)
		{
		 level--;	
     Path[length_path_name[level]]=0;//���������� ������� ������� ����������			 			
     HAL_Delay(300);
		 break;
		}	 
	 }
	 
  }
 }
 //f_closedir(&dir);
}