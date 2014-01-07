/**
  ******************************************************************************
  * @file    main.c
  * @author  Microcontroller Division
  * @version V1.0.3
  * @date    May-2013
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */
 
/* Includes ------------------------------------------------------------------*/

#include "main.h"
#include "LUT.h"


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/

#define DEBUG_SWD_PIN  				1  /* needs to be set to 1 to enable SWD debug pins, set to 0 for power consumption measurement*/
#define HEATER_GPIO_PORT 			GPIOA
#define FRIGE_GPIO_PORT     	GPIOC
#define HEATER1_GPIO_PIN 			GPIO_Pin_11
#define HEATER2_GPIO_PIN    	GPIO_Pin_12
#define FRIGE_GPIO_PIN      	GPIO_Pin_12
#define HEATER_GPIO_PORT_CLK  RCC_AHBPeriph_GPIOA
#define FRIGE_GPIO_PORT_CLK   RCC_AHBPeriph_GPIOC
#define VREFINT_CAL						1224
#define MAX_TEMP_CHNL					16
#define ADC_CONV_BUFF_SIZE		20
#define HEATER_PERIOD					50
#define FRIGE_PERIOD					100
#define Kp										-150
#define	Ki										-1
#define Kd									  200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

DMA_InitTypeDef DMA_InitStructure;
uint16_t ADC_ConvertedValueBuff[ADC_CONV_BUFF_SIZE];
uint16_t currentTmp;
uint16_t targetTmp=200;
uint16_t roomTmp=217;
uint8_t  heaterLevel;
uint8_t  frigeLevel;
char strDisp[20];
volatile bool flag_UserButton;
volatile bool flag_LED=0;
volatile bool flag_Frige=0;
volatile bool flag_Heater=0;
volatile bool flag_Mode=0;
RCC_ClocksTypeDef RCC_Clocks;

static volatile uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
void  		RCC_Configuration(void);
void  		Init_GPIOs (void);
void  		Init_Timer2 (void);
void			Init_Timer3 (void);
void 			Init_ADC (void);
void			Init_DMA (void);
void  		clearUserButtonFlag(void);
void 			insertionSort(uint16_t *numbers, uint32_t array_size);
uint32_t	interquartileMean(uint16_t *array, uint32_t numOfSamples);
uint16_t  getTmp(void);
	

/*******************************************************************************/

/**
  * @brief main entry point.
  * @par Parameters None
  * @retval void None
  * @par Required preconditions: None
  */
                                  
int main(void)
{
	uint16_t Message[7]; 
  /* Configure Clocks for Application need */
  RCC_Configuration();
  /* Set internal voltage regulator to 1.8V */
  PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);
  /* Wait Until the Voltage Regulator is ready */
  while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;
  /* Enable debug features in low power modes (Sleep, STOP and STANDBY) */
#ifdef  DEBUG_SWD_PIN
  DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);
#endif 
  /* Configure SysTick IRQ and SysTick Timer to generate interrupts */
  RCC_GetClocksFreq(&RCC_Clocks);
	/* Set Systick Timer Interval*/
  SysTick_Config(RCC_Clocks.HCLK_Frequency / 2000);
  /* Init I/O ports */
  Init_GPIOs();  
	
  /* Initializes the LCD glass */
  LCD_GLASS_Configure_GPIO();
  LCD_GLASS_Init(); 
	/* Initialize ADC and DMA */
	Init_ADC();
	Init_DMA();
	/* Initialize Touch Sensor */
	TSL_user_Init();
	/* Initialize Timer 2 */
	Init_Timer2();
	 /* Display Welcome message */ 
  //LCD_GLASS_ScrollSentence("**  LASER TEMP CONTROLLER  **",1,SCROLL_SPEED*2);
	LCD_GLASS_Clear();
	heaterLevel=0;
	frigeLevel=100;
	targetTmp=getTmp();
	frigeLevel=100;
	Init_Timer3();
	while(1)
	{
		if(flag_UserButton)
		{
			if(flag_Mode) flag_Mode=0;
			else flag_Mode=1;
			clearUserButtonFlag();
		}
		if(flag_Mode)
		{
			Message[0]=currentTmp/100+'0';
			Message[1]=(currentTmp%100)/10+'0';
			Message[2]=currentTmp%10+'0';
			Message[1]|=DOT;
			Message[3]='°' ;
			Message[4]='C' ;
			Message[5]='M';
			LCD_GLASS_DisplayStrDeci(Message);
		}
		else
		{
			if(TSL_user_Action()==TSL_STATUS_OK)
			{
				targetTmp=setTempearture();
				Message[0]=targetTmp/100+'0';
				Message[1]=(targetTmp%100)/10+'0';
				Message[2]=targetTmp%10+'0';
				Message[1]|=DOT;
				Message[3]='°' ;
				Message[4]='C' ;
				Message[5]='S';
				LCD_GLASS_DisplayStrDeci(Message);
			}
		}
	}
}

/*---------------------------------------------------------------------------*/

void setUserButtonFlag(void)
{
  flag_UserButton = TRUE;
}

void clearUserButtonFlag(void)
{
  flag_UserButton = FALSE;
}



/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{  
  /* Enable HSI Clock */
  RCC_HSICmd(ENABLE);
  /*!< Wait till HSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  RCC_MSIRangeConfig(RCC_MSIRange_6);
  RCC_HSEConfig(RCC_HSE_OFF);  
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    while(1);
  }
  /* Enable  comparator clock LCD and PWR mngt */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE); 
  /* Enable ADC clock & SYSCFG */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_SYSCFG, ENABLE);	
	/* Allow access to the RTC */
  PWR_RTCAccessCmd(ENABLE);
  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);	
	/* LSE Enable */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);  
  RCC_RTCCLKCmd(ENABLE);   
  /* LCD Clock Source Selection */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
}



/**
  * @brief  To initialize the I/O ports
  * @caller main
  * @param None
  * @retval None
  */


void conf_analog_all_GPIOS(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable GPIOs clock */ 	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | 
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);
  /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN; 
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOH, &GPIO_InitStructure);
#if  DEBUG_SWD_PIN == 1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_Pin_13) & (~GPIO_Pin_14);
#endif 
  GPIO_Init(GPIOA, &GPIO_InitStructure);  
  /* Disable GPIOs clock */ 	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | 
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
}

void  Init_GPIOs (void)
{
  GPIO_InitTypeDef GPIO_InitStructure; 
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  conf_analog_all_GPIOS();   /* configure all GPIOs as analog input */ 
  /* Enable GPIOs clock */ 	
  RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK | HEATER_GPIO_PORT_CLK | FRIGE_GPIO_PORT_CLK, ENABLE); 
  /* USER button and WakeUP button init: GPIO set in input interrupt active mode */  
  /* Configure User Button pin as input */
  GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
  GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);
  /* Connect Button EXTI Line to Button GPIO Pin */
  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);
  /* Configure User Button and IDD_WakeUP EXTI line */
  EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
  /* Enable and set User Button and IDD_WakeUP EXTI Interrupt to the lowest priority */
  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
/* Configure the GPIO_LED pins  LD3 & LD4*/
  GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
  GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);	
  GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);	
/* Configure the Frige pin*/
	GPIO_InitStructure.GPIO_Pin=FRIGE_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(FRIGE_GPIO_PORT,&GPIO_InitStructure);
	GPIO_HIGH(FRIGE_GPIO_PORT,FRIGE_GPIO_PIN);	
/* Configure the Hearter Control GPIO */
	GPIO_InitStructure.GPIO_Pin=HEATER1_GPIO_PIN|HEATER2_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(HEATER_GPIO_PORT,&GPIO_InitStructure);
	GPIO_LOW(HEATER_GPIO_PORT,HEATER1_GPIO_PIN);
	GPIO_LOW(HEATER_GPIO_PORT,HEATER2_GPIO_PIN);
/* Disable all GPIOs clock */ 	
  /*RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | 
                        RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | 
                        RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);*/
}  

void Init_Timer2(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
  TIM_DeInit(TIM2);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* 500Hz */
  TIM_TimeBaseStructure.TIM_Period=20;
  TIM_TimeBaseStructure.TIM_Prescaler=1600;
	TIM_TimeBaseStructure.TIM_ClockDivision=1;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM2, ENABLE); 		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void Init_Timer3(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;	
  TIM_DeInit(TIM3);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
	/* 20Hz */
  TIM_TimeBaseStructure.TIM_Period=500;
  TIM_TimeBaseStructure.TIM_Prescaler=1600;
	TIM_TimeBaseStructure.TIM_ClockDivision=1;
  TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM3, ENABLE); 		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void heaterISR(void)
{
	static uint8_t timeStep=0;
	uint8_t heater1,heater2;
	if(heaterLevel<=0||heaterLevel>100) 
	{
		setHeater(1,0);
		setHeater(2,0);
		return;
	}
	if(heaterLevel<=50)
	{
		heater1=heaterLevel;
		heater2=0;
	}
	else
	{
		heater1=50;
		heater2=heaterLevel-50;
	}
	if (timeStep<heater1) setHeater(1,1);
	else setHeater(1,0);
	if (timeStep<heater2) setHeater(2,1);
	else setHeater(2,0);
	if (timeStep==HEATER_PERIOD-1) timeStep=0;
	else timeStep++;
}

void frigeISR(void)
{
	static uint8_t timeStep=0;
	if (timeStep<frigeLevel) setFrige(1);
	else setFrige(0);
	if (timeStep==FRIGE_PERIOD-1) timeStep=0;
	else timeStep++;
}

void pidISR(void)
{
	static int32_t PID=0,PID_last=0,e=0,e1=0,e2=0;
	currentTmp=getTmp();
	if(currentTmp>999) currentTmp=999;
	PID_last=PID;
	e2=e1;
	e1=e;
	e=currentTmp-targetTmp;
	/* PID=PID_last + Kp (e-e1) + Ki (e) + Kd (e- 2*e1+e2) */
	PID=PID_last + Kp*(e-e1) + Ki*(e) + Kd*(e-2*e1+e2);
	if(PID>=4000) PID=3999;
	if(PID<0) PID=0;
	heaterLevel=PID*100/4000;
}

void Init_ADC(void) 
{
	uint32_t ch_index;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	/* Eanble ADC1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	/* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  /* Wait until the ADC1 is ready */
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADONS) == RESET);
	/* Setup ADC common init struct */
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
  ADC_CommonInit(&ADC_CommonInitStructure);	
	/* ADC1 Configuration ------------------------------------------------------*/ 
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution= ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; 
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConvEdge_None;  
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
	ADC_InitStructure.ADC_NbrOfConversion = ADC_CONV_BUFF_SIZE; 
	/* Now do the setup */ 
	ADC_Init(ADC1, &ADC_InitStructure); 	
	for (ch_index = 1; ch_index <= MAX_TEMP_CHNL; ch_index++)
	{
      ADC_RegularChannelConfig(ADC1, ADC_Channel_4, ch_index, ADC_SampleTime_384Cycles);
  }
	ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 17, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 18, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 19, ADC_SampleTime_384Cycles);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_17, 20, ADC_SampleTime_384Cycles);
	
}

void Init_DMA(void)
{
  /* Declare NVIC init Structure */
  //NVIC_InitTypeDef NVIC_InitStructure;
  
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  /* De-initialise  DMA */
  DMA_DeInit(DMA1_Channel1);
  /* DMA1 channel1 configuration */
  DMA_StructInit(&DMA_InitStructure);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);	     // Set DMA channel Peripheral base address to ADC Data register
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_ConvertedValueBuff;  // Set DMA channel Memeory base addr to ADC_ConvertedValueBuff address
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                         // Set DMA channel direction to peripheral to memory
  DMA_InitStructure.DMA_BufferSize = ADC_CONV_BUFF_SIZE;                     // Set DMA channel buffersize to peripheral to ADC_CONV_BUFF_SIZE
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	     // Disable DMA channel Peripheral address auto increment
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                    // Enable Memeory increment (To be verified ....)
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;// set Peripheral data size to 8bit 
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	     // set Memeory data size to 8bit 
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                              // Set DMA in normal mode
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                     // Set DMA channel priority to High
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                               // Disable memory to memory option 
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);								 // Use Init structure to initialise channel1 (channel linked to ADC)
  /* Enable Transmit Complete Interrup for DMA channel 1 */ 
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);	
	DMA_Cmd(DMA1_Channel1, ENABLE); 
  /* Setup NVIC for DMA channel 1 interrupt request
  NVIC_InitStructure.NVIC_IRQChannel =   DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);*/
}

uint16_t getTmp(void)
{
	uint32_t sampleAVG,refAVG,volt;
	/* Re-initialize DMA */
	DMA_DeInit(DMA1_Channel1);
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);	
	/* Disable DMA mode for ADC1 */ 
  ADC_DMACmd(ADC1, DISABLE);
   /* Enable DMA mode for ADC1 */  
  ADC_DMACmd(ADC1, ENABLE);	
	ADC_SoftwareStartConv(ADC1);
	while(DMA_GetFlagStatus(DMA1_FLAG_TC1)==RESET);
	DMA_ClearFlag(DMA1_FLAG_TC1);	
	insertionSort(ADC_ConvertedValueBuff,MAX_TEMP_CHNL);
	sampleAVG=interquartileMean(ADC_ConvertedValueBuff,MAX_TEMP_CHNL);
	refAVG=0;
	refAVG+=ADC_ConvertedValueBuff[16];
	refAVG+=ADC_ConvertedValueBuff[17];
	refAVG+=ADC_ConvertedValueBuff[18];
	refAVG+=ADC_ConvertedValueBuff[19];
	refAVG/=4;
	volt=VREFINT_CAL*sampleAVG/refAVG;
	return TmpLUT[volt];
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
  
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{

  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
	
}

void setFrige(uint8_t onoff)
{
	if(onoff)
	{
		GPIO_LOW(FRIGE_GPIO_PORT,FRIGE_GPIO_PIN);
	}
	else
	{
		GPIO_HIGH(FRIGE_GPIO_PORT,FRIGE_GPIO_PIN);
	}
}

void setHeater(uint8_t num,uint8_t onoff)
{
	switch (num)
	{
		case 1:
			if(onoff)
			{
				GPIO_HIGH(HEATER_GPIO_PORT,HEATER1_GPIO_PIN);
				GPIO_HIGH(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
			}
			else
			{
				GPIO_LOW(HEATER_GPIO_PORT,HEATER1_GPIO_PIN);
				GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
			}
			break;
		case 2:
			if(onoff)
			{
				GPIO_HIGH(HEATER_GPIO_PORT,HEATER2_GPIO_PIN);
				GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
			}
			else
			{
				GPIO_LOW(HEATER_GPIO_PORT,HEATER2_GPIO_PIN);
				GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
			}
			break;
		default:break;
	}
}

void changeLED(void)
{
	if (flag_LED)
	{
		GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);	
		GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
		flag_LED=0;
	} 
	else
	{
		GPIO_HIGH(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);	
		GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
		flag_LED=1;
	}
}

void toggleFrige(void)
{
	if (flag_Frige)
	{
		GPIO_HIGH(FRIGE_GPIO_PORT,FRIGE_GPIO_PIN);
		GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
		flag_Frige=0;
	}
	else
	{
		GPIO_LOW(FRIGE_GPIO_PORT,FRIGE_GPIO_PIN);
		GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
		flag_Frige=1;
	}
}

void toggleHeater()
{
	if (flag_Heater)
	{
		GPIO_LOW(HEATER_GPIO_PORT,HEATER1_GPIO_PIN);
		GPIO_LOW(HEATER_GPIO_PORT,HEATER2_GPIO_PIN);
		GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
		flag_Heater=0;
	}
	else
	{
		GPIO_HIGH(HEATER_GPIO_PORT,HEATER1_GPIO_PIN);
		GPIO_HIGH(HEATER_GPIO_PORT,HEATER2_GPIO_PIN);
		GPIO_HIGH(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
		flag_Heater=1;
	}
}

void insertionSort(uint16_t *numbers, uint32_t array_size) 
{
  
	uint32_t i, j;
	uint32_t index;

  for (i=1; i < array_size; i++) {
    index = numbers[i];
    j = i;
    while ((j > 0) && (numbers[j-1] > index)) {
      numbers[j] = numbers[j-1];
      j = j - 1;
    }
    numbers[j] = index;
  }
}

uint32_t interquartileMean(uint16_t *array, uint32_t numOfSamples)
{
    uint32_t sum=0;
    uint32_t  index, maxindex;
    /* discard  the lowest and the highest data samples */ 
	maxindex = 3 * numOfSamples / 4;
    for (index = (numOfSamples / 4); index < maxindex; index++){
            sum += array[index];
    }
	/* return the mean value of the remaining samples value*/
    return ( sum / (numOfSamples / 2) );
}

/**
  * @brief  Executed when a sensor is in Error state
  * @param  None
  * @retval None
  */
void MyLinRots_ErrorStateProcess(void)
{
  // Add here your own processing when a sensor is in Error state
  TSL_linrot_SetStateOff();
}


/**
  * @brief  Executed when a sensor is in Off state
  * @param  None
  * @retval None
  */
void MyLinRots_OffStateProcess(void)
{
  // Add here your own processing when a sensor is in Off state
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
