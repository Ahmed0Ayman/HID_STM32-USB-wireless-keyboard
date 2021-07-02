/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include "retarget.h"
#include "usbd_customhid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


uint8_t		USBbuffer[8];	/* the sending buffer to host */
uint32_t 	JOYRes[2];     /* hold the ADC measured value */
uint8_t 	CapsLock =0X82 ;  /* Hold The Caps lock state to generate Capital  letter */


extern USBD_HandleTypeDef hUsbDeviceFS;


/*variable to  hold all modifiers */
volatile union {
	struct {
	uint8_t LeftCTRL    	: 1 ;
	uint8_t LeftSHIFT    	: 1 ;
	uint8_t LeftALT   		: 1 ;
	uint8_t LeftGUI    		: 1 ;
	uint8_t RightCTRL     	: 1 ;
	uint8_t RightSHIFT    	: 1 ;
	uint8_t RightALT   		: 1 ;
	uint8_t RightGUI     	: 1 ;
	}Buttons;
	uint8_t AllModifiers  ;
}MoifiersButtons;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define HC05TaskDataReady_Flag_BIT			0x01u
#define KeyPadTaskDataReady_Flag_BIT		0x02u
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* hint it's useful to make all global var in your app as volatile actually not all var only that may be change in interrupt
 * but to make my code reliable i made it for all on the other hand  this reduce the performance a little bit but
 * a void hidden bugs that generate when your compiler optimize your code you can search for volatile keyword
 * usage for more information
 */
volatile uint8_t CharKey =0 ,KeyPadEquivelant[2] ={0},KeyPadAscii[2] ;

/* structure to group all KeyPad related variables */
volatile struct KeyPad_t {
uint8_t Key  ;
bool  status ;
char KeyChar[16];
}KeyPad = {.Key = 0, .status = false , .KeyChar = { /* initialize KeyPad with meaning Key ascii */
		'1','2','3','A',
		'4','5','6','B',
		'7','8','9','C',
		'*','0','#','D'
}
};


volatile uint8_t AsciiChar[3] ; /* used to receive char from uart connected with HC-05 minimun message receivced is 3 char (char + '\r + '\n') */
volatile uint8_t KeyBoardChar[100]; /* used to hold the converted characters */
volatile uint8_t ReceivedString[100];/* hold the received message and terminate this with null char */



  /* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

osSemaphoreId_t	HC05SemaHandle;
const osSemaphoreAttr_t HC05Sema_attributes = {
		/* set name for debugging information */
  .name = "HC05Sema_attributes"
};

/* for joy stick this will implemented next version */
osSemaphoreId_t	KeyPadSemaHandle;
const osSemaphoreAttr_t KeyPadSema_attributes = {
		/* set name for debugging information */
  .name = "KeyPadSemaphore"
};


osThreadId_t MainTaskHandle;
const osThreadAttr_t MainTasK_Attr ={
		.name = "MainTask" ,
		.stack_size =256 * 4 ,
		.priority =  (osPriority_t) osPriorityHigh2
};


osThreadId_t HC05TaskHandle;
const osThreadAttr_t HC05TasK_Attr  ={
		.name = "Hc05Task" ,
		.stack_size =256 * 4 ,
		.priority =  (osPriority_t) osPriorityHigh1
};



osThreadId_t KeyPadTaskHandle;
const osThreadAttr_t KeyPadTasK_Attr  ={
		.name = "KeyPadTask" ,
		.stack_size =256 * 4 ,
		.priority =  (osPriority_t) osPriorityHigh
};





void MainTask_Fun(void * arg);
void HC05Task_Fun(void * arg);
void KeyPadTask_Fun(void * arg);


/* this function used to convert char from Ascii code to USB usage code
 * also not all ascii is available by this function so for any unavailable
 * char will will avoid it right now  */
void GetKeyBoardEquivelant( uint8_t * AsciiChar ,uint8_t * KeyBoardChar);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* this function used to get keypressed char  */
void Get_Key(void)
{

	KeyPad.status = false ;
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 1 );
	osDelay(1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, 0 );
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 1 );
	osDelay(1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, 0 );
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 1 );
	osDelay(1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, 0 );
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1 );
	osDelay(1);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0 );
	if(KeyPad.status == false )
	{
		KeyPad.Key = 0 ;
		CharKey = 0 ;
	}

	/* check first if there is any valid pressed Key */
	  if(KeyPad.Key != 0)
	  CharKey = KeyPad.KeyChar[KeyPad.Key-1];

}


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
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	MainTaskHandle = osThreadNew(MainTask_Fun, NULL, &MainTasK_Attr);

	HC05TaskHandle = osThreadNew(HC05Task_Fun, NULL, &HC05TasK_Attr);

	KeyPadTaskHandle = osThreadNew(KeyPadTask_Fun, NULL, &KeyPadTasK_Attr);

	HC05SemaHandle = osSemaphoreNew(1, 0, &HC05Sema_attributes);

	KeyPadTaskHandle = osSemaphoreNew(1, 0, &KeyPadSema_attributes);





  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

 /* register uart1 to used by system calls printf and scanf */
  RetargetInit(&huart1);

  /* start timer 8 to work as trigger for ADC conversion */

  /* start TIM8 to generate periodic trigger event to capture anew ADC value */
  HAL_TIM_Base_Start(&htim8);

  /* start ADC and initialize it to work with DMA mode */

  /*initialize ADC to work with DMA when trigger event is received*/
  HAL_ADC_Start_DMA(&hadc1,&JOYRes, 2);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* KEYPAD key is work through interrupt mode so here we only detect the Key pressed number
 */
void  HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	KeyPad.status = false  ;
	if(GPIO_Pin == GPIO_PIN_11)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))
			KeyPad.Key = 1	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
			KeyPad.Key = 2	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))
			KeyPad.Key = 3	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
			KeyPad.Key = 4	;

	}
	else if(GPIO_Pin == GPIO_PIN_12)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))

			KeyPad.Key = 5	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
			KeyPad.Key = 6	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))
			KeyPad.Key = 7	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
			KeyPad.Key = 8	;

	}
	else if(GPIO_Pin == GPIO_PIN_13)
	{
		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))
			KeyPad.Key = 9	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
			KeyPad.Key = 10	;

		if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))
			KeyPad.Key = 11	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
			KeyPad.Key = 12	;

	}
	else if(GPIO_Pin == GPIO_PIN_14)
	{
		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10))
			KeyPad.Key = 13	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11))
			KeyPad.Key = 14	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12))
			KeyPad.Key = 15	;

		if( HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_13))
			KeyPad.Key = 16	;

	}

	/* to inform the GETKEY function we have anew event so go to calculate the KEY equivalent value */
	KeyPad.status = true ;
}

/* main task that receive the char from bluetooth or from Keypad and send them to USB host*/
void MainTask_Fun(void * arg)
{
	uint8_t index =0 ;
	uint32_t flag =0 ; /* local var to  iterate over the string array */

	while(1)
	{
			//osSemaphoreAcquire(JOYSemaHandle, osWaitForever);
			osThreadFlagsWait(HC05TaskDataReady_Flag_BIT |KeyPadTaskDataReady_Flag_BIT , osFlagsWaitAny|osFlagsNoClear, osWaitForever);

			flag = osThreadFlagsGet();
			index =0 ;
			if(flag  ==  HC05TaskDataReady_Flag_BIT )
			{
				while(KeyBoardChar[index] != '\0') /* loop until send all char */
				{
				USBbuffer[0] = 	1;// MouseButtons.AllBits ;
				USBbuffer[1] = 	0 ; // modifiers
				USBbuffer[2] =  0 ;  // reserved
				USBbuffer[4] = 	KeyBoardChar[index++] ; //first key
				USBbuffer[3] = 	CapsLock ; //second key

				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USBbuffer, 8);
				  osDelay(30); /* made so delay for next host request */

				  /* now send no event to prevent repetation */
				  USBbuffer[0] = 	1;// MouseButtons.AllBits ;
				  USBbuffer[1] = 	0 ; // modifiers
				  USBbuffer[2] =  	0 ;  // reserved
				  USBbuffer[3] = 	0; //first key
				  USBbuffer[4] = 	0; //second key
	              USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USBbuffer, 8);

	    		  osDelay(30); /* need another  delay because the possibility of iteration */
				}

				CapsLock =0X82;
	    		osThreadFlagsClear(HC05TaskDataReady_Flag_BIT);

				/* clear buffers to make it clear from garbage values */
				memset(KeyBoardChar , 0 , sizeof(KeyBoardChar));
				memset(ReceivedString , 0, sizeof(ReceivedString));
			}
			else if(flag  ==   KeyPadTaskDataReady_Flag_BIT) /* here Keypad return a notification */
			{
				USBbuffer[0] = 	1;// MouseButtons.AllBits ;
				USBbuffer[1] = 	0 ; // modifiers
				USBbuffer[2] =  0 ;  // reserved
				USBbuffer[4] = 	KeyPadEquivelant[0] ; //first key
				USBbuffer[3] = 	CapsLock ; //second key

				  USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USBbuffer, 8);

				  osDelay(20); /* made so delay for next host request */
				CapsLock =0X82;
				  /* now send no event to prevent repetation */
				  USBbuffer[0] = 	1;// MouseButtons.AllBits ;
				  USBbuffer[1] = 	0 ; // modifiers
				  USBbuffer[2] =  	0 ;  // reserved
				  USBbuffer[3] = 	0; //first key
				  USBbuffer[4] = 	0; //second key

	              USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, USBbuffer, 8);
	    		  osThreadFlagsClear(KeyPadTaskDataReady_Flag_BIT);

	    		  memset(KeyPadEquivelant , 0 , sizeof(KeyPadEquivelant));

				}


			}

}



 void HC05Task_Fun(void * arg)
{

	 while(1)
	{
		/* first receive character from bluetooth */
		HAL_UART_Receive_IT(&huart2, AsciiChar, 1) ;

		/*  wait until semaphore is released from uart that indicate the full message is received and start your processing  on it */
		osSemaphoreAcquire(HC05SemaHandle, osWaitForever);

		/* get the equivalent vakues for the received string */
		GetKeyBoardEquivelant( ReceivedString , KeyBoardChar);

		/* send notification for main task that you have anew message ready to send to host */
		osThreadFlagsSet(MainTaskHandle, HC05TaskDataReady_Flag_BIT);

	}


}

 /* Keypad task this driver work with keypad and hc05 */
void KeyPadTask_Fun(void * arg)
{
	while(1)
	{
		/* till the processor come here and made a busy wait state */
			while(CharKey == 0)
			{
				Get_Key();
			}

			KeyPadAscii[0] = CharKey ;
			KeyPadAscii[1] = 0 ;

			KeyPadEquivelant[0] = 0 ;
			GetKeyBoardEquivelant(KeyPadAscii,KeyPadEquivelant);

			osThreadFlagsSet(MainTaskHandle, KeyPadTaskDataReady_Flag_BIT);

			/* clear for next check */
			CharKey =0;

			/* make some delay to free up the processor useful in low power mode */
			osDelay(20);

	}


}



void GetKeyBoardEquivelant( uint8_t * AsciiChar ,uint8_t * KeyBoardChar)
{

	uint8_t IndexAscii =0 , IndexKeyboard = 0   ;

	while(AsciiChar[IndexAscii] != '\0' )
	{

	/* because the difference between ascii code and usage table for keyboard we here need to make a tons of checks */
	if((AsciiChar[IndexAscii] >= 97)&&(AsciiChar[IndexAscii] <= 122)) /* small letter */
	{
		/*we need to clear  caps lock */
		KeyBoardChar[IndexKeyboard] =  AsciiChar[IndexAscii] - 93 ;
	}
	else if((AsciiChar[IndexAscii] >= 65)&&(AsciiChar[IndexAscii] <= 90))/* capital letter */
	{
		/*we need to set capslock */

		CapsLock = 0x39 ;/* send caps lock to type capital letter  an screen */
		KeyBoardChar[IndexKeyboard] = AsciiChar[IndexAscii] - 61;
	}
	else if((AsciiChar[IndexAscii] >= 49)&&(AsciiChar[IndexAscii] <= 57)) /* num from 1 - 9 */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] =  AsciiChar[IndexAscii] - 19;

	}
	else if(AsciiChar[IndexAscii] == '\n' ) /* new line converted to enter key */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] = 88 ;
	}
	else if(AsciiChar[IndexAscii] == ' ' ) /* new line converted to enter key */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] = 44 ;
	}
	else if(AsciiChar[IndexAscii] == 48) /* num 0 */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] = 39 ;
	}
	else if(AsciiChar[IndexAscii] == '*') /* num 0 */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] = 85 ;
	}
	else if(AsciiChar[IndexAscii] == '#') /* num 0 */
	{
		/*we need to clear capslock */
		KeyBoardChar[IndexKeyboard] = 204 ;
	}
	else /* still unsupported characters by this driver */
	{
		IndexKeyboard--; /* just to escape this character because it still not supported */
	}

	/* go to next char */
	IndexAscii++;
	IndexKeyboard++;
	}

	/* terminate with NULL char */
	KeyBoardChar[IndexKeyboard] = 0 ;

}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	 static uint8_t index=0;



		/* every message received from bluetooth end with new line character */
	 if(AsciiChar[0]!= '\n')
	 {
		 ReceivedString[index++] = AsciiChar[0];
		 	 HAL_UART_Receive_IT(&huart2, AsciiChar, 1) ;
	 }

	/* call back function when complete DMA receive release the semaphore */
	else
	{
		ReceivedString[index] = 0; /* null terminate character */

		index=0;/* clear static ver */

		/* release semaphore to make HC-05 resume */
		osSemaphoreRelease(HC05SemaHandle) ;
	}

}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
