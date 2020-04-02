/* Includes */
#include <stddef.h>
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "stdio.h"
#include "string.h"
#include "task.h"
#include "FreeRTOSConfig.h"


// Macros
//address of the register
volatile unsigned int *DWT_CYCCNT   = (volatile unsigned int *)0xE0001004;

//address of the register
volatile unsigned int *DWT_CONTROL  = (volatile unsigned int *)0xE0001000;

#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define TRUE 1
#define FALSE 0
#define AVAILABLE TRUE
#define NOT_AVAILABLE FALSE

// Global Variables
uint8_t ACCESS_KEY = AVAILABLE;
FunctionalState NewState=0;
// Task Handle instance
TaskHandle_t xTaskHandle1=NULL ;
TaskHandle_t xTaskHandle2=NULL ;

/****************************** FUNCTION PROTOTYPE ************************************/
// Task Handler Functions Prototype
void vTask1_handler(void *params);
void vTask2_handler(void *params);
// Setup Functions Prototype
static void prvSetupHardware(void);
//print UART mesg Function
void printmsg(char *msg);
void UART1_init();




int main(void)
{

	/*1. Resets the RCC clock configuration to the default reset state.
	 * 	HSI ON , PLL OFF , HSE OFF
	 * 	System clock = 8Mhz
	 * 	CPU clock = 8Mhz
	 */
//	RCC_DeInit();
//
//	//2. Update SystemCoreClock variable
//	SystemCoreClockUpdate();

	// Enable clok counting Register(DWT section) to make time stamp in system Viewer
	*DWT_CYCCNT = 0; // reset the counter
	*DWT_CONTROL |= 1 ; // enable the counte
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();

	// Start Recording Tasks in System Viewer
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();
	//3. lets Create 2 tasks , task-1 , task-2
	xTaskCreate( vTask1_handler,"Task-1",configMINIMAL_STACK_SIZE,NULL,( tskIDLE_PRIORITY + 1 ),&xTaskHandle1 );
	xTaskCreate( vTask2_handler,"Task-2",configMINIMAL_STACK_SIZE,NULL,( tskIDLE_PRIORITY + 1 ),&xTaskHandle2 );

	/* Start the Schedular */
	vTaskStartScheduler();

  /* Infinite loop */
  while (1);
}


/***************************** Task Handler Definition****************************/
void vTask1_handler(void *params)
{
	while(1)
	{
		if(ACCESS_KEY==AVAILABLE)
		{
			ACCESS_KEY=NOT_AVAILABLE;
			printmsg("Hello form Task1\r\n");
			ACCESS_KEY=AVAILABLE;
			taskYIELD();
		}
	}
}

void vTask2_handler(void *params)
{
	while(1)
	{
		if(ACCESS_KEY==AVAILABLE)
		{
			ACCESS_KEY=NOT_AVAILABLE;
			printmsg("Hello form Task2\r\n");
			ACCESS_KEY=AVAILABLE;
			taskYIELD();
		}
	}
}

/*****************************FUNCTION Definition****************************/

static void prvSetupHardware(void)
{
	//setup UART_1
	UART1_init();
}

void printmsg(char *msg)
{
	for(uint32_t i=0 ; i<strlen(msg) ; i++)
	{
		//polling to check that the Date Register is Empty
		while(USART_GetFlagStatus(USART1, USART_FLAG_TXE)!=SET);
		//put our mesg in the UART Data Register
		USART_SendData(USART1,msg[i]);
	}

}


void UART1_init()
{
    /* USART configuration structure for USART1 */
    USART_InitTypeDef usart1_init_struct;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;

    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA, ENABLE);

    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);

    /* Enable USART1 */
    USART_Cmd(USART1, ENABLE);
    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1_init_struct);
}
