 /********************************************************************************
 *
 * File Name : main.c
 *
 * Description : FreeRTOS Application with External interrupt in PA5 Line
 * 				 that will Generate Interrupt to Toggle led on PC13
 *
 * Author : Shreef Mohamed
 *
 * Created on: March 21, 2020
 *
 ********************************************************************************/

/* Includes */
#include <stddef.h>
#include "string.h"
#include "stdio.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
/*****************************MACROS****************************/
#define TRUE 1
#define FALSE 0
#define PRESSED TRUE
#define NOT_PRESSED FALSE
/*****************************FUNCTION PPROTOTYPE****************************/
static void prvSetupHardware();
void prvGPIOSetup(void);
void prvUARTSetup();
void printmsg(char *msg);
void Button_Handler (void);
//Task Handler Prototype
void LED_Task_Handler (void *param);
/*****************************Global Variables****************************/
uint8_t button_status_flag = NOT_PRESSED ;


/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();

	// Create LED Task
	xTaskCreate(LED_Task_Handler,"LED-Task",configMINIMAL_STACK_SIZE,NULL,1,NULL);

	//Start Scheduler
	vTaskStartScheduler();


	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/
void LED_Task_Handler (void *param)
{
	while(1)
	{
		if(button_status_flag == PRESSED)
		{
			//Turn ON LED
			GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
		}
		else
		{
			//Turn OFF LED
			GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
		}
	}
}

void Button_Handler (void)
{
	// Toggling Status Flag that will cause Toggling the LED
	button_status_flag ^= 0x01 ;
}

void EXTI9_5_IRQHandler()
{
	traceISR_ENTER(); // for system View Tracing
	/*
	*1.Clear Interrupt Pending Bit of the EXTI Line
	* this bit is indication to the NVIC that interrupt is happened
	* then when interrupt Handler Called it must be Cleared
	*/
	EXTI_ClearITPendingBit(EXTI_Line5);
	//Call our Button Function Handler
	Button_Handler();
	traceISR_ENTER();
}
/*****************************FUNCTION Definition****************************/

static void prvSetupHardware(void)
{
	//Setup GPIO
	prvGPIOSetup();

	//setup UART_1
	prvUARTSetup();
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

void prvUARTSetup()
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

void prvGPIOSetup(void)
{
	//Enable The clok for GPIOA , GPIOC , AFIO Peripheral
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	// init structs
	GPIO_InitTypeDef led_init , Button_init ;
	EXTI_InitTypeDef EXTI_initStruct ;

	//initialize the LED PC13
	led_init.GPIO_Mode = GPIO_Mode_Out_PP ;
	led_init.GPIO_Pin = GPIO_Pin_13 ;
	led_init.GPIO_Speed = GPIO_Speed_2MHz ;
	GPIO_Init(GPIOC,&led_init);

	//initialize the Button PA5
	Button_init.GPIO_Mode = GPIO_Mode_IPD ; // input pull down
	Button_init.GPIO_Pin = GPIO_Pin_5 ;
	Button_init.GPIO_Speed = GPIO_Speed_2MHz ;
	GPIO_Init(GPIOA,&Button_init);

	//Interrupt Configuration for the Button (PA5)
	//1. System Configuration for EXTI LLine (AFIO Settings)
	AFIO->EXTICR[4]=AFIO_EXTICR4_EXTI13_PC;

	//2.EXTI Line init Struct Configuration settings
	EXTI_initStruct.EXTI_Line = EXTI_Line5 ;
	EXTI_initStruct.EXTI_Mode = EXTI_Mode_Interrupt ;
	EXTI_initStruct.EXTI_Trigger = EXTI_Trigger_Rising ;
	EXTI_initStruct.EXTI_LineCmd = ENABLE ;// unmask the interrupt
	EXTI_Init(&EXTI_initStruct);

	//3. NVIC Settings (IRQ Settings for the Selected EXTI Line)
	NVIC_SetPriority(EXTI9_5_IRQn,5);
	NVIC_EnableIRQ(EXTI9_5_IRQn);
}
