 /********************************************************************************
 *
 * File Name : main.c
 *
 * Description : FreeRTOS Application with
 * 				- 2 Tasks with Different Priority
 * 				- Task 1 Has low Priority  ---> Priority_2
 * 				- Task 2 has High Priority ---> Priority_3
 * 				- Task 2 will always Run as it's highest Priority
 * 				- External interrupt in PA5 Line (push Button)
 * 				- that will Generate Interrupt to Switch Tasks Priority
 * 				- "Every Button Click The Two Task's Priority will be Switched"
 *
 * Author : Shreef Mohamed
 *
 * Created on: March 23, 2020
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
void RTOS_Delay(uint32_t delay_in_ms);

//Task Handler Prototype
void vTask1_Handler (void *param);
void vTask2_Handler (void *param);
/*****************************Global Variables****************************/
uint8_t switching_priority_flag = FALSE ;
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL ;
char usr_msg[200];
/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();


	printmsg("This Task Switching Priority Project\r\n");

	// Create Task 1 low priority --> 2
	xTaskCreate(vTask1_Handler,"Task-1",300,NULL,2,&xTaskHandle1);

	// Create Task 2 high Priority --> 3
	xTaskCreate(vTask2_Handler,"Task-2",300,NULL,3,&xTaskHandle2);

	//Start Scheduler
	vTaskStartScheduler();

	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/
void vTask1_Handler (void *param)
{
	UBaseType_t p1 , p2 ;
	//Low Priority Task will run only when Task 2 is Deleted
	sprintf(usr_msg,"Task-1 is Running\r\n");
	printmsg(usr_msg);

	// Print Priority of Task 1
	sprintf(usr_msg,"Task-1 Priority %ld\r\n",uxTaskPriorityGet(xTaskHandle1));
	printmsg(usr_msg);

	// Print Priority of Task 1
	sprintf(usr_msg,"Task-2 Priority %ld\r\n",uxTaskPriorityGet(xTaskHandle2));
	printmsg(usr_msg);
	while(1)
	{
		if(switching_priority_flag)
		{
			// Button is Pressed
			switching_priority_flag=FALSE ;
			//Get the Priority of The 2 Tasks in Variables p1,p2
			p1 = uxTaskPriorityGet(xTaskHandle1);
			p2 = uxTaskPriorityGet(xTaskHandle2);

			//Switching Priority of The 2 Tasks
			vTaskPrioritySet(xTaskHandle1,p2);
			vTaskPrioritySet(xTaskHandle2,p1);
		}
		else
		{
			// Button is Not Pressed

			// LED is Toggled every 100 ms
			RTOS_Delay(100);
			GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
		}
	}
}

void vTask2_Handler (void *param)
{
	UBaseType_t p1 , p2 ;
	//Low Priority Task will run only when Task 2 is Deleted
	sprintf(usr_msg,"Task-2 is Running\r\n");
	printmsg(usr_msg);

	// Print Priority of Task 1
	sprintf(usr_msg,"Task-1 Priority %ld\r\n",uxTaskPriorityGet(xTaskHandle1));
	printmsg(usr_msg);

	// Print Priority of Task 1
	sprintf(usr_msg,"Task-2 Priority %ld\r\n",uxTaskPriorityGet(xTaskHandle2));
	printmsg(usr_msg);
	while(1)
	{
		if(switching_priority_flag)
		{
			// Button is Pressed
			switching_priority_flag=FALSE ;
			//Get the Priority of The 2 Tasks in Variables p1,p2
			p1 = uxTaskPriorityGet(xTaskHandle1);
			p2 = uxTaskPriorityGet(xTaskHandle2);

			//Switching Priority of The 2 Tasks
			vTaskPrioritySet(xTaskHandle1,p2);
			vTaskPrioritySet(xTaskHandle2,p1);
		}
		else
		{
			// Button is Not Pressed

			// LED is Toggled every 1 second
			RTOS_Delay(1000);
			GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
		}
	}
}

void EXTI9_5_IRQHandler()
{
	//traceISR_ENTER(); // for system View Tracing
	/*
	*1.Clear Interrupt Pending Bit of the EXTI Line
	* this bit is indication to the NVIC that interrupt is happened
	* then when interrupt Handler Called it must be Cleared
	*/
	EXTI_ClearITPendingBit(EXTI_Line5);

	// Switching Priority Between 2 tasks
	switching_priority_flag=TRUE ;

	//traceISR_ENTER(); // for system View Tracing
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
    usart1_init_struct.USART_BaudRate = 115200;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1,&usart1_init_struct);
}

void prvGPIOSetup(void)
{
	//Enable The clok for GPIOA , GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitTypeDef led_init , Button_init ;
	EXTI_InitTypeDef EXTI_initStruct;

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

	//1. System Configuration for EXTI Line (AFIO Settings)
	//Multiplexor select line 0(PA5)
	//check Reference Manual section External interrupt configuration register
	AFIO->EXTICR[4]=AFIO_EXTICR2_EXTI5_PA;

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

void RTOS_Delay(uint32_t delay_in_ms)
{
	// get the Global tick count value which is incremented by systic every 1ms
	uint32_t current_tick_count = xTaskGetTickCount();

	//configTICK_RATE_HZ/1000 will give us 1 (1ms)
	// Multiply 1 * delay_in_ms will get amount of delay in ms
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ)/1000 ;
	while(xTaskGetTickCount() < (current_tick_count + delay_in_ticks)) ;
}
