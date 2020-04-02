 /********************************************************************************
 *
 * File Name : main.c
 *
 * Description : FreeRTOS Application with Sleep Mode
 * 				- 2 Tasks with Different Priority
 * 				- Task 1 Has low Priority  ---> Priority_2
 * 				- Task 2 has High Priority ---> Priority_3
				- vTaskDelay when it's called , the Running Task will be blocked
				- Then the Lower Priority Task can be Run on The CPU
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


//Task Handler Prototype
void vTask1_Handler (void *param);
void vTask2_Handler (void *param);
/*****************************Global Variables****************************/

TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL ;
char usr_msg[200];
/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();


	printmsg("This Task vTaskDelay Project\r\n");

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
	while(1)
	{
		// Print Status of The LED
		sprintf(usr_msg,"Status of The led is: %d\r\n",GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13));
		printmsg(usr_msg);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void vTask2_Handler (void *param)
{
	while(1)
	{
		// Toggle The LED Every 1 second
		GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
		vTaskDelay(pdMS_TO_TICKS(1000));

	}
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
	//Enable The clok for GPIOA , GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	GPIO_InitTypeDef led_init , Button_init ;

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
}

/*****************************Call Back Functions****************************/
void vApplicationIdleHook(void)
{
	// send the cpu to the Normal Sleep
	__WFI();
}

