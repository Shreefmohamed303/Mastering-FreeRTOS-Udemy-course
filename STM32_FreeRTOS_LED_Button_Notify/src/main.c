
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
void LED_Task_Handler (void *param);
void Button_Task_Handler (void *param);
/*****************************Global Variables****************************/
uint8_t button_status_flag = NOT_PRESSED ;
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL ;
char usr_msg[200];
/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();

	printmsg("This Demo of Task Notify API \r\n");
	// Create LED Task
	xTaskCreate(LED_Task_Handler,"LED-Task",300,NULL,2,&xTaskHandle1);

	// Create Button Task
	xTaskCreate(Button_Task_Handler,"Button-Task",300,NULL,2,&xTaskHandle2);

	//Start Scheduler
	vTaskStartScheduler();


	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/
void LED_Task_Handler (void *param)
{
	// it will hold number of count that task is get Notified
	uint32_t current_Notification_Value=0 ;
	while(1)
	{
		// Lets Wait until we Received any Notification Event from Button Task
		//MaxDelay=xffffffff
		if(xTaskNotifyWait(0,0,&current_Notification_Value,portMAX_DELAY)==pdTRUE)
		{
			//Notifiaction is Received , Lets Toggle The LED
			if(button_status_flag==PRESSED)
			{
				GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
				button_status_flag=0;
			}

			else
			{
				GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
				button_status_flag=1;
			}

			printmsg("Notification is Received \r\n");
			sprintf(usr_msg,"Button Receive Count= %ld \r\n",current_Notification_Value);
			printmsg(usr_msg);
		}
	}
}

void Button_Task_Handler (void *param)
{
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) )
		{
			// Button is Pressed

			// lets  wait here for 100 ms for button Debouncing
			RTOS_Delay(100);

			// Lets send Notification to LED_task
			// and Increment the Notify Receiving Value
			xTaskNotify(xTaskHandle1,0x0,eIncrement );
		}

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

void RTOS_Delay(uint32_t delay_in_ms)
{
	// get the Global tick count value which is incremented by systic every 1ms
	uint32_t current_tick_count = xTaskGetTickCount();
	//configTICK_RATE_HZ/1000 will give us 1 (1ms)
	// Multiply 1 * delay_in_ms will get amount of delay in ms
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ)/1000 ;
	while(xTaskGetTickCount() < (current_tick_count + delay_in_ticks)) ;
}
