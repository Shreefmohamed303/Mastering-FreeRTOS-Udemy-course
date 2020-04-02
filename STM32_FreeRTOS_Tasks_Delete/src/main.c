
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

	printmsg("This Task Delete Project\r\n");
	// Create Task 1 low priority
	xTaskCreate(vTask1_Handler,"Task-1",300,NULL,1,&xTaskHandle1);

	// Create Task 2 high Priority
	xTaskCreate(vTask2_Handler,"Task-2",300,NULL,2,&xTaskHandle2);

	//Start Scheduler
	vTaskStartScheduler();


	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/
void vTask1_Handler (void *param)
{
	//Low Priority Task will run only when Task 2 is Deleted
	sprintf(usr_msg,"Task-1 is Running\r\n");
	printmsg(usr_msg);
	while(1)
	{
		// LED is Toggled every 1 sec
		//RTOS_Delay(200);

		/* We Use this Function as
		 * 1.it will make the 200ms  Delay
		 * 2.Remove task-1 for Ready Queue for 200 ms
		 * 3. When it Removed from Ready Queue then Idle Task will Run
		 * and idle Task's role is to Free Deleted Task from Memory
		 * (Remove TCB of task2) when it deleted
		 */
		vTaskDelay(200);
		GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
	}
}

void vTask2_Handler (void *param)
{
	// High Priority Task always will be running till it deleted
	sprintf(usr_msg,"Task-2 is Running\r\n");
	printmsg(usr_msg);
	while(1)
	{
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5) )
		{
			// Button is Pressed
			//task 2 delete itself
			sprintf(usr_msg,"Task-2 is Getting Deleted\r\n");
			printmsg(usr_msg);
			vTaskDelete(NULL);

		}
		else
		{
			// Button is Not Pressed
			// LED is Toggled every 1 sec
			RTOS_Delay(1000);
			GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
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
