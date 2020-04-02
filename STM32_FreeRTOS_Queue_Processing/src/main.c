 /********************************************************************************
 *
 * File Name : main.c
 *
 * Description : FreeRTOS Application with Queue Processing
 *
 * Author : Shreef Mohamed
 *
 * Created on: March 24, 2020
 *
 ********************************************************************************/
/* Includes */
#include <stddef.h>
#include "string.h"
#include "stdio.h"
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/*****************************MACROS****************************/
#define TRUE 1
#define FALSE 0

#define Exit_APP_COMMAND		0
#define LED_ON_COMMAND			1
#define LED_OFF_COMMAND			2
#define LED_TOGGLE_COMMAND		3
#define LED_TOGGLE_STOP			4
#define LED_READ_STATUS			5
#define RTC_READ_DATE_TIME		6

#define TOGGLE_DURATION_MS		500
/*****************************FUNCTION PPROTOTYPE****************************/
static void prvSetupHardware();
void prvGPIOSetup(void);
void prvUARTSetup();
void printmsg(char *msg);
void RTOS_Delay(uint32_t delay_in_ms);
uint8_t getCommandCode(uint8_t *buffer);

// Command Helper Function Prototype
void LED_ON(void);
void LED_OFF(void);
void LED_ToggleStart(uint32_t Duration);
void LED_ToggleStop(uint32_t Duration);
void LED_ReadStatus(char *task_mesg);
void RTC_ReadInfo(char *task_mesg);
void Print_Error_Message(char *task_mesg);
void Exit_App(void);

//Task Handler Prototype
void vTask1_menu_display (void *param);
void vTask2_cmd_handling (void *param);
void vTask3_cmd_processing (void *param);
void vTask4_uart_write (void *param);

// Software Timer Call back Prototype
void LED_SW_Timer_CallBack(TimerHandle_t xTimer);
void vApplicationIdleHook(void);
/*****************************Global Variables****************************/
char usr_msg[200]={0};

// Task Handles
TaskHandle_t xTaskHandle1=NULL;
TaskHandle_t xTaskHandle2=NULL ;
TaskHandle_t xTaskHandle3=NULL;
TaskHandle_t xTaskHandle4=NULL ;

// Queue Handle
QueueHandle_t comman_queue_handle = NULL ;
QueueHandle_t uart_write_queue_handle = NULL ;

// Software Timer Handle
TimerHandle_t LED_Timer_Handle=NULL;
// Command Structure
typedef struct APP_CMD
{
	uint8_t COMMAND_NUM ;
	uint8_t COMMAND_ARGS[10];

}APP_CMD_t;

uint8_t g_command_buffer[20];
uint8_t g_command_length=0 ;

char menu[]={"\
\r\nLED_ON				--->1 \
\r\nLED_OFF				--->2 \
\r\nLED_TOGGLE			--->3 \
\r\nLED_TOGGLE			--->4 \
\r\nLED_READ_STATUS 		--->5 \
\r\nRTC_PRINT_DATETIME 		--->6 \
\r\nExit_APP			--->0 \
\r\nType your Option Here : "};

/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();


	printmsg("\r\nThis Queue Command Processing Project\r\n");

	//Creation of Command Queue of 10 pointer to APP_CMD Structure
	//Using Pointer to minimize heap allocation memory it will have 4*10 = 40bytes
	comman_queue_handle = xQueueCreate(10,sizeof(APP_CMD_t *));

	// Creation of UART_Write Queue of 10 pointer to char
	uart_write_queue_handle = xQueueCreate(10,sizeof(char *));


	// check if the Creation of the Queue is successful or not
	if( (comman_queue_handle != NULL) && (uart_write_queue_handle != NULL) )
	{
	// Create Task 1
	xTaskCreate(vTask1_menu_display,"TASK1-MENU",300,NULL,1,&xTaskHandle1);

	// Create Task 2
	xTaskCreate(vTask2_cmd_handling,"TASK2-CMD-HANDLING",300,NULL,2,&xTaskHandle2);

	// Create Task 3
	xTaskCreate(vTask3_cmd_processing,"TASK3-CMD-PROCESSING",300,NULL,2,&xTaskHandle3);

	// Create Task 4
	xTaskCreate(vTask4_uart_write,"TASK4-UART-WRITE",300,NULL,2,&xTaskHandle4);

	//Start Scheduler
	vTaskStartScheduler();

	}
	else
	{
		sprintf(usr_msg,"Queue Creation Faild \r\n");
		printmsg(usr_msg);
	}

	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/

void vTask1_menu_display (void *param)
{
	char *pData = menu;
	while(1)
	{
		// second Parameter is address of Pointer to the Data
		xQueueSend(uart_write_queue_handle,&pData,portMAX_DELAY);

		// lets wait here until someone Notifies
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

	}
}
void vTask2_cmd_handling (void *param)
{
	APP_CMD_t * new_cmd;
	while(1)
	{
		// wait until Getting notified from UART ISR
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		new_cmd= (APP_CMD_t*)pvPortMalloc(sizeof(APP_CMD_t));
		//it's a Critical Section as we accessing Global Data
		// Global Data is shared between interrupt and command Handling Task
		// "g_command_buffer"  can be updated from ISR at any time
		// Here there is a Chance of Race condition
		//so, Solution is to Disable Interrupts until finishing accessing Global Variables

		taskENTER_CRITICAL(); // Disable Interrupts
		new_cmd->COMMAND_NUM= getCommandCode(g_command_buffer);
		taskEXIT_CRITICAL(); // Enable Interrupts

		// Send Command to the COMMAND_QUEUE
		xQueueSend(comman_queue_handle,&new_cmd,portMAX_DELAY);
	}
}
void vTask3_cmd_processing (void *param)
{
	APP_CMD_t * new_cmd;
	char task_mesg[50];
	uint32_t Toggle_Duration_Ticks= pdMS_TO_TICKS(TOGGLE_DURATION_MS);
	while(1)
	{
		// Receive Command from COMMAND QUEUE
		xQueueReceive(comman_queue_handle,(void*)&new_cmd,portMAX_DELAY);

		// Response to The Command which is get from User
		switch(new_cmd->COMMAND_NUM)
		{
		case LED_ON_COMMAND:  LED_ON();
							break ;
		case LED_OFF_COMMAND: LED_OFF();
							break ;
		case LED_TOGGLE_COMMAND: LED_ToggleStart(Toggle_Duration_Ticks);
							break ;
		case LED_TOGGLE_STOP: LED_ToggleStop(Toggle_Duration_Ticks);
							break ;
		case LED_READ_STATUS: LED_ReadStatus(task_mesg);
							break ;
		case RTC_READ_DATE_TIME: RTC_ReadInfo(task_mesg);
							break ;
		case Exit_APP_COMMAND : Exit_App();
							break ;
		default: Print_Error_Message(task_mesg);
		}

		// Lets Free The Allocated Memory for the "new_cmd"
		vPortFree(new_cmd);

	}
}
void vTask4_uart_write (void *param)
{
	char *pData=NULL ;
	while(1)
	{
		// Read data From The UART Queue
		xQueueReceive(uart_write_queue_handle,&pData,portMAX_DELAY);

		//Print The Data via UART
		printmsg(pData);
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

    /* Enable USART 1 Byte Reception Interrupt in Micro_controller */
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);

    /* Set The Priority in NVIC for USART1 interrupt */
    NVIC_SetPriority(USART1_IRQn,12);

    /* Enable The USART1 IRQ in the NVIC  */
    NVIC_EnableIRQ(USART1_IRQn);

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

	// turn OFF The LED at the First ** LED is Active LOW **
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);

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

uint8_t getCommandCode(uint8_t *buffer)
{
	// Convert ASCII to Numerical Value
	return buffer[0]-48;
}
/*****************************Call Back Functions****************************/
void USART1_IRQHandler(void)
{
	uint16_t uart_data_byte;
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

	// check if the callback function is called because of Receiving or else
	if( USART_GetFlagStatus(USART1,USART_FLAG_RXNE) )
	{
		// Data Byte is Received from the user
		uart_data_byte=USART_ReceiveData(USART1);
		// Get the data into The Buffer and save only the LSB
		g_command_buffer[g_command_length++]= uart_data_byte & 0xFF;

		// check if the User press Enter key "End of The Transmission"
		if(uart_data_byte=='\r')
		{
			// User Finishing Enter The Data

			//1. Reset The Command Length Variable
			g_command_length=0;

			//2. Lets Notify the Command Handling Task
			xTaskNotifyFromISR(xTaskHandle2,0,eNoAction,&pxHigherPriorityTaskWoken) ;

			//3. Lets Notify the Menu_Display Handling Task
			xTaskNotifyFromISR(xTaskHandle1,0,eNoAction,&pxHigherPriorityTaskWoken) ;
		}
	}
	// if the above FREE_RTOS APIs Wake up any higher priority Task
	// then yield the processor to the higher priority Task which is just waken up
	if(pxHigherPriorityTaskWoken)
	{
		taskYIELD();
	}
}
void LED_SW_Timer_CallBack(TimerHandle_t xTimer)
{
	GPIO_ToggleBit(GPIOC,GPIO_Pin_13);
}
void vApplicationIdleHook(void)
{
	// send the cpu to the Normal Sleep
	__WFI();
}

/*********************Command Helper Function Definition*********************/
void LED_ON(void)
{
	// LED on The Board is Active Low
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_RESET);
}
void LED_OFF(void)
{
	// LED on The Board is Active Low
	GPIO_WriteBit(GPIOC,GPIO_Pin_13,Bit_SET);
}
void LED_ToggleStart(uint32_t Duration)
{
	if(LED_Timer_Handle==NULL)
	{
		//It's First Enter To That Function

		//1. Lets Create The Software Timer
		LED_Timer_Handle=xTimerCreate("LED-TIMER",Duration,pdTRUE,NULL,LED_SW_Timer_CallBack);

		//2. Start The Software Timer
		xTimerStart(LED_Timer_Handle,portMAX_DELAY);
	}
	else
	{
		// Start The Software Timer
		xTimerStart(LED_Timer_Handle,portMAX_DELAY);
	}
}
void LED_ToggleStop(uint32_t Duration)
{
	if(xTimerIsTimerActive(LED_Timer_Handle) == pdTRUE)
	{
		xTimerStop(LED_Timer_Handle,portMAX_DELAY);
	}
}
void LED_ReadStatus(char *task_mesg)
{
	//put The Required Message to be Display in the task_mesg
	sprintf(task_mesg,"\r\nLED Status is : %d\r\n",GPIO_ReadOutputDataBit(GPIOC,GPIO_Pin_13));

	//Update The UART_WRITE_QUEUE with the New message to be Displayed
	xQueueSend(uart_write_queue_handle,&task_mesg,portMAX_DELAY);
}
void RTC_ReadInfo(char *task_mesg)
{
	sprintf(task_mesg,"\r\nNot Implemented Yet\r\n");
	//Update The UART_WRITE_QUEUE with the New message to be Displayed
	xQueueSend(uart_write_queue_handle,&task_mesg,portMAX_DELAY);
}
void Print_Error_Message(char *task_mesg)
{
	//put The Required Message to be Display in the task_mesg
	sprintf(task_mesg,"\r\nInvalid Command Received !!\r\n");

	//Update The UART_WRITE_QUEUE with the New message to be Displayed
	xQueueSend(uart_write_queue_handle,&task_mesg,portMAX_DELAY);
}
void Exit_App(void)
{
	//1. Deleting All Tasks
	vTaskDelete(xTaskHandle1);
	vTaskDelete(xTaskHandle2);
	vTaskDelete(xTaskHandle3);
	vTaskDelete(xTaskHandle4);

	//2. Disable All interrupts
	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);

	//3. Send CPU to the Sleep Mode by using IdleHook Function

}
