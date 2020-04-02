 /********************************************************************************
 *
 * File Name : main.c
 *
 * Description : FreeRTOS Application with Binary Semaphore Tasks
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
#include <stdlib.h>
#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*****************************MACROS****************************/
#define TRUE 		1
#define FALSE 		0

/*****************************FUNCTION PPROTOTYPE****************************/
static void prvSetupHardware();
void prvGPIOSetup(void);
void prvUARTSetup();
void printmsg(char *msg);
void RTOS_Delay(uint32_t delay_in_ms);

//Task Handler Prototype
static void vManagerTask( void *pvParameters );
static void vEmployeeTask( void *pvParameters );

//Call back Function Prototype
void vApplicationIdleHook(void);
void EmployeeDoWork(unsigned char TicketId);
/*****************************Global Variables****************************/
char usr_msg[200]={0};

// Task Handles
TaskHandle_t xTaskHandle_Manger=NULL;
TaskHandle_t xTaskHandle_Employee=NULL ;

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
semaphore that is used to synchronize both manager and employee task */
xSemaphoreHandle xWork;

/* this is the queue which manager uses to put the work ticket id */
xQueueHandle xWorkQueue;

/*****************************Main Function****************************/
int main(void)
{
	// Initializations
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	prvSetupHardware();


	printmsg("\r\nThis is Binary Semaphore Project\r\n");

	/* Before a semaphore is used it must be explicitly created.
	 * In this example a binary semaphore is created .
	 */
	vSemaphoreCreateBinary( xWork );

	/* The queue is created to hold a maximum of 1 Element. */
    xWorkQueue = xQueueCreate( 1, sizeof( unsigned int ) );

	// check if the Creation of the Queue is successful or not

    /* Check the semaphore and queue was created successfully. */
    if( (xWork != NULL) && (xWorkQueue != NULL) )
    {

		/* Create the 'Manager' task.
		 *  This is the task that will be synchronized with the Employee task.
		 *  The Manager task is created with a high priority
		 */
        xTaskCreate( vManagerTask, "Manager", 300, NULL, 3, NULL );

        /* Create a employee task with less priority than manager */
        xTaskCreate( vEmployeeTask, "Employee", 300, NULL, 1, NULL );

        /* Start the scheduler so the created tasks start executing. */
        vTaskStartScheduler();
    }
	else
	{
	    sprintf(usr_msg,"Queue/Sema create failed.. \r\n");
	    printmsg(usr_msg);

	}
    /* If all is well we will never reach here as the scheduler will now be
    running the tasks.  If we do reach here then it is likely that there was
    insufficient heap memory available for a resource to be created. */

	while(1);
	return 0 ;
}


/***************************** Task Handler Definition****************************/
void vManagerTask( void *pvParameters )
{
	 unsigned int xWorkTicketId;
	 portBASE_TYPE xStatus;

   /* The semaphore is created in the 'empty' state, meaning the semaphore must
	 first be given using the xSemaphoreGive() API function before it
	 can subsequently be taken (obtained) */
   xSemaphoreGive( xWork);

   while(1)
   {
       /* get a work ticket id(some random number) */
       xWorkTicketId = ( rand() & 0x1FF );

		/* Sends work ticket id to the work queue */
       //Post an item on back of the queue
		xStatus = xQueueSend( xWorkQueue, &xWorkTicketId , portMAX_DELAY );

		if( xStatus != pdPASS )
		{
			sprintf(usr_msg,"Could not send to the queue.\r\n");
		    printmsg(usr_msg);

		}else
		{
			/* Manager notifying the employee by "Giving" semaphore */
			xSemaphoreGive( xWork);
			/* after assigning the work , just yield the processor because nothing to do */
			taskYIELD();

		}
   }
}

static void vEmployeeTask( void *pvParameters )
{
	unsigned char xWorkTicketId;
	portBASE_TYPE xStatus;
    /* As per most tasks, this task is implemented within an infinite loop. */
    while(1)
    {
		/* First Employee tries to take the semaphore,
		 * if it is available that means there is a task assigned by manager,
		 * otherwise employee task will be blocked
		 * second Argument is BlockTime it will be zero
		 * Block Time: if the Semaphore is Not Available,then how long this task has to block
		 */
		xSemaphoreTake( xWork, 0 );

		/* get the ticket id from the work queue */
		xStatus = xQueueReceive( xWorkQueue, &xWorkTicketId, 0 );

		if( xStatus == pdPASS )
		{
		  /* employee may decode the xWorkTicketId in this function to do the work*/
			EmployeeDoWork(xWorkTicketId);
		}
		else
		{
			/* We did not receive anything from the queue.  This must be an error as this task should only run when the manager assigns at least one work. */
			sprintf(usr_msg,"Employee task : Queue is empty , nothing to do.\r\n");
		    printmsg(usr_msg);
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

/*****************************Call Back Functions****************************/

void vApplicationIdleHook(void)
{
	// send the cpu to the Normal Sleep
	__WFI();
}

void EmployeeDoWork(unsigned char TicketId)
{
	/* implement the work according to TickedID */
	sprintf(usr_msg,"Employee task : Working on Ticked id : %d\r\n",TicketId);
	printmsg(usr_msg);
	vTaskDelay(pdMS_TO_TICKS(500));
}
