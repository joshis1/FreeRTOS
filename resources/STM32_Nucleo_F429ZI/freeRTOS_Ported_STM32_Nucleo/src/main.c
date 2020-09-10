/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/** free rtos includes **/
#include "freeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

void vTask1_handler(void *params);
void vTask2_handler(void *params);

static void prvSetupHardware(void);
static void prvSetupUart(void);

void printmsg(char *msg);

TaskHandle_t xTaskHandle1 = NULL;
TaskHandle_t xTaskHandle2 = NULL;

//test print msg
char usr_msg[256];
//Ends

#define TRUE 1
#define FALSE 0

#define AVAILABLE TRUE
#define UNAVAILABLE FALSE

int8_t uart_access_key_available = AVAILABLE;

static void prvSetupUart(void)
{
	GPIO_InitTypeDef gpio_uart_pins;
	USART_InitTypeDef uart3_init;

	//GPIO D is connected to AHB1 bus.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Enable the UART 3 peripheral clock -- connected to APB1 bus.
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	// zero the local variable
	memset(&gpio_uart_pins,0,sizeof(gpio_uart_pins));
	memset(&uart3_init,0,sizeof(uart3_init));
	// GPIO port D - pin 8 like TX
	// GPIO port D - ping 9 like RX
	gpio_uart_pins.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF; //AF - Alternate function i.e. TX and RX.
	gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP; //Pull up so that we see some default voltage.

	GPIO_Init(GPIOD, &gpio_uart_pins);

	// set the Pin 8 and Pin 9 to AF 7 i.e. GPIO_AF_USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8 , GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9 , GPIO_AF_USART3);

	uart3_init.USART_BaudRate = 115200;
	uart3_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	uart3_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	uart3_init.USART_Parity = USART_Parity_No;
	uart3_init.USART_StopBits = USART_StopBits_1;
	uart3_init.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART3,&uart3_init);
	//Enable USART 3 peripheral.
	USART_Cmd(USART3, ENABLE);

}

static void prvSetupHardware(void)
{
	prvSetupUart();
}

int main(void)
{
   // Enable DWT register for Segger view time stamps.
   // Enable Cycle counting.
   // Cycle count register.
	DWT->CTRL |= (1 << 0);
	//HSI ON, HSE OFF, PLL OFF, System clock to 16 Mhz, CPU clock 16 Mhz
	// AHB Prescaler value will be 1 after RCC_DeInit()

	//Step number 1 - DeInit the RCC - Reset Control Clock so that we don't use Pre-scaler or PLL.
	// Basically, we want to use lower speed.
	RCC_DeInit();
	// update the System Core Clock

	//Step number 2 -- Update the clock to use the default clock.
	SystemCoreClockUpdate();

	// Initialize peripherals like UART
	prvSetupHardware();

	sprintf(usr_msg, "This is Hello World -Shreyas Joshi \r\n");

	printmsg(usr_msg);

	//Segger view start to see the view
	SEGGER_SYSVIEW_Conf();
	SEGGER_SYSVIEW_Start();

	// Step number 3 -- create the task.
	xTaskCreate(vTask1_handler, "Task-1", 1024, NULL, 2,  &xTaskHandle1);
	xTaskCreate(vTask2_handler, "Task-2", 1024, NULL, 2,  &xTaskHandle2);

#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("Hello World from main!!\r\n");
#endif

	//Step number 4. -- start the scheduler.
	vTaskStartScheduler();  // this function will never return.
	for(;;);
}

void vTask1_handler(void *params)
{
	while(1)
	{
		if(uart_access_key_available == AVAILABLE)
		{
			uart_access_key_available = UNAVAILABLE;
			printmsg("This is Task 1 \r\n");
			//printf("Hello World from Task 1!!\r\n");
			uart_access_key_available = AVAILABLE;
			// task yield will release the CPU for other task in the ready state with
			// the same priority or higher priority to run.
			/**
			 * taskYIELD() is used to request a context switch to another task.
			 * However, if there are no other tasks at a higher or equal priority
			 * to the task that calls taskYIELD() then the RTOS scheduler
			 * will simply select the task that called taskYIELD() to run again.
			 *
			 */
			taskYIELD();
		}
	}
}

void vTask2_handler(void *params)
{
	while(1)
	{
		if(uart_access_key_available == AVAILABLE)
		{
			uart_access_key_available = UNAVAILABLE;
			printmsg("This is Task 2-->\r\n");
			uart_access_key_available = AVAILABLE;
			taskYIELD();
		}
	}
}

void printmsg(char *msg)
{
	for(uint32_t i = 0; i < strlen(msg); i++)
	{
		while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) != SET);
		USART_SendData(USART3, msg[i]);
    }
}


