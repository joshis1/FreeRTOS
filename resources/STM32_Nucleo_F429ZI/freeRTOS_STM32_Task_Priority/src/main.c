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
void rtos_delay(uint32_t delay_in_ms);

static void prvSetupHardware(void);
static void prvSetupUart(void);

void printmsg(char *msg);


#define TRUE 1
#define FALSE 0

#define NOT_PRESSED   FALSE
#define PRESSED       TRUE

#define AVAILABLE TRUE
#define UNAVAILABLE FALSE

//test print msg
char usr_msg[256];

int8_t uart_access_key_available = AVAILABLE;

int8_t switch_prio = FALSE;

TaskHandle_t task1_handle;
TaskHandle_t task2_handle;

void prvSetupGpio(void)
{
	//GPIO B and GPIO C is hanging over the AHB Bus.
	// initialize AHB bus then.

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// Enable the APB2 - clock to EXTI block
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	// Blue LED is PB7
	GPIO_InitTypeDef led_init, button_init;
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	// PushPull, don't use Open Drain
	// cannot use OD - then you have to use give external
	//resistor to drive it high or low
	led_init.GPIO_OType= GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_7;
	led_init.GPIO_Speed = GPIO_Low_Speed;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &led_init);

	//Button PC13 - B1 - user button  - blue button in my board.
	button_init.GPIO_Mode = GPIO_Mode_IN;
	// PushPull, don't use Open Drain
	// cannot use OD - then you have to use give external
	//resistor to drive it high or low
	button_init.GPIO_OType= GPIO_OType_PP;
	button_init.GPIO_Pin = GPIO_Pin_13;
	button_init.GPIO_Speed = GPIO_Low_Speed;
	button_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &button_init);

	//interrupt configuration for the button (PC13)

	//EXTI line is connected to NVIC so first configure EXTI
	// EXTI is like a mux and is hanging on APB2 bus

    //Mux settings enable the GPIO C and 13 line i.e C13
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    EXTI_InitTypeDef exti_init;
    exti_init.EXTI_Mode = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger = EXTI_Trigger_Falling;
    exti_init.EXTI_Line = EXTI_Line13;
    exti_init.EXTI_LineCmd = ENABLE;
    EXTI_Init(&exti_init);

    //NVIC Settings
    //Check in the specs where the EXTI-13 is connected to IRQ.
    //Position 40 -- in the reference manual - Table - 62
    // priorities are from 0 to 15 - 0 being the highest
    // this priority is interrupt priority and not task priority.
    NVIC_SetPriority(EXTI15_10_IRQn, 5);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}
//ISR - defined in startup_stm32.s - can be overridden here.
// this is .weak that means can be overridden.

void EXTI15_10_IRQHandler(void)
{
	//1. clear the interrupt pending bit.
    EXTI_ClearITPendingBit(EXTI_Line13);
    printmsg("Switching the priority -- IRQ Handler\r\n");
    switch_prio = TRUE;

}


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
	prvSetupGpio();
}

//For LED, configure the GPIO to output mode
//STM32F49ZI

int main(void)
{
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

	sprintf(usr_msg, "This is Free RTOS LED Button -Shreyas Joshi \r\n");

	printmsg(usr_msg);

	// Step number 3 -- create the task.
	xTaskCreate(vTask1_handler, "TASK-1", 1024, NULL, 2,  &task1_handle);
	xTaskCreate(vTask2_handler, "TASK-2", 1024, NULL, 3,  &task2_handle);

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
	UBaseType_t p1,p2;

	printmsg("TASK 1 is running \r\n");

	sprintf(usr_msg, "Task 1 priority is %ld \r\n",uxTaskPriorityGet(task1_handle));
	printmsg(usr_msg);

	sprintf(usr_msg, "Task 2 priority is %ld \r\n",uxTaskPriorityGet(task2_handle));
    printmsg(usr_msg);

	while(1)
	{
		if(switch_prio)
		{
			switch_prio = FALSE;
			p1 = uxTaskPriorityGet(task1_handle);
			p2 = uxTaskPriorityGet(task2_handle);
			vTaskPrioritySet(task1_handle, p2);
			vTaskPrioritySet(task2_handle, p1);
		}
		else
		{
			GPIO_ToggleBits(GPIOB, GPIO_Pin_7);
			rtos_delay(200);
		}
	}
}

void vTask2_handler(void *params)
{
	UBaseType_t p1,p2;

	printmsg("TASK 2 is running \r\n");

	sprintf(usr_msg, "Task 1 priority is %ld \r\n",uxTaskPriorityGet(task1_handle));
	printmsg(usr_msg);

	sprintf(usr_msg, "Task 2 priority is %ld \r\n",uxTaskPriorityGet(task2_handle));
    printmsg(usr_msg);

	while(1)
	{
		if(switch_prio)
		{
			switch_prio = FALSE;
			p1 = uxTaskPriorityGet(task1_handle);
			p2 = uxTaskPriorityGet(task2_handle);
			vTaskPrioritySet(task1_handle, p2);
			vTaskPrioritySet(task2_handle, p1);
		}
		else
		{
			GPIO_ToggleBits(GPIOB, GPIO_Pin_7);
			rtos_delay(1000);
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

void rtos_delay(uint32_t delay_in_ms)
{
	uint32_t current_tick_count = xTaskGetTickCount();
	uint32_t delay_in_ticks = (delay_in_ms * configTICK_RATE_HZ)/1000;
    while(xTaskGetTickCount() <  current_tick_count + delay_in_ticks);
}
