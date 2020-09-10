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
#include "queue.h"
#include "timers.h"

void vTask1_menu_display(void *params);
void vTask2_cmd_handling(void *params);
void vTask3_cmd_processing(void *params);
void vTask4_uart_write(void *params);

void rtos_delay(uint32_t delay_in_ms);

static void prvSetupHardware(void);
static void prvSetupUart(void);

void printmsg(char *msg);

/**helper functions **/
static uint8_t getCommandCode(uint8_t *buffer);
static void getArguments(uint8_t *buffer);
static void make_led_on();
static void make_led_off();
static void make_led_toggle(uint32_t duration);
static void make_led_toggle_off();
static void read_led_status(char *msg);
static void read_rtc(char *msg);
static void print_error_message(char *msg);


#define TRUE 1
#define FALSE 0

#define NOT_PRESSED   FALSE
#define PRESSED       TRUE

#define AVAILABLE TRUE
#define UNAVAILABLE FALSE

typedef struct APP_CMD
{
   uint8_t command_num;
   uint8_t command_args[10];
}app_cmd_t;

//test print msg
char usr_msg[1024];

uint8_t command_buffer[20];
uint8_t command_len = 0;


int8_t uart_access_key_available = AVAILABLE;

TaskHandle_t task1_handle;
TaskHandle_t task2_handle;
TaskHandle_t task3_handle;
TaskHandle_t task4_handle;

//Queues
QueueHandle_t command_queue = NULL;
QueueHandle_t uart_write_queue = NULL;

//software timer
TimerHandle_t led_timer_handle = NULL;

typedef enum
{
	LED_ON = 1,
	LED_OFF,
	LED_TOGGLE,
	LED_TOGGLE_OFF,
	LED_READ_STATUS,
	RTC_PRINT_DATETIME,
	EXIT_APP
}command_enum_t;


char menu[]= {"\
\r\nLED_ON    --> 1 \
\r\nLED_OFF   --> 2  \
\r\nLED_TOGGLE -->3  \
\r\nLED_TOGGLE_OFF --> 4\
\r\nLED_READ_STATUS --> 5\
\r\nRTC_PRINT_DATETIME -->6\
\r\nEXIT_APP           -->0\
\r\nType your option here : "};


static uint8_t getCommandCode(uint8_t *buffer)
{
	return buffer[0] - 48;
}

static void getArguments(uint8_t *buffer)
{

}

void cb_timer_led_toggle(TimerHandle_t xtimer)
{
   GPIO_ToggleBits(GPIOB, GPIO_Pin_7);
}

static void make_led_on()
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);

}

static void make_led_off()
{
	GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
}

static void make_led_toggle(uint32_t duration)
{
   if(!led_timer_handle)
   {
	   led_timer_handle = xTimerCreate("LED-Timer", duration, pdTRUE,NULL, cb_timer_led_toggle);
   }
   xTimerStart(led_timer_handle, portMAX_DELAY);
}

static void make_led_toggle_off()
{
  xTimerStop(led_timer_handle, portMAX_DELAY);
}

static void read_led_status(char *msg)
{
	sprintf(msg,"\r\n LED Status is %d\r\n",GPIO_ReadOutputDataBit(GPIOB,GPIO_Pin_7));
	xQueueSend(uart_write_queue, &msg, portMAX_DELAY);
}

static void read_rtc(char *msg)
{
  RTC_TimeTypeDef rtc_time;
  RTC_DateTypeDef rtc_date;

  RTC_GetTime(RTC_Format_BIN, &rtc_time);
  RTC_GetDate(RTC_Format_BIN, &rtc_date);
  sprintf(msg,"\r\n Date is %d/%d/%d and time is %d %d %d \r\n",rtc_date.RTC_Date, rtc_date.RTC_Month, rtc_date.RTC_Year, rtc_time.RTC_Hours,
		  rtc_time.RTC_Minutes, rtc_time.RTC_Seconds);
  xQueueSend(uart_write_queue, &msg, portMAX_DELAY);

}

static void print_error_message(char *msg)
{
	sprintf(msg,"\r\n Invalid command received\r\n");
    xQueueSend(uart_write_queue, &msg, portMAX_DELAY);
}


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
	//Enable USART 3 interrupt

	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	// lets set the priority in NVIC for the UART 3 interrupt.
	// we cannot set priority 4 or below since we are calling FreeRTOS API from there.
	NVIC_SetPriority(USART3_IRQn,6);
	NVIC_EnableIRQ(USART3_IRQn);

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

	command_queue = xQueueCreate(10, sizeof(app_cmd_t *));
	uart_write_queue = xQueueCreate(10, sizeof(char*));

	if(command_queue != NULL && uart_write_queue != NULL)
	{

		// Step number 3 -- create the task.
		xTaskCreate(vTask1_menu_display, "TASK-1", 1024, NULL, 1,  &task1_handle);
		xTaskCreate(vTask2_cmd_handling, "TASK-2", 1024, NULL, 2,  &task2_handle);
		xTaskCreate(vTask3_cmd_processing, "TASK-3", 1024, NULL, 2, &task3_handle);
		xTaskCreate(vTask4_uart_write, "TASK-4", 1024, NULL, 2,  &task4_handle);
	}
	else
	{
		printmsg("Failed to create queue \r\n");
	}

#ifdef USE_SEMIHOSTING
	initialise_monitor_handles();
	printf("Hello World from main!!\r\n");
#endif

	//Step number 4. -- start the scheduler.
	vTaskStartScheduler();  // this function will never return.
	for(;;);
}

void vTask1_menu_display(void *params)
{
	printmsg("vTask1_menu_display  is running \r\n");
	char *pData = menu;
	while(1)
	{
      xQueueSend(uart_write_queue, &pData, portMAX_DELAY);
      xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
	}
}

void vTask2_cmd_handling(void *params)
{
	printmsg("vTask2_cmd_handling is running \r\n");
	uint8_t command_code;
	app_cmd_t *new_cmd;
	while(1)
	{
		new_cmd = pvPortMalloc(sizeof(app_cmd_t));
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

		//command_buffer is a shared resource between interrupt and this task.
		//serialize the usage of command_buffer to ensure there is no race condition
		//
		// taskEnter critical will disable the interrupt for the same priority or below priority.
		// those whose interrupt priority is higher will be allowed.
		taskENTER_CRITICAL();
		command_code = getCommandCode(command_buffer);
		new_cmd->command_num = command_code;
		getArguments(new_cmd->command_args);
		taskEXIT_CRITICAL();

		xQueueSend(command_queue,&new_cmd,portMAX_DELAY);
	}
}

void vTask3_cmd_processing(void *params)
{

	printmsg("vTask3_cmd_processing is running \r\n");
	app_cmd_t *new_cmd;
	command_enum_t cmd;
	char task_msg[50];

	char *p = &usr_msg;

	uint32_t toggle_duration = pdMS_TO_TICKS(500);

	while(1)
	{
		xQueueReceive(command_queue, &new_cmd, portMAX_DELAY);
		//printmsg("unblocked now \r\n");
		//sprintf(usr_msg, "SJ -command = %d\r\n",new_cmd->command_num);
		//xQueueSend(uart_write_queue, &p, portMAX_DELAY);
		switch(new_cmd->command_num)
		{

		case LED_ON:
			make_led_on();
			break;
		case LED_OFF:
			make_led_off();
			break;
		case LED_READ_STATUS:
			read_led_status(task_msg);
			break;
		case LED_TOGGLE:
			make_led_toggle(toggle_duration);
			break;
		case LED_TOGGLE_OFF:
			make_led_toggle_off();
			break;
		case RTC_PRINT_DATETIME:
			read_rtc(task_msg);
			break;
		default:
			print_error_message(task_msg);
            break;
		}
		//free the memory created by cmd_handling task.
        vPortFree(new_cmd);
	}
}

void vTask4_uart_write(void *params)
{
    char *pData = NULL;
	printmsg("vTask4_uart_write is running \r\n");
	while(1)
	{
       xQueueReceive(uart_write_queue, &pData, portMAX_DELAY);
       printmsg(pData);
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

void USART3_IRQHandler(void)
{
	uint16_t data_byte;
	BaseType_t higher_priority_task;
	if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE))
	{
		// a data byte is received from the user
		data_byte = USART_ReceiveData(USART3);
		//printmsg("Shreyas..got interrupt \r\n");
		// just read 1 byte
		command_buffer[command_len++] = data_byte & 0xff;
		//return key received
		if(data_byte == '\r')
		{
			command_len = 0;
			// then user is finished entering the data
			// let's notify the command handling task
			xTaskNotifyFromISR(task2_handle,0, eNoAction, &higher_priority_task);
			//let's notify the menu task
			xTaskNotifyFromISR(task1_handle,0, eNoAction, &higher_priority_task);

			//if the above freeRTOS APIs have waken up any higher priority task,
			// then yield the processor to the higher priority task which is just woken up.
			if(higher_priority_task)
			{
				taskYIELD();
			}
		}
	}

}
