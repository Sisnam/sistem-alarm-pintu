#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

#define USART_SERIAL_EXAMPLE             &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE    9600
#define USART_SERIAL_CHAR_LENGTH         USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY              USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT            false

/* Task Definitions */
static portTASK_FUNCTION_PROTO(vCheckDoor, pvParameters);
static portTASK_FUNCTION_PROTO(vAlarmControl, pvParameters);
static portTASK_FUNCTION_PROTO(vPushButton, pvParameters);

/* Global Variables */
static char strbuf[128];
bool system_active = false;  // Removed volatile as semaphores handle synchronization
bool door_open = false;      // Removed volatile for the same reason
bool alarm_active = false;   // Removed volatile for the same reason
int counter = 0;

/* Semaphore and Mutex */
SemaphoreHandle_t xSemaphoreDoor;
SemaphoreHandle_t xMutexSystemActive;

/************************************************************************/
/* UART Configuration                                                   */
/************************************************************************/
void setUpSerial()
{
	USARTC0_BAUDCTRLB = 0; // Ensure BSCALE is 0
	USARTC0_BAUDCTRLA = 0x0C; // Set baud rate

	// Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	// 8 data bits, no parity, 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	// Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

/************************************************************************/
/* Initiate Single Slope PWM                                            */
/************************************************************************/
void PWM_Init(void)
{
	/* Set output for Buzzer */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = PIN1_bm;
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);

	/* Set Period */
	TCC0.PER = 1000;

	/* Set Compare Register value*/
	TCC0.CCA = 0;
}

/************************************************************************/
/* Resets all actuators (buzzer & LED) and counter                      */
/************************************************************************/
void reset_actuators(void)
{
	// Reset buzzer
	TCC0.CCA = 0;
	// Turn off LED
	LED_Off(LED0);
	LED_Off(LED1);
	// Reset counter
	counter = 0;
}

/************************************************************************/
/* Task: Push Button Handling                                           */
/************************************************************************/
static portTASK_FUNCTION(vPushButton, pvParameters) {
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;

	gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont);

	while (1) {
		if (!(PORTF.IN & PIN1_bm)) {  // SW1 toggle system state
			if (xSemaphoreTake(xMutexSystemActive, portMAX_DELAY) == pdTRUE) {
				system_active = !system_active;
				gfx_mono_draw_string(system_active ? "Sistem Aktif    " : "Sistem Nonaktif     ", 0, 8, &sysfont);
				if (!system_active) {
					reset_actuators();
					alarm_active = false;
				}
				xSemaphoreGive(xMutexSystemActive);
			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}
		if (!(PORTF.IN & PIN2_bm)) {  // SW2 reset actuators
			if (xSemaphoreTake(xMutexSystemActive, portMAX_DELAY) == pdTRUE) {
				if (system_active) {
					alarm_active = false;
					reset_actuators();
				}
				xSemaphoreGive(xMutexSystemActive);
			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Task: Check Door Status                                              */
/************************************************************************/
static portTASK_FUNCTION(vCheckDoor, pvParameters) {
	PORTE.DIRCLR = PIN0_bm;
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;

	bool prev_door_open = false; // Track previous state
	while (1) {
		bool is_open = PORTE.IN & PIN0_bm;

		//if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(100)) == pdTRUE) {
			if (is_open != prev_door_open) {
				prev_door_open = is_open;
				//if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(100)) == pdTRUE) {
					door_open = is_open;
					//xSemaphoreGive(xMutexSystemActive);
				//}
				snprintf(strbuf, sizeof(strbuf), "Pintu: %s", door_open ? "Terbuka  " : "Tertutup ");
				gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
				//if (xSemaphoreGive(xSemaphoreDoor) != pdTRUE) {
					xSemaphoreGive(xSemaphoreDoor);
				//}
			//}
			//xSemaphoreGive(xMutexSystemActive);
		}
		
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Task: Alarm Control                                                  */
/************************************************************************/
static portTASK_FUNCTION(vAlarmControl, pvParameters) {
	gfx_mono_draw_string("Waktu: 0            ", 0, 24, &sysfont);
	PWM_Init();

	while (1) {
		if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
			if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(250)) == pdTRUE) {
				gfx_mono_draw_string("MASUK COY       ", 0, 0, &sysfont);
				if (system_active && door_open) {
					gfx_mono_draw_string("HARUSNYA NYALA COY", 0, 0, &sysfont);
					alarm_active = true;
					} else if (alarm_active) {
						gfx_mono_draw_string("JANGAN MATI COYYY", 0, 0, &sysfont);
					} else {
					gfx_mono_draw_string("HARUSNYA MATI COY", 0, 0, &sysfont);
					alarm_active = false;
				}
				xSemaphoreGive(xMutexSystemActive);
			}
			xSemaphoreGive(xSemaphoreDoor);
		}

		if (alarm_active) {
			if (counter < 10) {
				TCC0.CCA = 500; // Low buzzer frequency
				LED_On(LED0);
				LED_On(LED1);
				} else {
				TCC0.CCA = 800; // High buzzer frequency
				LED_Toggle(LED0);
				LED_Toggle(LED1);
			}
			counter++;
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d           ", counter);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			} else {
			reset_actuators();
			gfx_mono_draw_string("Waktu: 0          ", 0, 24, &sysfont);
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Main Function                                                        */
/************************************************************************/
int main(void) {
	board_init();
	sysclk_init();
	pmic_init();
	gfx_mono_init();

	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("Sisnam+", 0, 0, &sysfont);

	// Initialize Semaphores
	xSemaphoreDoor = xSemaphoreCreateBinary();
	xMutexSystemActive = xSemaphoreCreateMutex();
	
	

	// Create Tasks
	xTaskCreate(vPushButton, "PushButton", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vCheckDoor, "CheckDoor", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

	// Start Scheduler
	vTaskStartScheduler();
}
