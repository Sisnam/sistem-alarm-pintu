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

/* Global Variables */
static char strbuf[128];
volatile bool system_active = false;
volatile bool alarm_active = false;
volatile bool door_open = false;
static uint16_t counter = 0;
volatile bool access_granted = false;

/* Semaphore */
SemaphoreHandle_t xSemaphoreDoor;
SemaphoreHandle_t xSemaphoreSystem;
SemaphoreHandle_t xSemaphoreAlarm;

// Function Prototypes
void reset_actuators(void);
void init_pwm(void);

/* Task Definitions */
static portTASK_FUNCTION_PROTO(vTaskButtonSW1, pvParameters);
static portTASK_FUNCTION_PROTO(vTaskButtonSW2, pvParameters);
static portTASK_FUNCTION_PROTO(vTaskDoorSensor, pvParameters);
static portTASK_FUNCTION_PROTO(vTaskAlarmControl, pvParameters);

/************************************************************************/
/* PWM Initialization for Servo and Buzzer                             */
/************************************************************************/
void PWM_Init(void)
{
	/* PWM for Buzzer - TCC0 Channel A */
	PORTC.DIR |= PIN0_bm; // Set PORTC PIN0 as output for Servo
	TCC0.CTRLA = PIN1_bm; //(PIN2_bm) | (PIN0_bm);
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);
	TCC0.PER = 1000; // 20ms period (50Hz)
	TCC0.CCA = 0;  // Initial duty cycle for 0 degrees
}

/************************************************************************/
/* Resets all actuators (buzzer & LED) and counter                      */
/************************************************************************/
void reset_actuators(void)
{
	// Turn Off Buzzer
	TCC0.CCA = 0;
	
	// Turn off LED
	LED_Off(LED0);
	LED_Off(LED1);
	
	// Reset counter
	counter = 0;
}


/************************************************************************/
/* Task: Push Button SW1 Handling                                       */
/************************************************************************/
static portTASK_FUNCTION(vTaskButtonSW1, pvParameters) {
	PORTF.DIRCLR = PIN1_bm;  // Set SW1 as input
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc; // Falling edge detection

	while (1) {
		if (!(PORTF.IN & PIN1_bm)) {  // If SW1 is pressed
			if (xSemaphoreTake(xSemaphoreSystem, portMAX_DELAY) == pdTRUE) {
				system_active = !system_active;  // Toggle system state
				if (!system_active) {
					reset_actuators();
					gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont);
					} else {
					gfx_mono_draw_string("Sistem Aktif   ", 0, 8, &sysfont);
				}
				xSemaphoreGive(xSemaphoreSystem);
			}
			vTaskDelay(100 / portTICK_PERIOD_MS); // Debounce delay
		}
		vTaskDelay(50 / portTICK_PERIOD_MS); // Periodic check
	}
}


/************************************************************************/
/* Task: Push Button SW2 Handling                                       */
/************************************************************************/
static portTASK_FUNCTION(vTaskButtonSW2, pvParameters) {
	PORTF.DIRCLR = PIN2_bm;  // Set SW2 as input
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc; // Falling edge detection

	while (1) {
		if (!(PORTF.IN & PIN2_bm)) {  // If SW2 is pressed
			if (xSemaphoreTake(xSemaphoreAlarm, portMAX_DELAY) == pdTRUE) {
				alarm_active = false;  // Deactivate alarm
				reset_actuators();     // Reset actuators (buzzer, LED)
				gfx_mono_draw_string("Alarm Dihentikan", 0, 16, &sysfont);
				xSemaphoreGive(xSemaphoreAlarm);
			}
			vTaskDelay(100 / portTICK_PERIOD_MS); // Debounce delay
		}
		vTaskDelay(50 / portTICK_PERIOD_MS); // Periodic check
	}
}

/************************************************************************/
/* Task: Deteksi Pintu Menggunakan Sensor Magnetik MC-38                */
/************************************************************************/
static portTASK_FUNCTION(vTaskDoorSensor, pvParameters) {
	PORTE.DIRCLR = PIN0_bm;  // Set PIN0 (sensor door) as input
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc; // Enable pull-up resistor

	while (1) {
		if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
			// Read sensor state (active low: 0 = door closed, 1 = door open)
			door_open = (PORTE.IN & PIN0_bm) ? true : false;
			xSemaphoreGive(xSemaphoreDoor);
			// Update status on LCD
			if (door_open) {
				gfx_mono_draw_string("Pintu Terbuka  ", 0, 16, &sysfont);
				} else {
				gfx_mono_draw_string("Pintu Tertutup ", 0, 16, &sysfont);
			}
			
			
		}
		vTaskDelay(100 / portTICK_PERIOD_MS); // Check door status every 100ms
	}
}

static portTASK_FUNCTION(vTaskAlarmControl, pvParameters) {
	PWM_Init(); // Initialize buzzer PWM

	while (1) {
		gfx_mono_draw_string("Alarm Task Start", 0, 0, &sysfont);

		if (xSemaphoreTake(xSemaphoreSystem, portMAX_DELAY) == pdTRUE) {
			gfx_mono_draw_string("System Check   ", 0, 8, &sysfont);

			if (system_active) {
				if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
					gfx_mono_draw_string("Door Check     ", 0, 16, &sysfont);

					if (door_open && !alarm_active) {
						alarm_active = true;

						// Activate buzzer and LED
						TCC0.CCA = 500;  // Set buzzer frequency
						LED_On(LED0);
						LED_On(LED1);
						gfx_mono_draw_string("ALARM AKTIF!   ", 0, 24, &sysfont);
						} else if (!door_open && alarm_active) {
						alarm_active = false;

						// Deactivate buzzer and LED
						reset_actuators();
						gfx_mono_draw_string("ALARM MATI     ", 0, 24, &sysfont);
					}

					xSemaphoreGive(xSemaphoreDoor);
				}
				} else {
				alarm_active = false;
				reset_actuators();
				gfx_mono_draw_string("Sistem Nonaktif", 0, 24, &sysfont);
			}

			xSemaphoreGive(xSemaphoreSystem);
		}

		vTaskDelay(100 / portTICK_PERIOD_MS); // Check alarm status every 100ms
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
	xSemaphoreSystem = xSemaphoreCreateMutex();
	xSemaphoreAlarm = xSemaphoreCreateMutex();
	xSemaphoreDoor = xSemaphoreCreateMutex();
	
	if (xSemaphoreSystem == NULL || xSemaphoreAlarm == NULL || xSemaphoreDoor == NULL) {
	    gfx_mono_draw_string("Semaphore Failed", 0, 0, &sysfont);
	    while (1); // Semaphore creation failed
	}
		
		
	// Create Tasks
	xTaskCreate(vTaskButtonSW1, "PushButtonSW1", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vTaskButtonSW2, "PushButtonSW2", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vTaskDoorSensor, "DoorSensor", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	//xTaskCreate(vTaskAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);


	// Start Scheduler
	vTaskStartScheduler();
}