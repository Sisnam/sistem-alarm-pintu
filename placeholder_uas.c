#include <asf.h>
#include <stdio.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

/* Task Definitions */
static portTASK_FUNCTION_PROTO(vCheckDoor, pvParameters);
static portTASK_FUNCTION_PROTO(vAlarmControl, pvParameters);
static portTASK_FUNCTION_PROTO(vDisplayStatus, pvParameters);
static portTASK_FUNCTION_PROTO(vPushButton, pvParameters);

/* Global Variables */
static char strbuf[128];
volatile bool system_active = false;
volatile bool door_open = false;
volatile bool alarm_active = false;
volatile int counter = 0;

/* Semaphore */
SemaphoreHandle_t xSemaphoreDoor;
SemaphoreHandle_t xSemaphoreSystem;

/************************************************************************/
/* Initiate Single Slope PWM                                            */
/************************************************************************/
void PWM_Init(void)
{
	/* Set output for Buzzer */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = PIN1_bm; //(PIN2_bm) | (PIN0_bm);
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
	// Turn off the buzzer
	TCC0.CCA = 0;  
	
	// Turn off LED
	LED_Off(LED0);
	LED_Off(LED1);
	
	// Reset counter
	counter = 0;  
}


/************************************************************************/
/* Checks the status of the door sensor                                 */
/************************************************************************/
void check_door_sensor(void)
{
	// Read sensor state from PORTE PIN0
	bool is_closed = PORTE.IN & PIN0_bm;
	
	// Update door status
	if (is_closed) {
		door_open = false;
	}
	else{
		door_open = true;
	}
}

/************************************************************************/
/* Task: Push Button Handling                                           */
/************************************************************************/
static portTASK_FUNCTION(vPushButton, pvParameters) {
    while (1) {
        if (!(PORTF.IN & PIN1_bm)) {  // SW1 toggle system state
            system_active = !system_active;
            if (!system_active) {
                reset_actuators();
            }
            xSemaphoreGive(xSemaphoreSystem);  // Signal status change
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        if (!(PORTF.IN & PIN2_bm)) {  // SW2 reset actuators
            alarm_active = false;
            reset_actuators();
            xSemaphoreGive(xSemaphoreSystem);  // Signal system reset
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Task: Check Door Status                                              */
/************************************************************************/
static portTASK_FUNCTION(vCheckDoor, pvParameters) {
    while (1) {
        if (system_active) {
            bool current_status = is_door_open();
            if (door_open != current_status) {
                door_open = current_status;
                if (!door_open) {
                    // Door opened, send signal to alarm task
                    xSemaphoreGive(xSemaphoreDoor);
                }
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Task: Alarm Control                                                  */
/************************************************************************/
static portTASK_FUNCTION(vAlarmControl, pvParameters) {
    PWM_Init();
    while (1) {
        // Wait for signal from door task
        if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
            alarm_active = true;
            counter = 0;

            while (alarm_active && system_active) {
                if (counter < 10) {
                    TCC0.CCA = 800;
                    LED_On(LED0);
                    LED_On(LED1);
                } else {
                    TCC0.CCA = 1000;
                    LED_Toggle(LED0);
                    LED_Toggle(LED1);
                }
                counter++;
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            reset_actuators();  // Reset actuators after alarm
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/* Task: Display Status on LCD */
static portTASK_FUNCTION(vDisplayStatus, pvParameters) {
    gfx_mono_init();
    gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

    while (1) {
        if (xSemaphoreTake(xSemaphoreSystem, portMAX_DELAY) == pdTRUE) {
            gfx_mono_draw_string("Sistem: ", 0, 8, &sysfont);
            gfx_mono_draw_string(system_active ? "Aktif" : "Nonaktif", 50, 8, &sysfont);

            snprintf(strbuf, sizeof(strbuf), "Pintu: %s", door_open ? "Tutup" : "Buka");
            gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

            snprintf(strbuf, sizeof(strbuf), "Counter: %02d", counter);
            gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/* Task: Display Status on LCD */
static portTASK_FUNCTION(vDisplayStatus, pvParameters) {
    gfx_mono_init();
    gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

    while (1) {
        snprintf(strbuf, sizeof(strbuf), "Sistem: %s", system_active ? "Aktif" : "Nonaktif");
        gfx_mono_draw_string(strbuf, 0, 8, &sysfont);

        snprintf(strbuf, sizeof(strbuf), "Pintu: %s", door_open ? "Tutup" : "Buka");
        gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

        snprintf(strbuf, sizeof(strbuf), "Counter: %02d", counter);
        gfx_mono_draw_string(strbuf, 0, 24, &sysfont);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


/* Main Function */
int main(void) {
    board_init();
    PMIC.CTRL |= PMIC_LOLVLEN_bm;
    cpu_irq_enable();

    // Initialize Semaphores
    xSemaphoreDoor = xSemaphoreCreateBinary();
    xSemaphoreSystem = xSemaphoreCreateBinary();

    // Initialize button pins
    PORTF.DIRCLR = PIN1_bm | PIN2_bm;
    PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;
    PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;

    // Initialize door sensor pin
    PORTE.DIRCLR = PIN0_bm;
    PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;

    // Create Tasks from highest to lowest priority
    xTaskCreate(vPushButton, "PushButton", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vCheckDoor, "CheckDoor", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vDisplayStatus, "Display", 1000, NULL, tskIDLE_PRIORITY, NULL);

    // Start Scheduler
    vTaskStartScheduler();
}
