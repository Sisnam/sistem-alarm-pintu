#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

/* Task Definitions */
static portTASK_FUNCTION_PROTO(vCheckDoor, pvParameters);
static portTASK_FUNCTION_PROTO(vAlarmControl, pvParameters);
static portTASK_FUNCTION_PROTO(vServoControl, pvParameters);
static portTASK_FUNCTION_PROTO(vDisplayStatus, pvParameters);
static portTASK_FUNCTION_PROTO(vPushButton, pvParameters);
static portTASK_FUNCTION_PROTO(vUARTTask, pvParameters);

/* Global Variables */
static char strbuf[128];
volatile bool system_active = false;
volatile bool door_open = false;
volatile bool alarm_active = false;
volatile int counter = 0;
volatile bool access_granted = false;

/* Semaphore */
SemaphoreHandle_t xSemaphoreDoor;
SemaphoreHandle_t xSemaphoreSystem;

/************************************************************************/
/* UART Configuration                                                   */
/************************************************************************/
void setUpSerial() {
    USARTC0_BAUDCTRLB = 0;      // BSCALE = 0
    USARTC0_BAUDCTRLA = 0x0C;   // Baudrate ~9600 @ 2MHz
    USARTC0_CTRLA = 0;          // Disable interrupts
    USARTC0_CTRLC = USART_CHSIZE_8BIT_gc; // 8-bit, no parity
    USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm; // Enable TX & RX
}

void sendChar(char c) {
    while (!(USARTC0_STATUS & USART_DREIF_bm)); // Wait until DATA buffer is empty
    delay_ms(20); 
    USARTC0_DATA = c;
}

void sendString(char *text) {
    while (*text) {
        sendChar(*text++);
    }
}

char receiveChar() {
    while (!(USARTC0_STATUS & USART_RXCIF_bm)); // Wait until receive finish
    delay_ms(20); 
    return USARTC0_DATA;
}

void receiveString(char *reads, int maxSize) {
    int i = 0;
    while (1) {
        char inp = receiveChar();
        if (inp == '\n' || i >= maxSize - 1) {
            reads[i] = '\0'; // Null terminate the string
            break;
        } else {
            reads[i++] = inp;
        }
    }
}

/************************************************************************/
/* PWM Initialization for Servo and Buzzer                             */
/************************************************************************/
void PWM_Init(void)
{
    /* PWM for Servo - TCC0 Channel A */
    PORTC.DIR |= PIN0_bm; // Set PORTC PIN0 as output for Servo
    TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
    TCC0.CTRLB = TC_WGMODE_SINGLESLOPE_gc | TC0_CCAEN_bm;
    TCC0.PER = 5000; // 20ms period (50Hz)
    TCC0.CCA = 75;  // Initial duty cycle for 0 degrees

    /* PWM for Buzzer - TCC1 Channel A */
    PORTD.DIR |= PIN0_bm; // Set PORTD PIN0 as output for Buzzer
    TCD0.CTRLA = TC_CLKSEL_DIV8_gc;
    TCD0.CTRLB = TC_WGMODE_SINGLESLOPE_gc | TC0_CCAEN_bm;
    TCD0.PER = 1000; // Set period for Buzzer frequency
    TCD0.CCA = 0;    // Start with buzzer off
}

/************************************************************************/
/* Resets all actuators (buzzer & LED) and counter                      */
/************************************************************************/
void reset_actuators(void)
{
    // Turn off the buzzer
    TCD0.CCA = 0;
    // Reset servo to 0 degrees
    TCC0.CCA = 75;
    // Turn off LED
    LED_Off(LED0);
    LED_Off(LED1);
    // Reset counter
    counter = 0;
}

/************************************************************************/
/* Function to Check Door Sensor                                        */
/************************************************************************/
void check_door_sensor(void)
{
    bool is_closed = PORTE.IN & PIN0_bm;
    door_open = !is_closed;
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
            check_door_sensor(); // Update door_open status
            if (!door_open && !access_granted) {
                xSemaphoreGive(xSemaphoreDoor); // Trigger alarm
            }
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Servo Motor Control Task                                             */
/************************************************************************/
static portTASK_FUNCTION(vServoControl, pvParameters) {
    while (1) {
        if (access_granted) {
            // Move servo to unlock position (180 degrees)
            TCC0.CCA = 325;
            vTaskDelay(3000 / portTICK_PERIOD_MS); // Hold position for 3 seconds

            // Reset servo to lock position (0 degrees)
            TCC0.CCA = 75;
            access_granted = false;
            counter = 0; // Confirm counter reset after servo action
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Task: Alarm Control                                                  */
/************************************************************************/
static portTASK_FUNCTION(vAlarmControl, pvParameters) {
    while (1) {
        if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
            alarm_active = true;
            counter = 0;

            while (alarm_active && system_active) {
                if (counter < 10) {
                    TCD0.CCA = 500; // Low buzzer frequency
                    LED_On(LED0);
                    LED_On(LED1);
                } else {
                    TCD0.CCA = 800; // High buzzer frequency
                    LED_Toggle(LED0);
                    LED_Toggle(LED1);
                }
                counter++;
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            reset_actuators();
            counter = 0; // Confirm counter reset after alarm deactivation
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Task: Display Status on LCD                                          */
/************************************************************************/
static portTASK_FUNCTION(vDisplayStatus, pvParameters) {
    gfx_mono_init();
    gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

    while (1) {
        if (xSemaphoreTake(xSemaphoreSystem, portMAX_DELAY) == pdTRUE) {
            gfx_mono_draw_string("Sistem: ", 0, 8, &sysfont);
            gfx_mono_draw_string(system_active ? "Aktif" : "Nonaktif", 50, 8, &sysfont);

            snprintf(strbuf, sizeof(strbuf), "Pintu: %s", door_open ? "Buka" : "Tutup");
            gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

            snprintf(strbuf, sizeof(strbuf), "Counter: %02d", counter);
            gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* UART Task: Receive RFID Logs and Status                              */
/************************************************************************/
static portTASK_FUNCTION(vUARTTask, pvParameters) {
    char logBuffer[128];
    while (1) {
        memset(logBuffer, 0, sizeof(logBuffer));
        receiveString(logBuffer, sizeof(logBuffer));

        // Parse the log message
        if (strstr(logBuffer, "ACCESS: TRUE")) {
            access_granted = true;
            alarm_active = false;
            reset_actuators();
        } else if (strstr(logBuffer, "ACCESS: FALSE")) {
            access_granted = false;
        }

        // Optional: Display the log on LCD
        gfx_mono_draw_string(logBuffer, 0, 32, &sysfont);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/************************************************************************/
/* Main Function                                                        */
/************************************************************************/
int main(void) {
    board_init();
    PMIC.CTRL |= PMIC_LOLVLEN_bm;
    cpu_irq_enable();

    // Initialize Semaphores
    xSemaphoreDoor = xSemaphoreCreateBinary();
    xSemaphoreSystem = xSemaphoreCreateBinary();

    // Initialize UART
    setUpSerial();

    // Initialize PWM for Servo and Buzzer
    PWM_Init();

    // Initialize button pins
    PORTF.DIRCLR = PIN1_bm | PIN2_bm;
    PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;
    PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;

    // Initialize door sensor pin
    PORTE.DIRCLR = PIN0_bm;
    PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;

    // Create Tasks
    xTaskCreate(vPushButton, "PushButton", 1000, NULL, tskIDLE_PRIORITY + 4, NULL);
    xTaskCreate(vCheckDoor, "CheckDoor", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(vAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vServoControl, "ServoControl", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(vUARTTask, "UARTTask", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vDisplayStatus, "Display", 1000, NULL, tskIDLE_PRIORITY, NULL);

    // Start Scheduler
    vTaskStartScheduler();
}