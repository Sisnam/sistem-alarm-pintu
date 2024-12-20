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
static portTASK_FUNCTION_PROTO(vServoControl, pvParameters);
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
void setUpSerial()
{
	// Baud rate selection
	// BSEL = (2000000 / (2^0 * 16*9600) -1 = 12.0208... ~ 12 -> BSCALE = 0
	// FBAUD = ( (2000000)/(2^0*16(12+1)) = 9615.384 -> mendekati lah ya
	
	USARTC0_BAUDCTRLB = 0; //memastikan BSCALE = 0
	USARTC0_BAUDCTRLA = 0x0C; // 12
	
	//USARTC0_BAUDCTRLB = 0; //Just to be sure that BSCALE is 0
	//USARTC0_BAUDCTRLA = 0xCF; // 207
	
	//Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	//8 data bits, no parity and 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	
	//Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
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
	gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont); 
	while (1) {		
		if (!(PORTF.IN & PIN1_bm)) {  // SW1 toggle system state
			system_active = !system_active;
			
			gfx_mono_draw_string("Sistem Aktif", 0, 8, &sysfont); 
			
			if (!door_open){
				alarm_active = true;
			}
			
			if (!system_active) {
				gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont); 
				
				reset_actuators();
				
				// Ensure alarm is off
				alarm_active = false;
			}
				
			// xSemaphoreGive(xSemaphoreSystem);  // Signal status change
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
		if (!(PORTF.IN & PIN2_bm)) {  // SW2 reset actuators
			if (system_active) {
				alarm_active = false;
				reset_actuators();
			}
			
			// xSemaphoreGive(xSemaphoreSystem);  // Signal system reset
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}	
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/************************************************************************/
/* Task: Check Door Status (DONE)                                       */
/************************************************************************/
static portTASK_FUNCTION(vCheckDoor, pvParameters) {
	// Initialize door sensor pin
	PORTE.DIRCLR = PIN0_bm;
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
	gfx_mono_draw_string("Pintu Tertutup", 0, 16, &sysfont); 
	while (1) {
		
		bool is_open = PORTE.IN & PIN0_bm;
		
		// Update door status
		if (!is_open) {
			door_open = false;
			gfx_mono_draw_string("Pintu Tertutup", 0, 16, &sysfont); 
		}
		else{
			door_open = true;
			gfx_mono_draw_string("Pintu Terbuka", 0, 16, &sysfont); 
		}
	
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}

/************************************************************************/
/* Task: Alarm Control                                                  */
/************************************************************************/
static portTASK_FUNCTION(vAlarmControl, pvParameters) {
	gfx_mono_draw_string("Waktu: 0", 0, 24, &sysfont);
	PWM_Init();
	while (1) {
		//if (xSemaphoreTake(xSemaphoreDoor, portMAX_DELAY) == pdTRUE) {
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
				
				snprintf(strbuf, sizeof(strbuf), "Waktu: %02d", counter);
				gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
				
				vTaskDelay(100 / portTICK_PERIOD_MS);
			}
			reset_actuators();
			counter = 0; // Confirm counter reset after alarm deactivation
			gfx_mono_draw_string("Waktu: 0", 0, 24, &sysfont);

		//}
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

		vTaskDelay(100 / portTICK_PERIOD_MS);
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
	xSemaphoreSystem = xSemaphoreCreateBinary();
	
	PORTC_OUTSET = PIN3_bm; // PC3 as TX
	PORTC_DIRSET = PIN3_bm; //TX pin as output
	    
	PORTC_OUTCLR = PIN2_bm; //PC2 as RX
	PORTC_DIRCLR = PIN2_bm; //RX pin as input
	
	// Initialize UART
	setUpSerial();
	
	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
		.stopbits = USART_SERIAL_STOP_BIT
	};
	
	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);
	

	// Initialize button pins
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;


	// Create Tasks
	xTaskCreate(vPushButton, "PushButton", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vCheckDoor, "CheckDoor", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
//	xTaskCreate(vServoControl, "ServoControl", 1000, NULL, tskIDLE_PRIORITY, NULL);
//	xTaskCreate(vUARTTask, "UARTTask", 1000, NULL, tskIDLE_PRIORITY + 4, NULL);

	// Start Scheduler
	vTaskStartScheduler();
}