#include <asf.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS/include/FreeRTOS.h"
#include "FreeRTOS/include/queue.h"
#include "FreeRTOS/include/task.h"
#include "FreeRTOS/include/timers.h"
#include "FreeRTOS/include/semphr.h"

/* USART Arduino */
#define USART_SERIAL_EXAMPLE &USARTC0
#define USART_SERIAL_EXAMPLE_BAUDRATE 9600
#define USART_SERIAL_CHAR_LENGTH USART_CHSIZE_8BIT_gc
#define USART_SERIAL_PARITY USART_PMODE_DISABLED_gc
#define USART_SERIAL_STOP_BIT false
#define BUFFER_SIZE 10

/* USART Dashboard */
#define USART_DASHBOARD &USARTE0
#define USART_DASHBOARD_BAUDRATE 9600
#define USART_DASHBOARD_CHAR_LENGTH USART_CHSIZE_8BIT_gc
#define USART_DASHBOARD_PARITY USART_PMODE_DISABLED_gc
#define USART_DASHBOARD_STOP_BIT false
#define DASHBOARD_BUFFER_SIZE 10

/* Task Definitions */
static portTASK_FUNCTION_PROTO(vCheckDoor, pvParameters);
static portTASK_FUNCTION_PROTO(vAlarmControl, pvParameters);
static portTASK_FUNCTION_PROTO(vPushButton, pvParameters);
static portTASK_FUNCTION_PROTO(vListenUART, pvParameters);
static portTASK_FUNCTION_PROTO(vAccessStatusControl, pvParameters);
static portTASK_FUNCTION_PROTO(vReceiveSystemCommand, pvParameters);
static portTASK_FUNCTION_PROTO(vSendSystemStatus, pvParameters);


/* Function Prototype */
void setUpSerial(void);
void PWM_Init(void);
void reset_actuators(void);
void reset_lcd(void);
char receiveChar(void);
int receiveString(char *, int);
void sendChar(char);

void setUpSerialDashboard(void);
char receiveCharE(void);
int receiveStringE(char *, int);
void sendCharE(char);
void sendStringE(char *);

/* Global Variables */
static char strbuf[128];
bool system_active = false;	  // Removed volatile as semaphores handle synchronization
bool alarm_active = false;	  // Removed volatile for the same reason
bool system_deactive = false; // Trigger resend door status to queue
bool access_granted = false;
int counter = 0;

/* Semaphore and Mutex */
SemaphoreHandle_t xMutexSystemActive;
SemaphoreHandle_t xMutexSystemDeactive;
SemaphoreHandle_t xMutexAccessControl;

/* Door queue */
QueueHandle_t xQueueSemaphoreDoor;
QueueHandle_t xQueueAccessStatus;

// Structure for our data
typedef struct
{
	bool door_status;
} DataPacket_t;

typedef struct
{
	char buffer[BUFFER_SIZE];
} DataPacketStatusAccess_t;

/************************************************************************/
/* UART Configuration for Arduino                                        */
/************************************************************************/
void setUpSerial()
{
	USARTC0_BAUDCTRLB = 0;	  // Ensure BSCALE is 0
	USARTC0_BAUDCTRLA = 0x0C; // Set baud rate

	// Disable interrupts, just for safety
	USARTC0_CTRLA = 0;
	// 8 data bits, no parity, 1 stop bit
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;
	// Enable receive and transmit
	USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;
}

/* Send char to Arduino */
void sendChar(char c)
{
	while (!(USARTC0_STATUS & USART_DREIF_bm))
	; // Wait until DATA buffer is empty
	delay_ms(10);
	USARTC0_DATA = c;
}

/* Receive char from Arduino */
char receiveChar()
{
	while (!(USARTC0_STATUS & USART_RXCIF_bm))
	; // Wait until receive finish

	return USARTC0_DATA;
}

int receiveString(char *buffer, int size)
{
	char c;
	int index = 0;

	for (int i = 0; i < size; i++)
	{
		buffer[i] = '\0';
	}

	while (index < size - 1)
	{ // Leave space for null terminator
		c = receiveChar();

		// Check for newline or carriage return
		if (c == '\n' || c == '\r')
		{
			break;
		}

		buffer[index++] = c;
	}

	buffer[index] = '\0'; // Null terminate
	return index;		  // Return the number of characters read
}


void setUpSerialDashboard()
{
	// Configure USARTE0 pins first
	PORTE_OUTSET = PIN3_bm; // PE3 as TX
	PORTE_DIRSET = PIN3_bm; // TX pin as output
	
	PORTE_OUTCLR = PIN2_bm; // PE2 as RX
	PORTE_DIRCLR = PIN2_bm; // RX pin as input
	
	// Configure USARTE0
	USARTE0_BAUDCTRLB = 0;
	USARTE0_BAUDCTRLA = 0x0C;  // For 9600 baud rate
	
	USARTE0_CTRLC = USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;
	USARTE0_CTRLB = USART_RXEN_bm | USART_TXEN_bm;  // Enable RX and TX
}

/* Send char to Dashboard */
void sendCharE(char c)
{
	while (!(USARTE0_STATUS & USART_DREIF_bm))
	; // Wait until DATA buffer is empty
	delay_ms(10);
	USARTE0_DATA = c;
}

void sendStringE(char *text)
{
	while(*text)
	{
		sendCharE(*text);
		text++;
	}
}

/* Receive char from Dashboard */
char receiveCharE()
{
	while (!(USARTE0_STATUS & USART_RXCIF_bm))
	; // Wait until receive finish

	return USARTE0_DATA;
}

int receiveStringE(char *buffer, int size)
{
	char c;
	int index = 0;

	for (int i = 0; i < size; i++)
	{
		buffer[i] = '\0';
	}

	while (index < size - 1)
	{ // Leave space for null terminator
		c = receiveCharE();

		// Check for newline or carriage return
		if (c == '\n' || c == '\r')
		{
			break;
		}

		buffer[index++] = c;
	}

	buffer[index] = '\0'; // Null terminate
	return index;		  // Return the number of characters read
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
/* Reset LCD                                                            */
/************************************************************************/
void reset_lcd(void)
{
	gfx_mono_draw_string("Sisnam+          ", 0, 0, &sysfont);
	gfx_mono_draw_string("Sistem: Nonaktif ", 0, 8, &sysfont);
	gfx_mono_draw_string("Pintu: -         ", 0, 16, &sysfont);
	gfx_mono_draw_string("Waktu: -         ", 0, 24, &sysfont);
}

/************************************************************************/
/* Task: Push Button Handling                                           */
/************************************************************************/
static portTASK_FUNCTION(vPushButton, pvParameters)
{
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;

	gfx_mono_draw_string("Sistem: Nonaktif", 0, 8, &sysfont);

	while (1)
	{
		if (!(PORTF.IN & PIN1_bm))
		{ // SW1 toggle system state
			if (xSemaphoreTake(xMutexSystemActive, portMAX_DELAY) == pdTRUE)
			{
				if (xSemaphoreTake(xMutexSystemDeactive, pdMS_TO_TICKS(250)) == pdTRUE)
				{
					system_active = !system_active;
					system_deactive = true;
					gfx_mono_draw_string(system_active ? "Sistem: Aktif    " : "Sistem: Nonaktif     ", 0, 8, &sysfont);

					if (!system_active)
					{
						reset_actuators();
						reset_lcd();
						alarm_active = false;
					}
					else
					{
						gfx_mono_draw_string("Alarm: Nonaktif", 0, 0, &sysfont);
					}
					xSemaphoreGive(xMutexSystemDeactive);
				}

				xSemaphoreGive(xMutexSystemActive);
			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}
		else if (!(PORTF.IN & PIN2_bm))
		{ // SW2 reset actuators
			if (xSemaphoreTake(xMutexSystemActive, portMAX_DELAY) == pdTRUE)
			{
				if (xSemaphoreTake(xMutexSystemDeactive, portMAX_DELAY) == pdTRUE)
				{
					if (system_active)
					{

						system_deactive = true;
						alarm_active = false;
						reset_actuators();
						gfx_mono_draw_string("Waktu: -         ", 0, 24, &sysfont);
					}

					xSemaphoreGive(xMutexSystemDeactive);
				}

				xSemaphoreGive(xMutexSystemActive);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Task: Check Door Status                                              */
/************************************************************************/
static portTASK_FUNCTION(vCheckDoor, pvParameters)
{
	PORTE.DIRCLR = PIN0_bm;
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;

	gfx_mono_draw_string("Pintu: -         ", 0, 16, &sysfont);

	DataPacket_t dataPacket;
	bool prev_door_status = false;

	while (1)
	{
		bool door_status = PORTE.IN & PIN0_bm;

		if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(250)) == pdTRUE)
		{
			if (system_active)
			{
				if (xSemaphoreTake(xMutexSystemDeactive, pdMS_TO_TICKS(250)) == pdTRUE)
				{
					if (door_status != prev_door_status || system_deactive)
					{
						// Reset access granted
						if (xSemaphoreTake(xMutexAccessControl, pdMS_TO_TICKS(250)) == pdTRUE)
						{
							if (access_granted && prev_door_status && !door_status)
							{
								access_granted = false;
								gfx_mono_draw_string("Waktu: -            ", 0, 24, &sysfont);
							}

							xSemaphoreGive(xMutexAccessControl);
						}

						prev_door_status = door_status;
						system_deactive = false;

						snprintf(strbuf, sizeof(strbuf), "Pintu: %s", door_status ? "Terbuka  " : "Tertutup ");
						gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

						// Send door status to queue
						dataPacket.door_status = door_status;
						if (xQueueSend(xQueueSemaphoreDoor, &dataPacket, pdMS_TO_TICKS(100)) == pdPASS)
						{
							printf("Sent: Door status=%d\n", dataPacket.door_status);
						}
					}
					xSemaphoreGive(xMutexSystemDeactive);
				}
			}

			xSemaphoreGive(xMutexSystemActive);
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Task: Alarm Control                                                  */
/************************************************************************/
static portTASK_FUNCTION(vAlarmControl, pvParameters)
{
	gfx_mono_draw_string("Waktu: -            ", 0, 24, &sysfont);
	PWM_Init();

	DataPacket_t receivedPacket;

	while (1)
	{
		if (xQueueReceive(xQueueSemaphoreDoor, &receivedPacket, pdMS_TO_TICKS(100)) == pdPASS)
		{
			if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(250)) == pdTRUE)
			{
				if (xSemaphoreTake(xMutexAccessControl, pdMS_TO_TICKS(250)) == pdTRUE)
				{
					if (system_active && receivedPacket.door_status && !access_granted)
					{
						gfx_mono_draw_string("Alarm: Aktif   ", 0, 0, &sysfont);
						alarm_active = true;
					}
					else if (alarm_active)
					{
						gfx_mono_draw_string("Alarm: Aktif   ", 0, 0, &sysfont);
					}
					else
					{
						gfx_mono_draw_string("Alarm: Nonaktif", 0, 0, &sysfont);
						alarm_active = false;
					}

					xSemaphoreGive(xMutexAccessControl);
				}

				xSemaphoreGive(xMutexSystemActive);
			}
		}

		if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(250)) == pdTRUE)
		{
			if (xSemaphoreTake(xMutexAccessControl, pdMS_TO_TICKS(250)) == pdTRUE)
			{
				if (alarm_active && !access_granted)
				{
					if (counter < 10)
					{
						TCC0.CCA = 500; // Low buzzer frequency
						LED_On(LED0);
						LED_On(LED1);
					}
					else
					{
						TCC0.CCA = 800; // High buzzer frequency
						LED_Toggle(LED0);
						LED_Toggle(LED1);
					}
					counter++;
					snprintf(strbuf, sizeof(strbuf), "Waktu: %02d           ", counter);
					gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
				}

				xSemaphoreGive(xMutexAccessControl);
			}

			xSemaphoreGive(xMutexSystemActive);
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Receive RFID Access Status                                           */
/************************************************************************/
static portTASK_FUNCTION(vListenUART, pvParameters)
{
	PORTC_OUTSET = PIN3_bm; // PC3 as TX
	PORTC_DIRSET = PIN3_bm; // TX pin as output

	PORTC_OUTCLR = PIN2_bm; // PC2 as RX
	PORTC_DIRCLR = PIN2_bm; // RX pin as input

	setUpSerial();

	static usart_rs232_options_t USART_SERIAL_OPTIONS = {
		.baudrate = USART_SERIAL_EXAMPLE_BAUDRATE,
		.charlength = USART_SERIAL_CHAR_LENGTH,
		.paritytype = USART_SERIAL_PARITY,
	.stopbits = USART_SERIAL_STOP_BIT};

	usart_init_rs232(USART_SERIAL_EXAMPLE, &USART_SERIAL_OPTIONS);

	// Store received access status
	char buffer[BUFFER_SIZE];

	DataPacketStatusAccess_t dataPacket;

	while (1)
	{
		receiveString(buffer, BUFFER_SIZE);

		if (strcmp(buffer, "TRUE") == 0 || strcmp(buffer, "FALSE") == 0)
		{
			// Send access status
			strcpy(dataPacket.buffer, buffer);

			if (xQueueSend(xQueueAccessStatus, &dataPacket, pdMS_TO_TICKS(100)) == pdPASS)
			{
				printf("Sent: Access Status");
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Control Access Status                                                */
/************************************************************************/
static portTASK_FUNCTION(vAccessStatusControl, pvParameters)
{
	DataPacketStatusAccess_t receivedPacket;

	while (1)
	{
		if (xQueueReceive(xQueueAccessStatus, &receivedPacket, pdMS_TO_TICKS(100)) == pdPASS)
		{
			if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(250)) == pdTRUE)
			{
				if (system_active)
				{
					if (strcmp(receivedPacket.buffer, "TRUE") == 0)
					{
						gfx_mono_draw_string("Akses: Diberikan ", 0, 24, &sysfont);

						if (xSemaphoreTake(xMutexAccessControl, pdMS_TO_TICKS(250)) == pdTRUE)
						{
							access_granted = true;
							xSemaphoreGive(xMutexAccessControl);
						}
					}
					else if (strcmp(receivedPacket.buffer, "FALSE") == 0)
					{
						gfx_mono_draw_string("Akses: Ditolak   ", 0, 24, &sysfont);

						if (xSemaphoreTake(xMutexAccessControl, pdMS_TO_TICKS(250)) == pdTRUE)
						{
							access_granted = false;
							xSemaphoreGive(xMutexAccessControl);

							delay_ms(200);

							gfx_mono_draw_string("Waktu: -            ", 0, 24, &sysfont);
						}
					}
				}

				xSemaphoreGive(xMutexSystemActive);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

/************************************************************************/
/* Receive Dashboard Arduino Communication                              */
/************************************************************************/
static portTASK_FUNCTION(vReceiveSystemCommand, pvParameters)
{
	char buffer[DASHBOARD_BUFFER_SIZE];
	
	// Initialize USART E
	setUpSerialDashboard();

	while (1)
	{
		if (USARTE0.STATUS & USART_RXCIF_bm)  // Check if data is available
		{
			int len = receiveStringE(buffer, DASHBOARD_BUFFER_SIZE);
			if (len > 0){
				// Take mutex before modifying system state
				if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(100)) == pdTRUE)
				{
					if (strcmp(buffer, "S:1") == 0){
						system_active = true;
						gfx_mono_draw_string("Sistem: Aktif    ", 0, 8, &sysfont);
					}
					else if (strcmp(buffer, "S:0") == 0){
						if (xSemaphoreTake(xMutexSystemDeactive, pdMS_TO_TICKS(100)) == pdTRUE){
							system_active = false;
							system_deactive = true;
							alarm_active = false;
							reset_actuators();
							reset_lcd();
							xSemaphoreGive(xMutexSystemDeactive);
						}
					}
					xSemaphoreGive(xMutexSystemActive);
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

static portTASK_FUNCTION(vSendSystemStatus, pvParameters)
{
	 // Increased buffer size for safety
	char status_buffer[32];
	
	// Initialize USART E
	setUpSerialDashboard();

	while (1)
	{
		if (xSemaphoreTake(xMutexSystemActive, pdMS_TO_TICKS(100)) == pdTRUE)
		{
			// Prepare status string
			snprintf(status_buffer, sizeof(status_buffer), "S:%c|A:%c|P:%c\n",
				system_active ? '1' : '0',
				alarm_active ? '1' : '0',
				(PORTE.IN & PIN0_bm) ? '1' : '0');
			
			char *ptr = status_buffer;
			while (*ptr)
			{
				sendCharE(*ptr++);
			}
			xSemaphoreGive(xMutexSystemActive);
		}

		vTaskDelay(pdMS_TO_TICKS(1000)); // Kirim status setiap 1 detik
	}
}


/************************************************************************/
/* Main Function                                                        */
/************************************************************************/
int main(void)
{
	board_init();
	sysclk_init();
	pmic_init();
	gfx_mono_init();

	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);
	gfx_mono_draw_string("Sisnam+", 0, 0, &sysfont);

	// Initialize Semaphores
	xMutexSystemActive = xSemaphoreCreateMutex();
	xMutexSystemDeactive = xSemaphoreCreateMutex();
	xMutexAccessControl = xSemaphoreCreateMutex();

	// Initialize Queue
	xQueueSemaphoreDoor = xQueueCreate(10, sizeof(DataPacket_t));
	xQueueAccessStatus = xQueueCreate(10, sizeof(DataPacketStatusAccess_t));

	// Create Tasks
	xTaskCreate(vPushButton, "PushButton", 1000, NULL, tskIDLE_PRIORITY + 3, NULL);
	xTaskCreate(vCheckDoor, "CheckDoor", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vAlarmControl, "AlarmControl", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vListenUART, "Receive Access Status", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(vAccessStatusControl, "Control Access Status", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vReceiveSystemCommand, "Receive System Command", 1000, NULL, tskIDLE_PRIORITY + 2, NULL);
	xTaskCreate(vSendSystemStatus, "Send System Status", 1000, NULL, tskIDLE_PRIORITY + 1, NULL);

	// Start Scheduler
	vTaskStartScheduler();
}
