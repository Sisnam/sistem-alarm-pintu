/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>

// Initiate variables
static char strbuf[128];
volatile bool system_active = false;
volatile bool door_open = false;
volatile bool alarm_active = false;
volatile int counter = 0;

// Initiate function prototypes
void PWM_Init(void);
void init_interrupts(void);
void set_on_led(void);
void set_flicker_led(void);
void init_door_sensor(void);
void reset_actuators(void);
void check_door_sensor(void);

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
/* ISR for handling button interrupts from SW1 and SW2                  
   SW1 toggles the system state (active/inactive)
   SW2 deactivates the actuators if the system is active
*/
/************************************************************************/
ISR(PORTF_INT0_vect)
{
	// Handle SW1 press to toggle system state
	if (!(PORTF.IN & PIN1_bm)) {
		system_active = !system_active;
		
		// Reset actuators if the system is inactive
		if (!system_active) {
			reset_actuators(); 
			
			// Ensure alarm is off 
			alarm_active = false;  
			
			// Update LCD display status
			gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont); 
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			delay_ms(100);
		}
		delay_ms(100);
	}
	
	// Handle SW2 press to deactivate actuators if system is active
	if (!(PORTF.IN & PIN2_bm)) {
		if (system_active) {
			alarm_active = false;
			reset_actuators();
		}
		delay_ms(100);
	}
}

/************************************************************************/
/* Initializes external interrupts for SW1 and SW2                      */
/************************************************************************/
void init_interrupts(void)
{
	// Set PIN1 dan PIN2 as input
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;

	// Configure falling edge detection for both switches
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;  // SW1
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;  // SW2

	// Enable interrupts for both switches
	PORTF.INT0MASK = PIN1_bm | PIN2_bm;  // Mask interrupt for PIN1 and PIN2
	PORTF.INTCTRL = PORT_INT0LVL_LO_gc;  // Set interrupt level to low

	// Enable low-level interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// Enable global interrupts
	cpu_irq_enable();
}

/************************************************************************/
/* Initializes the magnetic door sensor on PORTE PIN0                   */
/************************************************************************/
void init_door_sensor(void)
{
	PORTE.DIRCLR = PIN0_bm;
	
	// Enable pull-up resistor
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

/************************************************************************/
/* Turns on both LEDs (LED0 and LED1)                                   */
/************************************************************************/
void set_on_led(void)
{
	LED_On(LED0);
	LED_On(LED1);
}

/************************************************************************/
/* Makes the LEDs blink alternately.                                    */
/************************************************************************/
void set_flicker_led(void)
{
	LED_On(LED0);
	LED_Off(LED1);

	delay_ms(100);

	LED_Off(LED0);
	LED_On(LED1);
	
	delay_ms(100);
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
/* Main function                                                        */
/************************************************************************/
int main(void)
{
	// Initialize  board
	board_init();

	// Initialize  PWM
	PWM_Init();

	// Initialize  Interrupt
	init_interrupts();

	// Initialize  LCD
	gfx_mono_init();

	// Enable background lamp LCD
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	gfx_mono_draw_string("Sisnam+", 0, 0, &sysfont);
	
	// Initialize door sensor
	init_door_sensor();
	
	// Display initial system state
	gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont);
	snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
	gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
	
	// Main loop
	while (true)
	{
		// Check door status
		check_door_sensor();
		snprintf(strbuf, sizeof(strbuf), "Status Pintu: %s", door_open ? "Closed" : "Opened");
		gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
		
		// Check if system is active
		if (system_active)
		{
			// Update system status on LCD display
			gfx_mono_draw_string("Sistem Aktif   ", 0, 8, &sysfont);
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			
			// Check if door is opened
			if (!door_open){
				alarm_active = true;				
			}
			
			// Check if alarm is active
			if (alarm_active){
				
				// Set low buzzer and LED intensity below 10 seconds
				if (counter < 10)
				{
					TCC0.CCA = 800;
					set_on_led();
				}
				// Set high buzzer and LED intensity above 10 seconds
				else
				{
					TCC0.CCA = 1000;
					set_flicker_led();
				}

				// Increment counter each loop
				counter++;
				delay_ms(100);
			}
		}
		// Reset actuators if system is inactive
		else{
			reset_actuators();
		}

	}
}