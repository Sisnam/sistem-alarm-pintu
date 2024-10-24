#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>

static char strbuf[128];
volatile bool system_active = false;
volatile bool door_open = false;
volatile bool alarm_active = false;
volatile int counter = 0;

// Function prototypes
void PWM_Init(void);
void init_interrupts(void);
void set_on_led(void);
void set_flicker_led(void);
void init_door_sensor(void);
void reset_actuators(void);
void check_door_sensor(void);

// Inisiliasi Single Slope PWM
void PWM_Init(void)
{
	/* Set output */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = PIN1_bm; //(PIN2_bm) | (PIN0_bm);
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);

	/* Set Period */
	TCC0.PER = 1000;

	/* Set Compare Register value*/
	TCC0.CCA = 0;
}

// Reset Aktuator
void reset_actuators(void)
{
	TCC0.CCA = 0;  // Matikan buzzer
	LED_Off(LED0);
	LED_Off(LED1);
	counter = 0;  // Reset counter
}


ISR(PORTF_INT0_vect)
{
	if (!(PORTF.IN & PIN1_bm)) {
		// SW1 ditekan - Toggle status sistem
		system_active = !system_active;
		
		if (!system_active) {
			reset_actuators();  // Reset aktuator jika sistem dinonaktifkan
			alarm_active = false;  // Pastikan alarm juga dinonaktifkan
			gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont); // Update status di LCD
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			delay_ms(100);
		}
		delay_ms(100);
	}
	
	if (!(PORTF.IN & PIN2_bm)) {
		// SW2 ditekan - Matikan aktuator jika sistem aktif
		if (system_active) {
			alarm_active = false;
			reset_actuators();
		}
		delay_ms(100);
	}
}

void init_interrupts(void)
{
	// Set PIN1 dan PIN2 sebagai input
	PORTF.DIRCLR = PIN1_bm | PIN2_bm;  // Set kedua PIN sebagai input

	// Konfigurasi kontrol PIN1 (SW1) dan PIN2 (SW2)
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc;  // Detect falling edge untuk SW1
	PORTF.PIN2CTRL = PORT_ISC_FALLING_gc;  // Detect falling edge untuk SW2

	// Aktifkan interrupt mask untuk PIN1 dan PIN2
	PORTF.INT0MASK = PIN1_bm | PIN2_bm;  // Mask interrupt untuk PIN1 dan PIN2
	PORTF.INTCTRL = PORT_INT0LVL_LO_gc;  // Set interrupt level ke low

	// Aktifkan low-level interrupts
	PMIC.CTRL |= PMIC_LOLVLEN_bm;

	// Enable global interrupts
	cpu_irq_enable();
}

void init_door_sensor(void)
{
	PORTE.DIRCLR = PIN0_bm;
	PORTE.PIN0CTRL = PORT_OPC_PULLUP_gc;
}

void set_on_led(void)
{
	LED_On(LED0);
	LED_On(LED1);
}

void set_flicker_led(void)
{
	LED_On(LED0);
	LED_Off(LED1);

	delay_ms(100);

	LED_Off(LED0);
	LED_On(LED1);
}

void check_door_sensor(void)
{
	bool is_closed = PORTE.IN & PIN0_bm;
	
	if (is_closed) {
		door_open = false;
	}
	else{
		door_open = true;
	}
}



int main(void)
{
	// Inisialisasi board
	board_init();

	// Inisialisasi PWM dengan timer
	PWM_Init();

	// Inisialisasi Interrupt
	init_interrupts();

	// Inisialisasi LCD
	gfx_mono_init();

	// Menyalakan lampu background LCD
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	gfx_mono_draw_string("Sisnam+", 0, 0, &sysfont);
	
	init_door_sensor();
	
	gfx_mono_draw_string("Sistem Nonaktif", 0, 8, &sysfont); // Update status di LCD
	snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
	gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
	
	while (true)
	{
		// Mengecek status sensor pintu
		check_door_sensor();
		snprintf(strbuf, sizeof(strbuf), "Status Pintu: %s", door_open ? "Closed" : "Opened");
		gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
		
		if (system_active)
		{
			// Tampilkan status pintu dan counter di LCD
			gfx_mono_draw_string("Sistem Aktif   ", 0, 8, &sysfont);
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
			
			if (!door_open){
				alarm_active = true;				
			}
			
			if (alarm_active){
				// Kedip LED0 dan LED1 jika lebih dari 10 detik
				if (counter < 10)
				{
					TCC0.CCA = 800;
					set_on_led();
				}
				else
				{
					TCC0.CCA = 1000;
					set_flicker_led();
				}

				// Tambah counter setiap detik
				counter++;
				delay_ms(100);
			}
		}
		else{
			reset_actuators();
		}

	}
}