#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>

static char strbuf[128];
volatile bool door_open = false;
volatile bool alarm_active = false;
volatile int counter = 0;

// Function prototypes
void PWM_Init(void);
void init_interrupts(void);
void set_on_led(void);
void set_flicker_led(void);
void init_door_sensor(void);

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

// ISR untuk SW0 (PORTF PIN1)
ISR(PORTF_INT0_vect)
{
	door_open = !door_open;	  // Toggle status pintu
	alarm_active = door_open; // Set status alarm berdasarkan status pintu
	counter = 0;			  // Reset counter setiap kali status berubah
}

// Inisialisasi interrupt eksternal untuk SW0
void init_interrupts(void)
{
	PORTF.DIRCLR = PIN1_bm;				  // Set PIN1 sebagai input (SW0)
	PORTF.PIN1CTRL = PORT_ISC_FALLING_gc; // Detect falling edges
	PORTF.INT0MASK = PIN1_bm;			  // Enable interrupt untuk PIN1
	PORTF.INTCTRL = PORT_INT0LVL_LO_gc;	  // Set level interrupt ke low
	PMIC.CTRL |= PMIC_LOLVLEN_bm;		  // Enable low-level interrupts
	cpu_irq_enable();					  // Enable global interrupts
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

void init_door_sensor(void)
{
	
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
	
	while (true)
	{
		 // Mengecek status sensor pintu
		 bool is_closed = !(PORTC.IN & PIN2_bm);
		 snprintf(strbuf, sizeof(strbuf), "closed: %s", is_closed ? "true" : "false");
		 gfx_mono_draw_string(strbuf, 0, 24, &sysfont);
		 
		 
		if (alarm_active)
		{
			// Tampilkan status pintu dan counter di LCD
			gfx_mono_draw_string("Pintu Terbuka ", 0, 8, &sysfont);
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

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
		}
		else
		{
			// Jika alarm tidak aktif (pintu tertutup)
			// Reset semua
			TCC0.CCA = 0;
			LED_Off(LED0);
			LED_Off(LED1);
			counter = 0;

			gfx_mono_draw_string("Pintu Tertutup ", 0, 8, &sysfont); // Update status di LCD
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 16, &sysfont);
		}
	}
}