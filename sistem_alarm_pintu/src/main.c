#include <asf.h>
#include <stdio.h>
#include <ioport.h>
#include <board.h>

// Variabel global
volatile bool door_open = false;   // Status pintu (true jika terbuka)
volatile uint8_t counter = 0;      // Counter waktu (detik)
volatile bool alarm_active = false; // Status alarm aktif atau tidak
volatile bool toggle_state = false; // Status untuk kedap-kedip bergantian
volatile bool last_button_state = true;  // Status terakhir tombol (untuk debounce)
static char strbuf[128];           // Buffer untuk tampilan LCD

// ISR untuk SW0 (PORTF PIN1) – Deteksi perubahan state tombol
ISR(PORTF_INT0_vect)
{
	bool current_button_state = !(PORTF.IN & PIN1_bm);  // Baca status tombol (0 = ditekan)

	// Jika tombol dilepas (rising edge), buka pintu
	if (last_button_state && !current_button_state) {
		door_open = true;      // Pintu terbuka
		alarm_active = true;   // Aktifkan alarm
		counter = 0;           // Reset counter
	}

	// Jika tombol ditekan (falling edge), tutup pintu
	if (!last_button_state && current_button_state) {
		door_open = false;     // Pintu tertutup
		alarm_active = false;  // Matikan alarm
	}

	last_button_state = current_button_state;  // Update status tombol terakhir
}

// Inisialisasi interrupt eksternal untuk SW0
void init_interrupts(void)
{
	PORTF.DIRCLR = PIN1_bm;  // Set PIN1 sebagai input (SW0)
	PORTF.PIN1CTRL |= PORT_OPC_PULLUP_gc | PORT_ISC_BOTHEDGES_gc;  // Detect both edges
	PORTF.INT0MASK = PIN1_bm;  // Enable interrupt untuk PIN1
	PORTF.INTCTRL = PORT_INT0LVL_LO_gc;  // Set level interrupt ke low
	PMIC.CTRL |= PMIC_LOLVLEN_bm;  // Enable low-level interrupts
	cpu_irq_enable();  // Enable global interrupts
}

// Setup LED
void setup_led(void)
{
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD, 0), IOPORT_DIR_OUTPUT);  // LED0
	ioport_set_pin_dir(IOPORT_CREATE_PIN(PORTD, 1), IOPORT_DIR_OUTPUT);  // LED1
	LED_Off(LED0);  // Pastikan LED0 mati saat awal
	LED_Off(LED1);  // Pastikan LED1 mati saat awal
}

// Setup PWM untuk buzzer
void setup_pwm_buzzer(void)
{
	/* Set output pin untuk servo */
	PORTC.DIR |= PIN0_bm;

	/* Set Register */
	TCC0.CTRLA = TC_CLKSEL_DIV8_gc;
	TCC0.CTRLB = (PIN4_bm) | (PIN2_bm) | (PIN1_bm);
	
	/* Set Period PWM dimana Servo menggunakan 20ms(50Hz) */
	TCC0.PER = 2000;

	/* Set Compare Register value default untuk 0 derajat */
	TCC0.CCA = 500;
}

// Inisialisasi LCD
void setup_lcd(void)
{
	gfx_mono_init();
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);  // Aktifkan backlight
	gfx_mono_draw_string("Alarm Pintu", 0, 0, &sysfont);  // Pesan awal
}

// Fungsi utama
int main(void)
{
	// Inisialisasi board dan sistem
	board_init();
	sysclk_init();
	setup_lcd();
	setup_led();  // Inisialisasi LED
	setup_pwm_buzzer();
	init_interrupts();

	// Loop utama
	while (1) {
		if (alarm_active) {  // Jika alarm aktif (pintu terbuka)
			// Tampilkan status pintu dan counter di LCD
			gfx_mono_draw_string("Pintu Terbuka", 0, 0, &sysfont);
			snprintf(strbuf, sizeof(strbuf), "Waktu: %02d detik", counter);
			gfx_mono_draw_string(strbuf, 0, 16, &sysfont);

			if (counter >= 10) {
				TCC0.CCA = 2000;  // Intensitas maksimal buzzer
				LED_On(LED0);  // Nyalakan LED0, Matikan LED1
				LED_Off(LED1);
				delay_ms(100);  // Tunda untuk simulasi
				LED_Off(LED0);  // Nyalakan LED1, Matikan LED0
				LED_On(LED1);
				delay_ms(100);  // Tunda untuk simulasi
				} else {
				// Sebelum 10 detik, nyalakan LED0 dan LED1 secara bersamaan
				LED_On(LED0);
				LED_On(LED1);
				
				TCC0.CCA = 1000;  // Bunyi normal (intensitas sedang)
				delay_ms(200);   // Bunyi pendek berulang
				TCC0.CCA = 0;    // Matikan buzzer
				delay_ms(200);
			}
			// Tambah counter setiap detik
			delay_ms(500);  // Tunggu 1 detik
			counter++;
			} else {  // Jika alarm tidak aktif (pintu tertutup)
			// Reset semua
			TCC0.CCA = 0;  // Matikan buzzer
			LED_Off(IOPORT_CREATE_PIN(PORTD, 0));  // Matikan LED0
			LED_Off(IOPORT_CREATE_PIN(PORTD, 1));  // Matikan LED1
			gfx_mono_draw_string("Pintu Tertutup", 0, 0, &sysfont);  // Update LCD
			counter = 0;  // Reset counter
		}
	}
}