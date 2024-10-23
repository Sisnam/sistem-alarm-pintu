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

// Variabel global
volatile uint8_t seconds = 0;   // Variabel untuk menghitung waktu
volatile uint8_t intensity = 0; // Variabel untuk mengatur intensitas

// Prototipe fungsi
void setup_pwm_buzzer(void);
void setup_pwm_led(void);
void setup_timer_interrupt(void);
void adjust_pwm_based_on_intensity(void);

// Interrupt handler untuk timer overflow
ISR(TCC0_OVF_vect)
{
    static uint16_t overflow_count = 0;
    overflow_count++;

    // Hitung overflow hingga mencapai 1 detik
    if (overflow_count >= 61) { // Sekitar 1 detik pada clock 16 MHz dengan prescaler 1024
        overflow_count = 0;
        seconds++;

        // Jika sudah mencapai 15 detik, tingkatkan intensitas
        if (seconds >= 15) {
            intensity++;
            if (intensity > 3) intensity = 0;
            seconds = 0; // Reset setelah 15 detik
        }
    }
}

// Setup PWM untuk buzzer menggunakan TCC0
void setup_pwm_buzzer(void)
{
    // Set output pin untuk buzzer (misal PIN0)
    PORTC.DIR |= PIN0_bm;

    // Setup PWM pada timer TCC0
    TCC0.CTRLB |= TC_WGMODE_FRQ_gc | TC0_CCAEN_bm; // Mode fast PWM
    TCC0.CTRLA |= TC_CLKSEL_DIV8_gc;               // Prescaler 8
    TCC0.PER = 255;                                // Period PWM
    TCC0.CCA = 128;                                // Duty cycle default (50%)
}

// Setup PWM untuk LED menggunakan TCC1
void setup_pwm_led(void)
{
    // Set output pin untuk LED (misal PIN1)
    PORTD.DIR |= PIN1_bm;

    // Setup PWM pada timer TCC1
    TCC1.CTRLB |= TC_WGMODE_FRQ_gc | TC1_CCAEN_bm; // Mode fast PWM
    TCC1.CTRLA |= TC_CLKSEL_DIV8_gc;               // Prescaler 8
    TCC1.PER = 255;                                // Period PWM
    TCC1.CCA = 128;                                // Duty cycle default (50%)
}

// Setup timer interrupt untuk menghitung waktu setiap 1 detik
void setup_timer_interrupt(void)
{
    // Set timer overflow setiap 1 detik
    TCC0.PER = 50000;                              // Setting periode timer overflow
    TCC0.CTRLA |= TC_CLKSEL_DIV1024_gc;            // Prescaler 1024
    TCC0.INTCTRLA |= TC_OVFINTLVL_LO_gc;           // Enable overflow interrupt
}

// Menyesuaikan PWM berdasarkan intensitas
void adjust_pwm_based_on_intensity(void)
{
    if (intensity == 0) {
        TCC0.CCA = 128;  // Suara buzzer normal
        TCC1.CCA = 128;  // LED kedip normal
    } else if (intensity == 1) {
        TCC0.CCA = 200;  // Suara lebih cepat
        TCC1.CCA = 200;  // LED lebih cepat
    } else if (intensity == 2) {
        TCC0.CCA = 255;  // Suara sangat cepat
        TCC1.CCA = 255;  // LED sangat cepat
    }
}

int main(void)
{
    // Inisialisasi board dan clock sistem
    sysclk_init();
    board_init();

    // Inisialisasi PWM untuk buzzer dan LED
    setup_pwm_buzzer();
    setup_pwm_led();

    // Inisialisasi timer untuk interrupt setiap 1 detik
    setup_timer_interrupt();

    // Enable global interrupts
    cpu_irq_enable();

    while (1)
    {
        // Setiap kali timer interrupt terjadi, adjust_pwm_based_on_intensity akan memanggil perubahan intensitas
        adjust_pwm_based_on_intensity();
    }
}
