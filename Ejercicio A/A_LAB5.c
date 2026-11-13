#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>

// Dirección I2C
#define PCF8574_ADDR 0x27

// Asignación de Pines
#define BUTTON_INT0 PD2 // Pin D2
#define TOGGLE_INT1 PD3 // Pin D3
#define DHT11_PIN   PD4 // Pin D4
#define BUZZER_PIN  PD5 // Pin D5
#define LED_PIN     PB5 // Pin D13

// Variables Globales Volátiles
volatile uint32_t system_ms = 0;
volatile uint32_t last_activity_ms = 0;
volatile uint16_t buzzer_timer_ms = 0;
volatile uint8_t adc_trigger_flag = 0;
volatile uint16_t pot_val = 0;

volatile uint8_t lcd_backlight_mask = 0x08; // 0x08 = ON, 0x00 = OFF

// Prototipos de Funciones de Estado para la FSM
void estado_activo(void);
void estado_standby(void);
void estado_apagado(void);

// PUNTERO A FUNCIÓN DE ESTADO (Mantiene el estado actual de la FSM)
void (*estado_actual)(void) = estado_activo;
void (*ultimo_estado_ejecutado)(void) = NULL;

// Variables compartidas entre tareas dentro del modo activo
uint8_t temp = 25, hum = 50;
uint8_t new_temp = 0, new_hum = 0;
uint32_t last_led_ms = 0;
uint32_t last_lcd_ms = 0;
uint32_t last_dht_ms = 0;


// CONFIGURACIÓN DE TIMERS


// Timer0: Base de tiempos de 1 ms (Modo CTC)
void Timer0_init(void) {
	TCCR0A = (1 << WGM01);
	TCCR0B = (1 << CS01) | (1 << CS00);
	OCR0A = 249;
	TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
	system_ms++;
	
	// Control no bloqueante del Buzzer
	if (buzzer_timer_ms > 0) {
		buzzer_timer_ms--;
		PORTD ^= (1 << BUZZER_PIN); // Conmutar pin para generar tono
		} else {
		PORTD &= ~(1 << BUZZER_PIN); // Apagar buzzer
	}
}

// Timer1: Interrupción periódica de 100 ms para solicitar lectura del ADC
void Timer1_init(void) {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS12); // Modo CTC, Prescaler 256 -> 62.5 kHz
	OCR1A = 6249;                        // (6249 + 1) / 62500Hz = 0.1s = 100ms
	TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
	adc_trigger_flag = 1;
}

