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

// CONTROL DEL BUZZER (PIN D5)

void Buzzer_init(void) {
	DDRD |= (1 << BUZZER_PIN);
	PORTD &= ~(1 << BUZZER_PIN);
}

void Buzzer_trigger(uint16_t duration_ms) {
	cli();
	buzzer_timer_ms = duration_ms;
	sei();
}


// DRIVER DHT11 (LECTURA DE TEMPERATURA / HUMEDAD)

uint8_t DHT11_read(uint8_t *humidity, uint8_t *temperature) {
	uint8_t data[5] = {0, 0, 0, 0, 0};
	uint8_t i, j;
	uint16_t timeout;

	cli();

	DDRD |= (1 << DHT11_PIN);
	PORTD &= ~(1 << DHT11_PIN);
	_delay_ms(20);
	PORTD |= (1 << DHT11_PIN);
	_delay_us(30);
	DDRD &= ~(1 << DHT11_PIN);
	PORTD |= (1 << DHT11_PIN);

	_delay_us(40);
	if (PIND & (1 << DHT11_PIN)) { sei(); return 0; }
	_delay_us(80);
	if (!(PIND & (1 << DHT11_PIN))) { sei(); return 0; }
	_delay_us(80);

	for (j = 0; j < 5; j++) {
		uint8_t result = 0;
		for (i = 0; i < 8; i++) {
			timeout = 0;
			while (!(PIND & (1 << DHT11_PIN))) {
				if (++timeout > 500) { sei(); return 0; }
				_delay_us(1);
			}
			_delay_us(30);
			if (PIND & (1 << DHT11_PIN)) {
				result |= (1 << (7 - i));
			}
			timeout = 0;
			while (PIND & (1 << DHT11_PIN)) {
				if (++timeout > 500) break;
				_delay_us(1);
			}
		}
		data[j] = result;
	}

	sei();

	if ((uint8_t)(data[0] + data[1] + data[2] + data[3]) == data[4] && data[0] != 0) {
		*humidity = data[0];
		*temperature = data[2];
		return 1;
	}
	return 0;
}

// CONVERTIDOR ANALÓGICO-DIGITAL (ADC)

void ADC_init(void) {
	ADMUX = (1 << REFS0); // AVcc con capacitor externo en AREF
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Prescaler 128
}

uint16_t ADC_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}


// DRIVER I2C / TWI Y PANTALLA LCD

void TWI_init(void) {
	TWSR = 0x00;
	TWBR = (uint8_t)(((F_CPU / 100000UL) - 16) / 2); // 100 kHz I2C
	TWCR = (1 << TWEN);
}

void TWI_start(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void TWI_stop(void) {
	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
}

void TWI_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN);
	while (!(TWCR & (1 << TWINT)));
}

void LCD_backlight(uint8_t turn_on) {
	if (turn_on) lcd_backlight_mask = 0x08;
	else lcd_backlight_mask = 0x00;
	
	TWI_start();
	TWI_write(PCF8574_ADDR << 1);
	TWI_write(lcd_backlight_mask);
	TWI_stop();
}

void LCD_send_nibble(uint8_t nibble, uint8_t mode) {
	uint8_t data = (nibble & 0xF0) | mode | lcd_backlight_mask;
	TWI_start();
	TWI_write(PCF8574_ADDR << 1);
	TWI_write(data | 0x04);
	_delay_us(1);
	TWI_write(data & ~0x04);
	_delay_us(50);
	TWI_stop();
}

void LCD_command(uint8_t cmd) {
	LCD_send_nibble(cmd & 0xF0, 0);
	LCD_send_nibble((cmd << 4) & 0xF0, 0);
}

void LCD_data(uint8_t data) {
	LCD_send_nibble(data & 0xF0, 1);
	LCD_send_nibble((data << 4) & 0xF0, 1);
}

void LCD_init(void) {
	_delay_ms(50);
	LCD_send_nibble(0x30, 0); _delay_ms(5);
	LCD_send_nibble(0x30, 0); _delay_us(150);
	LCD_send_nibble(0x30, 0);
	LCD_send_nibble(0x20, 0);

	LCD_command(0x28); // 4 bits, 2 líneas
	LCD_command(0x0C); // Display ON, Cursor OFF
	LCD_command(0x06); // Incremento automático
	LCD_command(0x01); // Limpiar pantalla
	_delay_ms(2);
}

void LCD_print(const char *str) {
	while (*str) LCD_data((uint8_t)*str++);
}

void LCD_print_num(uint16_t num) {
	char buf[6];
	itoa(num, buf, 10);
	LCD_print(buf);
}

void LCD_set_cursor(uint8_t col, uint8_t row) {
	uint8_t offsets[] = {0x00, 0x40};
	LCD_command(0x80 | (col + offsets[row]));
}
// INTERRUPCIONES EXTERNAS

void External_Interrupts_init(void) {
	DDRD &= ~((1 << DDD2) | (1 << DDD3));
	PORTD |= (1 << PORTD2) | (1 << PORTD3); // Pull-up interno

	EICRA = (1 << ISC11) | (1 << ISC01); // Flanco descendente
	EIMSK = (1 << INT0) | (1 << INT1);
}

// INT0: Pulsador de evento -> reactiva sistema y reinicia inactividad
ISR(INT0_vect) {
	last_activity_ms = system_ms;
	estado_actual = estado_activo;
	buzzer_timer_ms = 80;
}

// INT1: Conmutador de estado ACTIVO / STANDBY
ISR(INT1_vect) {
	last_activity_ms = system_ms;
	if (estado_actual == estado_activo) {
		estado_actual = estado_standby;
		} else if (estado_actual == estado_standby) {
		estado_actual = estado_activo;
	}
}


// FUNCIONES DE ESTADO DE LA FSM

void estado_activo(void) {
	uint32_t current_ms;
	cli(); current_ms = system_ms; sei();

	// Transición de entrada
	if (ultimo_estado_ejecutado != estado_activo) {
		LCD_backlight(1);
		LCD_command(0x01); _delay_ms(2);
		Buzzer_trigger(80);
		ultimo_estado_ejecutado = estado_activo;
	}

	//  Lectura de ADC cada 100 ms
	if (adc_trigger_flag) {
		adc_trigger_flag = 0;
		pot_val = ADC_read(0);
	}

	//  Parpadeo del LED cada 200 ms
	if (current_ms - last_led_ms >= 200) {
		last_led_ms = current_ms;
		PORTB ^= (1 << LED_PIN);
	}

	// Lectura del Sensor DHT11 cada 2000 ms
	if (current_ms - last_dht_ms >= 2000) {
		last_dht_ms = current_ms;
		if (DHT11_read(&new_hum, &new_temp)) {
			hum = new_hum;
			temp = new_temp;
		}
	}

	//  Refresco de pantalla LCD cada 500 ms
	if (current_ms - last_lcd_ms >= 500) {
		last_lcd_ms = current_ms;

		LCD_set_cursor(0, 0);
		LCD_print("ACTIVO   T:"); LCD_print_num(temp); LCD_print("C ");

		LCD_set_cursor(0, 1);
		LCD_print("H:"); LCD_print_num(hum);
		LCD_print("%   P:"); LCD_print_num(pot_val); LCD_print("   ");
	}

	// Evaluación de Transición de Inactividad (10 segundos)
	if (current_ms - last_activity_ms >= 10000UL) {
		estado_actual = estado_standby;
	}
}

void estado_standby(void) {
	uint32_t current_ms;
	cli(); current_ms = system_ms; sei();

	if (ultimo_estado_ejecutado != estado_standby) {
		LCD_backlight(1);
		LCD_command(0x01); _delay_ms(2);
		LCD_set_cursor(0, 0); LCD_print("Esperando...");
		LCD_set_cursor(0, 1); LCD_print("Modo STANDBY");
		PORTB &= ~(1 << LED_PIN); // Apagar LED
		ultimo_estado_ejecutado = estado_standby;
	}

	// Evaluación de Transición por Inactividad Global (30 segundos desde la última actividad)
	if (current_ms - last_activity_ms >= 30000UL) {
		estado_actual = estado_apagado;
	}
}

void estado_apagado(void) {
	if (ultimo_estado_ejecutado != estado_apagado) {
		LCD_command(0x01); _delay_ms(2);
		LCD_set_cursor(0, 0); LCD_print("Estado: APAGADO");
		_delay_ms(500);
		
		LCD_command(0x01); _delay_ms(2);
		LCD_backlight(0); // Apagar retroiluminación LCD
		
		PORTB &= ~(1 << LED_PIN);
		ultimo_estado_ejecutado = estado_apagado;
	}
}


// BUCLE (MAIN)

int main(void) {
	External_Interrupts_init();
	TWI_init();
	LCD_init();
	ADC_init();
	Buzzer_init();
	Timer0_init();
	Timer1_init();

	DDRB |= (1 << LED_PIN);
	sei();

	Buzzer_trigger(100);
	last_activity_ms = system_ms;

	while (1) {
		estado_actual();
	}

	return 0;
}