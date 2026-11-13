#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>

// Configuración UART (9600 Baudios)
#define BAUDRATE 9600
#define UBRR_VAL ((F_CPU / (16UL * BAUDRATE)) - 1)

// Estados del Sistema
#define STATE_ACTIVO  0
#define STATE_STANDBY 1
#define STATE_APAGADO 2

// Dirección I2C LCD
#define PCF8574_ADDR 0x27

// Pines
#define BUTTON_INT0 PD2 // Pin D2 (Pulsador Evento / Despertador / Recuperación)
#define TOGGLE_INT1 PD3 // Pin D3 (Conmutador Activo/Standby)
#define DHT11_PIN   PD4 // Pin D4 (Sensor Temp/Hum)
#define BUZZER_PIN  PD5 // Pin D5 (Zumbador)
#define LED_PIN     PB5 // Pin D13 (LED Arduino)

// Protocolo de Tramas Bluetooth: [START][COMANDO][DATO][CHECKSUM][STOP]
#define START_BYTE          0xAA
#define STOP_BYTE           0x55

// Comandos Bluetooth
#define CMD_GET_STATUS      0x01
#define CMD_GET_SENSORS     0x02
#define CMD_SET_STATE       0x03
#define CMD_SET_ACTUATOR    0x04
#define CMD_SAFE_STATE      0x05
#define CMD_GET_ALARMS      0x06
#define CMD_RECOVER_SAFE    0x07
#define CMD_EVENT_NOTIFY    0x08
#define CMD_ERROR           0xEE

// Códigos de Error Bluetooth
#define ERR_CHECKSUM        0x01
#define ERR_BAD_STOP        0x02
#define ERR_UNKNOWN_CMD     0x03
#define ERR_INVALID_DATA    0x04
#define ERR_FRAME_TIMEOUT   0x05
#define ERR_SAFE_LOCKED     0x06

// Máscaras de Eventos de Alarma
#define ALARM_NONE          0x00
#define ALARM_EMERGENCY     0x01
#define ALARM_WDT_WAKEUP    0x02
#define ALARM_COMM_ERROR    0x04

// Búferes Circulares UART (RX y TX)
#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 64

volatile uint8_t rx_ring[RX_BUFFER_SIZE];
volatile uint8_t rx_head = 0;
volatile uint8_t rx_tail = 0;

volatile uint8_t tx_ring[TX_BUFFER_SIZE];
volatile uint8_t tx_head = 0;
volatile uint8_t tx_tail = 0;

// Variables Globales del Sistema
volatile uint8_t system_state = STATE_ACTIVO;
volatile uint8_t emergency_locked = 0;    // Bloqueo de seguridad ante emergencia
volatile uint8_t alarm_flags = ALARM_NONE;
volatile uint32_t system_ms = 0;          // Base de tiempos 1 ms (Timer0)
volatile uint32_t last_activity_ms = 0;   // Timestamp de última actividad válida
volatile uint16_t buzzer_timer_ms = 0;    // Buzzer no bloqueante
volatile uint8_t adc_trigger_flag = 0;    // Trigger ADC 100 ms (Timer1)
volatile uint16_t pot_val = 0;
volatile uint8_t temp_val = 25, hum_val = 50;

volatile uint8_t lcd_backlight_mask = 0x08;
volatile uint8_t wdt_wakeup_count = 0;


// TIMERS (BASE DE TIEMPOS NO BLOQUEANTE)
void Timer0_init(void) {
	TCCR0A = (1 << WGM01);              // Modo CTC
	TCCR0B = (1 << CS01) | (1 << CS00); // Prescaler 64
	OCR0A = 249;                        // Interrupción cada 1 ms
	TIMSK0 = (1 << OCIE0A);
}

ISR(TIMER0_COMPA_vect) {
	system_ms++;
	if (buzzer_timer_ms > 0) {
		buzzer_timer_ms--;
		PORTD ^= (1 << BUZZER_PIN);
		} else {
		PORTD &= ~(1 << BUZZER_PIN);
	}
}

void Timer1_init(void) {
	TCCR1A = 0;
	TCCR1B = (1 << WGM12) | (1 << CS12); // Modo CTC, Prescaler 256
	OCR1A = 6249;                        // Interrupción cada 100 ms
	TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect) {
	adc_trigger_flag = 1;
}

// WATCHDOG TIMER (PRUEBA A Y PRUEBA B)

void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void) {
	MCUSR = 0;
	wdt_disable();
}

// Configuración para PRUEBA B: Reset de recuperación a ~2s
void WDT_enable_reset_2s(void) {
	uint8_t sreg = SREG;
	cli();
	wdt_reset();
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0);
	SREG = sreg;
}

// Configuración para PRUEBA A: Interrupción como despertador periódico a ~8s
void WDT_enable_interrupt_8s(void) {
	uint8_t sreg = SREG;
	cli();
	wdt_reset();
	WDTCSR |= (1 << WDCE) | (1 << WDE);
	WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0);
	SREG = sreg;
}

ISR(WDT_vect) {
	wdt_wakeup_count++;
	alarm_flags |= ALARM_WDT_WAKEUP;
}


// PERIFÉRICOS BÁSICOS

void Buzzer_init(void) {
	DDRD |= (1 << BUZZER_PIN);
	PORTD &= ~(1 << BUZZER_PIN);
}

void Buzzer_trigger(uint16_t duration_ms) {
	uint8_t sreg = SREG;
	cli();
	buzzer_timer_ms = duration_ms;
	SREG = sreg;
}

void ADC_init(void) {
	ADMUX = (1 << REFS0);
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel) {
	ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
	ADCSRA |= (1 << ADSC);
	while (ADCSRA & (1 << ADSC));
	return ADC;
}

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
			if (PIND & (1 << DHT11_PIN)) result |= (1 << (7 - i));
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

// UART Y BLUETOOTH CON BÚFERES CIRCULARES
void UART_init(void) {
	UBRR0H = (uint8_t)(UBRR_VAL >> 8);
	UBRR0L = (uint8_t)(UBRR_VAL);
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

// ISR de recepción: solo guarda el byte recibido en el búfer circular RX
ISR(USART_RX_vect) {
	uint8_t byte_in = UDR0;
	uint8_t next_head = (uint8_t)((rx_head + 1) % RX_BUFFER_SIZE);
	if (next_head != rx_tail) {
		rx_ring[rx_head] = byte_in;
		rx_head = next_head;
	}
}

// ISR de transmisión: envía el siguiente byte del búfer circular TX
ISR(USART_UDRE_vect) {
	if (tx_head == tx_tail) {
		UCSR0B &= ~(1 << UDRIE0); // Búfer vacío, deshabilita interrupción UDRE
		} else {
		UDR0 = tx_ring[tx_tail];
		tx_tail = (uint8_t)((tx_tail + 1) % TX_BUFFER_SIZE);
	}
}

void UART_send_byte(uint8_t data) {
	uint8_t next_head = (uint8_t)((tx_head + 1) % TX_BUFFER_SIZE);
	while (next_head == tx_tail); // Espera solo si el búfer circular está lleno

	uint8_t sreg = SREG;
	cli();
	tx_ring[tx_head] = data;
	tx_head = next_head;
	UCSR0B |= (1 << UDRIE0); // Habilita interrupción de registro vacío
	SREG = sreg;
}

void UART_send_frame(uint8_t cmd, uint8_t data) {
	uint8_t chk = cmd ^ data;
	UART_send_byte(START_BYTE);
	UART_send_byte(cmd);
	UART_send_byte(data);
	UART_send_byte(chk);
	UART_send_byte(STOP_BYTE);
}

void UART_flush_tx(void) {
	while (tx_head != tx_tail);
	while (!(UCSR0A & (1 << TXC0)));
}

uint8_t UART_available(void) {
	return (rx_head != rx_tail);
}

uint8_t UART_read_byte(void) {
	uint8_t data = rx_ring[rx_tail];
	rx_tail = (uint8_t)((rx_tail + 1) % RX_BUFFER_SIZE);
	return data;
}

void Bluetooth_execute_command(uint8_t cmd, uint8_t data, uint32_t now_ms) {
	if (cmd == CMD_GET_STATUS) {
		// Consulta periódica: NO reinicia el contador de inactividad
		UART_send_frame(CMD_GET_STATUS, system_state);
	}
	else if (cmd == CMD_GET_SENSORS) {
		// Permite pedir Temp (0), Hum (1) o Potenciómetro escalado 0-255 (2)
		if (data == 0) {
			UART_send_frame(CMD_GET_SENSORS, temp_val);
			} else if (data == 1) {
			UART_send_frame(CMD_GET_SENSORS, hum_val);
			} else if (data == 2) {
			UART_send_frame(CMD_GET_SENSORS, (uint8_t)(pot_val >> 2));
			} else {
			UART_send_frame(CMD_ERROR, ERR_INVALID_DATA);
		}
	}
	else if (cmd == CMD_SET_STATE) {
		if (emergency_locked) {
			UART_send_frame(CMD_ERROR, ERR_SAFE_LOCKED);
			return;
		}
		if (data == STATE_ACTIVO || data == STATE_STANDBY || data == STATE_APAGADO) {
			cli();
			last_activity_ms = now_ms;
			system_state = data;
			sei();
			Buzzer_trigger(50);
			UART_send_frame(CMD_SET_STATE, system_state);
			} else {
			UART_send_frame(CMD_ERROR, ERR_INVALID_DATA);
		}
	}
	else if (cmd == CMD_SET_ACTUATOR) {
		if (emergency_locked) {
			UART_send_frame(CMD_ERROR, ERR_SAFE_LOCKED);
			return;
		}
		cli();
		last_activity_ms = now_ms;
		sei();
		if (data == 1) {
			PORTB |= (1 << LED_PIN);
			UART_send_frame(CMD_SET_ACTUATOR, 1);
			} else if (data == 0) {
			PORTB &= ~(1 << LED_PIN);
			UART_send_frame(CMD_SET_ACTUATOR, 0);
			} else if (data == 2) {
			Buzzer_trigger(150);
			UART_send_frame(CMD_SET_ACTUATOR, 2);
			} else {
			UART_send_frame(CMD_ERROR, ERR_INVALID_DATA);
		}
	}
	else if (cmd == CMD_SAFE_STATE) {
		cli();
		last_activity_ms = now_ms;
		emergency_locked = 1;
		alarm_flags |= ALARM_EMERGENCY;
		system_state = STATE_APAGADO;
		sei();
		PORTB &= ~(1 << LED_PIN);
		Buzzer_trigger(300);
		UART_send_frame(CMD_SAFE_STATE, 0xFF);
	}
	else if (cmd == CMD_GET_ALARMS) {
		UART_send_frame(CMD_GET_ALARMS, alarm_flags);
	}
	else if (cmd == CMD_RECOVER_SAFE) {
		// Condición explícita de recuperación: enviar dato 0xAA
		if (data == 0xAA) {
			cli();
			emergency_locked = 0;
			alarm_flags &= ~ALARM_EMERGENCY;
			last_activity_ms = now_ms;
			system_state = STATE_ACTIVO;
			sei();
			Buzzer_trigger(80);
			UART_send_frame(CMD_RECOVER_SAFE, STATE_ACTIVO);
			} else {
			UART_send_frame(CMD_ERROR, ERR_INVALID_DATA);
		}
	}
	else {
		alarm_flags |= ALARM_COMM_ERROR;
		UART_send_frame(CMD_ERROR, ERR_UNKNOWN_CMD);
	}
}

void Bluetooth_process_rx(uint32_t now_ms) {
	static uint8_t frame_buf[5];
	static uint8_t frame_idx = 0;
	static uint32_t frame_start_ms = 0;

	// Detección de tramas incompletas (timeout de 100 ms)
	if (frame_idx > 0 && (now_ms - frame_start_ms) >= 100UL) {
		frame_idx = 0;
		alarm_flags |= ALARM_COMM_ERROR;
		UART_send_frame(CMD_ERROR, ERR_FRAME_TIMEOUT);
	}

	while (UART_available()) {
		uint8_t byte_in = UART_read_byte();

		if (frame_idx == 0) {
			if (byte_in == START_BYTE) {
				frame_buf[0] = byte_in;
				frame_idx = 1;
				frame_start_ms = now_ms;
			}
			} else {
			frame_buf[frame_idx] = byte_in;
			frame_idx++;

			if (frame_idx == 5) {
				frame_idx = 0;
				uint8_t cmd  = frame_buf[1];
				uint8_t data = frame_buf[2];
				uint8_t chk  = frame_buf[3];
				uint8_t stop = frame_buf[4];

				if (stop != STOP_BYTE) {
					alarm_flags |= ALARM_COMM_ERROR;
					UART_send_frame(CMD_ERROR, ERR_BAD_STOP);
					} else if (chk != (cmd ^ data)) {
					alarm_flags |= ALARM_COMM_ERROR;
					UART_send_frame(CMD_ERROR, ERR_CHECKSUM);
					} else {
					Bluetooth_execute_command(cmd, data, now_ms);
				}
			}
		}
	}
}
