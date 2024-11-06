#include <asf.h>
#include <util/delay.h>
#include <stdio.h>

#define DHT_PIN  IOPORT_CREATE_PIN(PORTD, 2)  // Define the DHT22 data pin
#define RELAY_PIN IOPORT_CREATE_PIN(PORTC, 0) // Define the relay pin on PORTC PIN0
#define BUZZER_PIN IOPORT_CREATE_PIN(PORTC, 1) // Define the buzzer pin on PORTC PIN1
#define DHT_TIMEOUT 10000  // Timeout value

static char display_buffer[128];  // Buffer for displaying text on the LCD
volatile uint16_t temperature = 0; // Global variable for temperature

// Function prototypes
void DHT22_send_start(void);
int DHT22_check_response(void);
uint8_t DHT22_read_byte(void);
int DHT22_read_data(uint16_t *temperature);
void gfx_mono_draw_string_on_lcd(const char *str, uint8_t x, uint8_t y);
void gfx_mono_clear_screen(void);
void init_external_interrupt(void);

int main(void) {
	// Initialize system and LCD
	sysclk_init();
	board_init();
	gfx_mono_init();  // Initialize the GFX monochrome LCD

	// Enable LCD backlight
	gpio_set_pin_high(LCD_BACKLIGHT_ENABLE_PIN);

	// Initialize relay pin and buzzer pin as output with pull-down resistor
	ioport_set_pin_dir(RELAY_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_low(RELAY_PIN);

	ioport_set_pin_dir(BUZZER_PIN, IOPORT_DIR_OUTPUT);
	ioport_set_pin_low(BUZZER_PIN);

	// Configure timer interrupt
	init_external_interrupt();

	// Enable global interrupts
	cpu_irq_enable();

	while (1) {
		DHT22_send_start();

		// Then, check for DHT22 sensor response and read the temperature
		if (DHT22_check_response() == 0 && DHT22_read_data(&temperature) == 0) {
			// Convert temperature to Celsius and display on LCD
			int temp_integer = temperature / 10;    // Integer part of the temperature
			int temp_decimal = temperature % 10;    // Decimal part of the temperature

			// Display temperature in "XX.XÅãC" format
			snprintf(display_buffer, sizeof(display_buffer), "Temp: %d.%dC", temp_integer, temp_decimal);
			gfx_mono_draw_string_on_lcd(display_buffer, 0, 16);  // Display temperature on LCD

			// Check if the temperature exceeds 30ÅãC for relay
			if (temp_integer >= 27) {
				ioport_set_pin_high(RELAY_PIN);  // Nyalakan relay jika suhu >= 30ÅãC
				// Display fan if >= 30
				snprintf(display_buffer, sizeof(display_buffer), "Kipas Nyala");
				gfx_mono_draw_string_on_lcd(display_buffer, 0, 0);  // Display fan in lcd

				} else {
				ioport_set_pin_low(RELAY_PIN);   // Matikan relay jika suhu < 30ÅãC
				snprintf(display_buffer, sizeof(display_buffer), "Kipas  Mati");
				gfx_mono_draw_string_on_lcd(display_buffer, 0, 0);  // Display fan in lcd
			}

			// Check if the temperature exceeds 31ÅãC for buzzer
			if (temp_integer >= 28) {
				ioport_set_pin_high(BUZZER_PIN);  // Nyalakan buzzer jika suhu >= 31ÅãC
				} else {
				ioport_set_pin_low(BUZZER_PIN);   // Matikan buzzer jika suhu < 31ÅãC
			}
			} else {
			snprintf(display_buffer, sizeof(display_buffer), "No Response");
			gfx_mono_draw_string_on_lcd(display_buffer, 0, 16);  // Display error if no response
		}
		_delay_ms(1000);
	}
}

// Configure Timer/Counter for interrupt
void init_external_interrupt(void)
{
	// Set PORTF Pin 1 as input (clear bit 1 in DIR register)
	PORTF.DIR &= ~(1 << 1);  // Clear bit 1 to set PORTF_PIN1 as input

	// Enable pull-up resistor on PORTF Pin 1 (PULLUP configuration in PIN1CTRL)
	PORTF.PIN1CTRL &= (3<<3); //(0x03 << 3);  // Set PULLUP bit in PIN1CTRL (bit 3)

	// Configure interrupt to trigger on rising edge (ISC setting in PIN1CTRL)
	PORTF.PIN1CTRL |= (0x03 << 0);  // Set ISC0 in PIN1CTRL (bit 0) for low level

	// Enable interrupt for PORTF Pin 1 (INT0MASK configuration)
	PORTF.INT0MASK = (1 << 1);  // Set bit 1 to enable INT0 for PORTF_PIN1

	// Set interrupt priority to medium level (INTCTRL configuration)
	PORTF.INTCTRL = (0x01 << 0);  // Set INT0LVL to low
	
	PMIC.CTRL |= 0x01; //PMIC_LOLVLEN_bm;

	// Enable global interrupts
	SREG |= (1<<7);
}

// Timer overflow interrupt service routine (ISR)
ISR(PORTF_INT0_vect)
{
	ioport_set_pin_high(RELAY_PIN);  // Nyalakan relay jika suhu >= 30ÅãC
	// Display fan if >= 30
	snprintf(display_buffer, sizeof(display_buffer), "Kipas Nyala");
	gfx_mono_draw_string_on_lcd(display_buffer, 0, 0);  // Display fan in lcd
}

// Send start signal to DHT22 sensor
void DHT22_send_start(void) {
	ioport_set_pin_mode(DHT_PIN, IOPORT_MODE_PULLUP);  // Enable pull-up resistor
	ioport_set_pin_dir(DHT_PIN, IOPORT_DIR_OUTPUT);  // Set pin as output
	ioport_set_pin_level(DHT_PIN, false);  // Pull pin LOW to send start signal
	_delay_ms(18);  // Delay 18ms as per DHT22 start signal requirement

	ioport_set_pin_level(DHT_PIN, true);  // Set pin HIGH
	_delay_us(30);  // Wait for 20-40 microseconds

	ioport_set_pin_dir(DHT_PIN, IOPORT_DIR_INPUT);  // Set pin as input
}

// Check if DHT22 sensor responds (returns 0 if response is received, -1 otherwise)
int DHT22_check_response(void) {
	uint16_t timeout_counter = 0;

	while (ioport_get_pin_level(DHT_PIN)) {
		timeout_counter++;
		if (timeout_counter > DHT_TIMEOUT) {
			return -1;  // Timeout, no response
		}
	}
	_delay_us(80);  // Wait for the sensor's low signal

	timeout_counter = 0;
	while (!ioport_get_pin_level(DHT_PIN)) {
		timeout_counter++;
		if (timeout_counter > DHT_TIMEOUT) {
			return -1;  // Timeout, no response
		}
	}
	_delay_us(80);  // Wait for the sensor's high signal

	return 0;  // Successfully received response from DHT22
}

// Read a byte from the DHT22 sensor
uint8_t DHT22_read_byte(void) {
	uint8_t byte = 0;
	for (int i = 0; i < 8; i++) {
		while (!ioport_get_pin_level(DHT_PIN));  // Wait for the pin to go high
		_delay_us(30);  // Wait for 30us
		if (ioport_get_pin_level(DHT_PIN)) {
			byte |= (1 << (7 - i));  // Read a '1' if pin is still high
		}
		while (ioport_get_pin_level(DHT_PIN));  // Wait for the pin to go low again
	}
	return byte;
}

// Read data from the DHT22 sensor (returns 0 on success, -1 on failure)
int DHT22_read_data(uint16_t *temperature) {
	uint8_t humidity_high, humidity_low, temp_high, temp_low, checksum;

	// Read 5 bytes of data from the sensor
	humidity_high = DHT22_read_byte();
	humidity_low = DHT22_read_byte();
	temp_high = DHT22_read_byte();
	temp_low = DHT22_read_byte();
	checksum = DHT22_read_byte();

	// Verify checksum
	uint8_t sum = humidity_high + humidity_low + temp_high + temp_low;
	if (sum != checksum) {
		return -1;  // Checksum mismatch
	}

	// Combine high and low bytes of temperature
	*temperature = ((temp_high & 0x7F) << 8) | temp_low;

	// If the temperature is negative, the highest bit of the high byte is set
	if (temp_high & 0x80) {
		*temperature = -(*temperature);  // Negative temperature
	}

	return 0;  // Successfully read data
}

// Draw a string on the GFX monochrome LCD at the specified position
void gfx_mono_draw_string_on_lcd(const char *str, uint8_t x, uint8_t y) {
	gfx_mono_draw_string(str, x, y, &sysfont);  // Display string on the Monochrome LCD
}

// Clear the GFX monochrome LCD screen
void gfx_mono_clear_screen(void) {
	gfx_mono_draw_filled_rect(0, 0, GFX_MONO_LCD_WIDTH, GFX_MONO_LCD_HEIGHT, GFX_PIXEL_CLR);  // Clear the screen
}