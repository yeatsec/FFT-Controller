/*
 * LEDFFTController.c
 *
 * Eric Yeats
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define ADC_PIN	0
#define TIMER_CONTROL_A		0b10100011
#define TIMER_PRESCALE	0b00000101
#define MAX	0xFF
#define RED	OCR2B
#define GREEN	OCR0B
#define BLUE	OCR0A
#define LED_PIN	PB5
#define THRESHOLD_MAGNITUDE	8196
#define BUFFER_SIZE	128
#define HALF_BUFFER_SIZE	64
#define BITS	7
#define BUFFER_WIDTH	8

/* Function Declarations */
void iterative_fft(void);
uint8_t pow_of_two(uint8_t exponent);
void set_colors(void);

const float r_root[BUFFER_SIZE] PROGMEM = {1.0, 0.99879546, 0.99518473, 0.98917651, 0.98078528, 0.97003125, 0.95694034, 0.94154407, 0.92387953, 0.90398929, 0.88192126, 0.85772861, 0.83146961, 0.80320753, 0.77301045, 0.74095113, 0.70710678, 0.67155895, 0.63439328, 0.5956993, 0.55557023, 0.51410274, 0.47139674, 0.42755509, 0.38268343, 0.33688985, 0.29028468, 0.24298018, 0.19509032, 0.14673047, 0.09801714, 0.04906767, 0.0, -0.04906767, -0.09801714, -0.14673047, -0.19509032, -0.24298018, -0.29028468, -0.33688985, -0.38268343, -0.42755509, -0.47139674, -0.51410274, -0.55557023, -0.5956993, -0.63439328, -0.67155895, -0.70710678, -0.74095113, -0.77301045, -0.80320753, -0.83146961, -0.85772861, -0.88192126, -0.90398929, -0.92387953, -0.94154407, -0.95694034, -0.97003125, -0.98078528, -0.98917651, -0.99518473, -0.99879546, -1.0, -0.99879546, -0.99518473, -0.98917651, -0.98078528, -0.97003125, -0.95694034, -0.94154407, -0.92387953, -0.90398929, -0.88192126, -0.85772861, -0.83146961, -0.80320753, -0.77301045, -0.74095113, -0.70710678, -0.67155895, -0.63439328, -0.5956993, -0.55557023, -0.51410274, -0.47139674, -0.42755509, -0.38268343, -0.33688985, -0.29028468, -0.24298018, -0.19509032, -0.14673047, -0.09801714, -0.04906767, -0.0, 0.04906767, 0.09801714, 0.14673047, 0.19509032, 0.24298018, 0.29028468, 0.33688985, 0.38268343, 0.42755509, 0.47139674, 0.51410274, 0.55557023, 0.5956993, 0.63439328, 0.67155895, 0.70710678, 0.74095113, 0.77301045, 0.80320753, 0.83146961, 0.85772861, 0.88192126, 0.90398929, 0.92387953, 0.94154407, 0.95694034, 0.97003125, 0.98078528, 0.98917651, 0.99518473, 0.99879546};
const float i_root[BUFFER_SIZE] PROGMEM = {0.0, 0.04906767, 0.09801714, 0.14673047, 0.19509032, 0.24298018, 0.29028468, 0.33688985, 0.38268343, 0.42755509, 0.47139674, 0.51410274, 0.55557023, 0.5956993, 0.63439328, 0.67155895, 0.70710678, 0.74095113, 0.77301045, 0.80320753, 0.83146961, 0.85772861, 0.88192126, 0.90398929, 0.92387953, 0.94154407, 0.95694034, 0.97003125, 0.98078528, 0.98917651, 0.99518473, 0.99879546, 1.0, 0.99879546, 0.99518473, 0.98917651, 0.98078528, 0.97003125, 0.95694034, 0.94154407, 0.92387953, 0.90398929, 0.88192126, 0.85772861, 0.83146961, 0.80320753, 0.77301045, 0.74095113, 0.70710678, 0.67155895, 0.63439328, 0.5956993, 0.55557023, 0.51410274, 0.47139674, 0.42755509, 0.38268343, 0.33688985, 0.29028468, 0.24298018, 0.19509032, 0.14673047, 0.09801714, 0.04906767, 0.0, -0.04906767, -0.09801714, -0.14673047, -0.19509032, -0.24298018, -0.29028468, -0.33688985, -0.38268343, -0.42755509, -0.47139674, -0.51410274, -0.55557023, -0.5956993, -0.63439328, -0.67155895, -0.70710678, -0.74095113, -0.77301045, -0.80320753, -0.83146961, -0.85772861, -0.88192126, -0.90398929, -0.92387953, -0.94154407, -0.95694034, -0.97003125, -0.98078528, -0.98917651, -0.99518473, -0.99879546, -1.0, -0.99879546, -0.99518473, -0.98917651, -0.98078528, -0.97003125, -0.95694034, -0.94154407, -0.92387953, -0.90398929, -0.88192126, -0.85772861, -0.83146961, -0.80320753, -0.77301045, -0.74095113, -0.70710678, -0.67155895, -0.63439328, -0.5956993, -0.55557023, -0.51410274, -0.47139674, -0.42755509, -0.38268343, -0.33688985, -0.29028468, -0.24298018, -0.19509032, -0.14673047, -0.09801714, -0.04906767};
volatile const uint8_t bitflip[BUFFER_SIZE] PROGMEM = {0, 64, 32, 96, 16, 80, 48, 112, 8, 72, 40, 104, 24, 88, 56, 120, 4, 68, 36, 100, 20, 84, 52, 116, 12, 76, 44, 108, 28, 92, 60, 124, 2, 66, 34, 98, 18, 82, 50, 114, 10, 74, 42, 106, 26, 90, 58, 122, 6, 70, 38, 102, 22, 86, 54, 118, 14, 78, 46, 110, 30, 94, 62, 126, 1, 65, 33, 97, 17, 81, 49, 113, 9, 73, 41, 105, 25, 89, 57, 121, 5, 69, 37, 101, 21, 85, 53, 117, 13, 77, 45, 109, 29, 93, 61, 125, 3, 67, 35, 99, 19, 83, 51, 115, 11, 75, 43, 107, 27, 91, 59, 123, 7, 71, 39, 103, 23, 87, 55, 119, 15, 79, 47, 111, 31, 95, 63, 127};
volatile float real[BUFFER_SIZE];
volatile float imag[BUFFER_SIZE];

volatile float time[BUFFER_SIZE];
volatile uint8_t fft_flag;
volatile uint8_t time_index;
volatile uint8_t prim_freq;

int main(void)
{
	cli();
	/* Initialize Global Variables */
	fft_flag = 0;
	time_index = 0;
	prim_freq = 0;
	DDRB |= (1<<LED_PIN);
	/* Initialize ADC */
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADMUX |= (1<<REFS0);	/* Use 5V Reference */
	DIDR0 |= (1<<ADC0D);
	ADCSRA |= (1<<ADEN) | (1<<ADIE);	/* Enable ADC and enable Conversion Complete Interrupt */

	/* Initialize PWM */
	/* Output on PD3 (OC2B), PD5 (OC0B), PD6 (OC0A) */
	DDRD = (1<<3) | (1<<5) | (1<<6);
	TCCR0A = TIMER_CONTROL_A;
	TCCR2A = TIMER_CONTROL_A;
	TCCR0B = TIMER_PRESCALE;
	TCCR2B = TIMER_PRESCALE;
	/* Start off on */
	RED = MAX;
	GREEN = MAX;
	BLUE = MAX;
	sei();
	ADCSRA |= (1<<ADSC);	/* Initialize First Conversion */

    while (1) 
    {
		if (fft_flag)
		{
			iterative_fft();
			set_colors();
			fft_flag = 0;
		}
		if (prim_freq > 8)
		{
			PORTB = (1<<LED_PIN);
		} 
		else 
		{
			PORTB = 0;
		}
    }
	return 0;
}

/* ISR will fill time buffer, set fft_flag, and begin a new conversion */
ISR(ADC_vect)
{
	time[pgm_read_byte(&bitflip[time_index])] = ((float) ADC); /* cast from uint16_t to float (word to double word representation) */
	if (time_index < BUFFER_SIZE - 1)
	{
		++time_index;
	} 
	else /* time_index = BUFFER_SIZE - 1 */
	{
		time_index = 0;
		fft_flag = 1;
	}
	ADCSRA |= (1<<ADSC);
}

/************************************************************************/
/* 
 * iterative_fft()
 * computes a FFT of BUFFER_SIZE length and determines the largest
 * frequency content of the captured signal
*/
/************************************************************************/
void iterative_fft(void)
{
	/* Copy time into real and set imag to 0 - hopefully the first value in time will not change before copying */
	for (uint8_t i = 0; i < BUFFER_SIZE; ++i)
	{
		real[i] = time[i];
		imag[i] = 0.0;
	}
	for (uint8_t stage = 0; stage < BITS; ++stage)
	{
		uint8_t half_frame = pow_of_two(stage);
		uint8_t frame = 2 * half_frame;
		uint8_t rootSpacing = pow_of_two(BITS-1-stage);
		for (uint8_t a = 0; a < BUFFER_SIZE; a += frame)
		{
			/* Butterfly Calculation */
			for (uint8_t b = 0; b < half_frame; ++b)
			{
				uint8_t left_butterfly_index = a + b;
				uint8_t right_butterfly_index = left_butterfly_index + half_frame;
				uint8_t left_root_index = b * rootSpacing;
				uint8_t right_root_index = (b + half_frame) * rootSpacing;
				/* First Half of Butterfly Calculation */
				float real_left = real[left_butterfly_index] + (real[right_butterfly_index] * pgm_read_float(&r_root[left_root_index])) - (imag[right_butterfly_index] * pgm_read_float(&i_root[left_root_index]));
				float imag_left = imag[left_butterfly_index] + (real[right_butterfly_index] * pgm_read_float(&i_root[left_root_index])) + (imag[right_butterfly_index] * pgm_read_float(&r_root[left_root_index]));
				/* Second Half of Butterfly Calculation */
				float real_right = real[left_butterfly_index] + (real[right_butterfly_index] * pgm_read_float(&r_root[right_root_index])) - (imag[right_butterfly_index] * pgm_read_float(&i_root[right_root_index]));
				float imag_right = imag[left_butterfly_index] + (real[right_butterfly_index] * pgm_read_float(&i_root[right_root_index])) + (imag[right_butterfly_index] * pgm_read_float(r_root[right_root_index]));
				/* Update Component Buffers */
				real[left_butterfly_index] = real_left;
				imag[left_butterfly_index] = imag_left;
				real[right_butterfly_index] = real_right;
				imag[right_butterfly_index] = imag_right;
			}
		}
	}
	/* Search result of FFT for primary frequency signal (excluding DC bias and negative frequencies) */
	float max = 0;
	uint8_t max_index = 1;
	for(uint8_t i = 1; i < HALF_BUFFER_SIZE; ++i) /* start at 1 to ignore DC bias */
	{
		float real_comp = real[i];
		float imag_comp = imag[i];
		float mag2 = (real_comp * real_comp) + (imag_comp * imag_comp);
		if (mag2 > max)
		{
			max = mag2;
			max_index = i;
		}
	}
	if (max > THRESHOLD_MAGNITUDE)
	{
		prim_freq = max_index;
	}
}

uint8_t pow_of_two(uint8_t exponent)
{
	uint8_t ans = 1;
	for (uint8_t i = 0; i < exponent; ++i)
	{
		ans *= 2;
	}
	return ans;
}

void set_colors(void)
{
	/*
	if (prim_freq < 32)
	{
		RED = 0xFF;
		GREEN = (8 * prim_freq);
		BLUE = 0x00;
	}
	else
	{
		RED = 0xFF;
		GREEN = 0xFF;
		BLUE = (8 * (prim_freq - 32));
	}
	*/
	/*
	if (prim_freq < 15)
	{
		RED = 0xFF;
		GREEN = 17 * prim_freq;
		BLUE = 0x00;
	}
	else if (prim_freq < 30)
	{
		RED = 0xFF - (17 * (prim_freq-15));
		GREEN = 0xFF;
		BLUE = 0x00;
	}
	else if (prim_freq < 45)
	{
		RED = 0x00;
		GREEN = 0xFF;
		BLUE = ((prim_freq - 30) * 17);
	}
	else if (prim_freq < 60)
	{
		RED = 0x00;
		GREEN = 0xFF - ((prim_freq-45) * 17);
		BLUE = 0xFF;
	}
	else 
	{
		RED = (prim_freq - 60) * 20;
		GREEN = 0x00;
		BLUE = 0xFF;
	}
	*/
	if (prim_freq < 32)
	{
		RED = 0x00;
		GREEN = 8 * prim_freq;
		BLUE = 0xFF;
	}
	else
	{
		RED = 0x00;
		GREEN = 0xFF;
		BLUE = 255 - (8 * (prim_freq - 32));
	}
}
