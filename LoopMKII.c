

#define 	F_CPU   	128000UL
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

/*List of the pins
		
		PA0 - MENU pull to ground to activate
		PA1 - Pull high to activate +8.4 volt rail
		PA2 - Pull high to activate +3.3 volt rail
		PA3 - NC
		PA4 - ICSP CLK pad
		PA5 - ICSP MISO pad, Power button
		PA6 - ICSP MOSI pad
		PA7 - (ADC7) Battery pack voltage (shifted to 3.3V range)

		PB0 - NC
		PB1 - NC
		PB2 - VBUS
		PB3 - #RESET pin

	*/

enum {ON, OFF};
enum {NOCHANGE, CHANGE};

//Note: This interrupt is set to trigger eight times a second
ISR (TIM1_COMPA_vect) {

	static uint16_t button_count = 0;
	static uint16_t click_count = 0;
	static uint16_t click_countdown = 0;

	static uint8_t  state_flag = OFF;
	static uint8_t  modechange_flag = NOCHANGE;
	static uint8_t devmode_flag = OFF;
	static uint8_t startmenu=OFF;
	static uint16_t menucount=0;

	if (startmenu == ON)
	{	
		menucount++;
		if (menucount>20)
		{
			startmenu=OFF;
			menucount=0;
			PORTA |= _BV(PA0);
		}
	}

	if ((PINB & _BV(PB2)) == 0) {
		//USB cable isn't plugged in.

		//We need to differentiate between button held down and button clicked. We do this by counting how long the button has been held down.

		if ((PINA & _BV(PA5)) == 0) 
			{
			//If button line is LOW (i.e. it has been grounded by the button), do the following:

			button_count++;		//Count the number of interrupts that go by with the button held down

			if (button_count > 50) {	//3 second * 50 units per second = 150 'ticks'
				//Button held down for three seconds? Turn on/off the stabilizer.
				switch (state_flag) {
					case ON:	//Are we already on? Turn off.
						PORTA &= ~_BV(PA1);	//Turn 8.4V n-FET off (set bit low)
						PORTA &= ~_BV(PA2);	//Turn 3.3V n-FET off (set bit low)
						state_flag = OFF;

						//Reset some values while we're here. Probably not needed, but doing it anyway.
						devmode_flag = OFF;
						click_countdown = 0;
						click_count = 0;
						break;
					case OFF:	//Are we already off? TuRN IT UP! On. I mean, turn it on.
						PORTA |= _BV(PA1);	//Turn 8.4V n-FET on (set bit high)
						PORTA |= _BV(PA2);	//Turn 3.3V n-FET on (set bit high)
						state_flag = ON;
						break;
				}//End switch
				modechange_flag = CHANGE;	//Note that we changed mode.
				button_count = 0;			//Reset counter!	
			}//End button_count check

		} else {
			//Actually the button is not being pushed right now.

			/*We need to turn on Menu if the Button is clicked for a short time, so we need to ground PA0*/
			if (button_count>0)
			{
				PORTA &= ~_BV(PA0);		//Turn MENU On (set bit low)
				startmenu=ON;
				//click_count++;			//Make note that we had (another?) click
				//click_countdown = 100;		//About two seconds of interrupts; refresh each time there is a click
			}

			button_count = 0;			//Reset counter! No point in rolling this 
								//over till the next time the button is pressed. It'll throw off the count.
			modechange_flag = NOCHANGE;		//Now that the counter is zero, reset this flag so 
								//we can be ready to detect clicks on the next loop.
		}//End button state check

		


		/*//Firmware Update.
		if (click_count > 4) {
			//Turn on just the 3.3V power rail
			PORTA &= ~_BV(PA1);	//Turn 8.4V n-FET off (set bit low)
			PORTA |= _BV(PA2);	//Turn 3.3V n-FET on (set bit high)
			state_flag = ON;
			devmode_flag = ON;	//Make sure we block charging mode below.
		}

		//Dev Mode.
		if (click_count > 6) {
			//Turn on both power rails:
			PORTA |= _BV(PA1);	//Turn 8.4V n-FET on (set bit high)
			PORTA |= _BV(PA2);	//Turn 3.3V n-FET on (set bit high)
			state_flag = ON;
			devmode_flag = ON;	//Make sure we block charging mode below.
		}

		//Decrements click counter, and clear the click count at the end of the countdown
		if (click_countdown > 0) click_countdown--;
		if (click_countdown == 0) { click_count = 0; }

*/

	} else {
		//USB cable IS plugged in, so Charging Mode.
		if ((PINA & _BV(PA5)) == 0)
			{
				PORTA &= ~_BV(PA0);	//Turn MENU On (set bit low)
				startmenu=ON;
			}
		if (devmode_flag == OFF) {
			
			//Turn device Off while charging.
			//PORTA &= ~_BV(PA1);	//Turn 8.4V n-FET off (set bit low)
			//PORTA &= ~_BV(PA2);	//Turn 3.3V n-FET off (set bit low)

			//Reset the system state flag to OFF.
			//If the USB cable is disconnected, both internal power rails will be turned off.
			state_flag = OFF;

			//Reset the button timer counter system back to default/zero.
			button_count = 0;
			modechange_flag = NOCHANGE;

			//Clear timer interrupt flags - just set everything to zero
			//This prevents the system from immediately triggering again on the next tick.
			TIFR1 = 0x00;
		}//End Dev Mode check
	}//End USB connection check

}//End ISR
//--------------------------------------------------------------------------------------------------------------------------------------------------------------
		

int main (void) {

	
	/*By default, the device ships with the system clock fed by an 8 MHz clock
	that is divided by 8. Thus, we have a ~1 MHz clock.*/

	//Set up I/O ports -----------------------------------------------------------------------------------------------------------------------------------------

	/*The Tiny24A has two Ports (A, B) - eight pins on A, four on B.
		
		PA0 - MENU Pull to ground to activate
		PA1 - Pull high to activate +8.4 volt rail
		PA2 - Pull high to activate +3.3 volt rail
		PA3 - NC
		PA4 - ICSP CLK pad
		PA5 - ICSP MISO pad, Power button
		PA6 - ICSP MOSI pad
		PA7 - (ADC7) Battery pack voltage (shifted to 3.3V range)

		PB0 - NC
		PB1 - NC
		PB2 - VBUS
		PB3 - #RESET pin

		Note: Any bit in DRRn set to zero sets that pin as an input.
	*/

	DDRA  = 0b00000111;	
	PORTA = 0b00100001;	//Note the pull-up signal for PA5 and PA0

	DDRB  = 0b00000000;	
	PORTB = 0b00000000;	 

	//That's all the ports set up. -----------------------------------------------------------------------------------------------------------------------------


	//Turn on the ADC hardware and set various settings. -------------------------------------------------------------------------------------------------------

	//The ADC is enabled by setting the ADC Enable bit, ADEN in ADCSRA.
	ADCSRA |= _BV(ADEN);

	/*Set the clock.
		"By default, the successive approximation circuitry requires an input clock frequency between 50 kHz and 200 kHz to get maximum resolution."
		We're running at 128 kHz, so we need to turn off the divider.*/
	//By default the ADPS bits are set to zero. No need to change anything.

	/*The ADC voltage reference is selected by writing the REFS[1:0] bits in the ADMUX register.
		We're using VCC (3.3V) so no changes need to be made.

	/*The analog input channel and differential gain are selected by writing to the MUX bits in ADMUX.
		ADC7 (PA7) is the input from the voltage divider, and is single ended.*/
	ADMUX |= 0x07;	//b000111

	//Turn off the digital input buffer for ADC7 (PA7) - not strictly needed, but saves power
	DIDR0 |= _BV(ADC7D);

	/*Typically the result is right-adjusted and stored in ADCH & ADCL registers.
		In our case we don't need that level of fidelity, so we left-adjust by setting the ADLAR bit in ADCSRB,
		and just read out the contents of ADCH when a sample is taken.*/
	ADCSRB |= _BV(ADLAR);

	//End ADC Setup --------------------------------------------------------------------------------------------------------------------------------------------


	//Configure the internal clock system ----------------------------------------------------------------------------------------------------------------------

	/*In typical operation this microcontroller will be powered up at all times, either from the internal battery or from a USB cable.
	If the USB cable is not plugged in, it will monitor the Power/Status button.
		If the button is clicked, the microcontroller will measure the voltage of the battery pack and illuminate the status lights.
		If the button is held down for several second, the MCU will activate the internal power rails to power on the device.
	If the USB cable IS plugged in, the MCU will make sure all internal power rails are turned off and will start monitoring the status of the battery charging
	circuit.

	The Clock Source fuses are configured to select the 128 kHz internal RC oscillator. The DIV8 bit is unselected (divide by 8 scaler is off).

	The vast majority of the logic for this system is contained inside a timer-called interrupt service routine.*/
	

	//We're using Timer/Counter 1 (16 bit) to call the interrupt routine
	TCCR1B |= _BV(CS10);		//Use Clock Source b001 (CLKio/1, no prescale)
	TCCR1B |= _BV(WGM12);		//Enable Clear Timer on Compare (w/ ORC0A)
	
	//We're running at a 128 kHz, and we want about 50 interrupts a second:
	//128,000 / 50 = 2560, which works out to 0x0A00:
	OCR1A = 0x0A00;			//0.54 mA
	//OCR1A = 0x140;		//0.54 mA

	TIMSK1 |= _BV (OCIE1A);	// Enable T/C0 Output Compare A Match Interrupt

	//----------------------------------------------------------------------------------------------------------------------------------------------------------

	//----------------------------------------------------------------------------------------------------------------------------------------------------------

	//----------------------------------------------------------------------------------------------------------------------------------------------------------

	sei ();		//Turn on global interrupts

    //Sleep. Interrupts will handle everything else.
	
    for (;;)
	sleep_mode();

}//End firmware


