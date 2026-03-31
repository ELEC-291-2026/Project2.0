// Controller.mk: Code for project 2 controller 
//
// By:  Team C13
//

#include <EFM8LB1.h>
#include <stdio.h>

#define SYSCLK      72000000L  // SYSCLK frequency in Hz
#define BAUDRATE      115200L  // Baud rate of UART in bps
#define SARCLK 		18000000L

#define LED_SIGNAL P1_7
#define LED_SENSOR P1_6
#define JOYSTICK_SW P2_0
#define BUTTON_1 P3_1
#define BUTTON_2 P1_6
#define BUTTON_3 P1_3

unsigned char overflow_count;

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;
	RSTSRC = 0x02;

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	P0MDOUT |= 0x10; // Enable UART0 TX as push-pull output
	P1MDOUT |= 0x80; // 0x80 is the bitmask for Pin 7 (1000 0000 in binary)
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00;
	XBR2     = 0x40; // Enable crossbar and weak pull-ups

	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
	
	return 0;
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}


// ------------------------------------------------------------------------------
// TIMER 2 INTERRUPTS
//
// ------------------------------------------------------------------------------

void TIMER2_Init(void)
{
    TMR2CN0 = 0x00;   // Stop Timer 2; Clear flags; 16-bit auto-reload
    CKCON0 &= ~0x30;  // Timer 2 uses SYSCLK/12
    
    // Set reload value for 1ms (65536 - 6000)
    TMR2RL = 0xE890; 
    TMR2 = TMR2RL;    // Initialize TMR2 with reload value

    ET2 = 1;          // Enable Timer 2 interrupts
    TR2 = 1;          // Start Timer 2
}

volatile int counterMS2 = 0;
volatile int quartSecFlag = 0;

void TIMER2_ISR(void) interrupt 5
{
    TF2H = 0;       // Clear Timer 2 high byte overflow flag
    counterMS2++;     // Increment your 1ms counter
    if(counterMS2 >= 145)
    {
    	quartSecFlag = 1;
    	counterMS2 = 0;
    }
}

// ------------------------------------------------------------------------------
// TIMER 0 INTERRUPTS
//
// ------------------------------------------------------------------------------

void TIMER0_Init(void)
{
    // Set Timer 0 to use SYSCLK (1:1) instead of SYSCLK/12
    SFRPAGE = 0x00;
    CKCON0 |= 0x04; // Bit 2 sets T0 to system clock

    TMOD &= 0xF0;   // Clear Timer 0 bits
    TMOD |= 0x01;   // 16-bit mode
    
    // Pre-load for 38kHz toggle (947 cycles)
    // 65536 - 947 = 64589 (0xFC4D)
    TH0 = 0xFC;
    TL0 = 0x4D;
    
    ET0 = 1; // Enable Timer 0 interrupt
    TR0 = 1; // Start Timer 0
}

volatile int overflow = 0;
volatile int buttonValue = 0;

void TIMER0_ISR(void) interrupt 1
{
	TH0 = 0xFC; 
	TL0 = 0x4D;
	if(overflow <= buttonValue*10)
	{
		LED_SIGNAL = !LED_SIGNAL;
		overflow++;
	}
	else 
		LED_SIGNAL = 0;
}


// ------------------------------------------------------------------------------
// ADC READER
//
// ------------------------------------------------------------------------------

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

void main (void) 
{
	// car var
	float counterMS = 0;
	float normalized[2] = {0,0};
	float translated_v[2] = {0,0};
	
	// control var
	float current_v[2] = {0,0};
	int alternatingVolt = 0;
	int counterNormalize = 0;
	
	TIMER0_Init();
	TIMER2_Init();
	InitADC();
	
	InitPinADC(2, 1); // Configure P2.2 as analog input
	InitPinADC(2, 2); // Configure P2.3 as analog input


	EA = 1;
	waitms(500);
	printf("\x1b[2J"); //clears screen


    while (1)
    {

    	/*if(JOYSTICK_SW == 0)
    	
    	{
    		buttonValue = 350;
    		counterMS2 = -500;
    		quartSecFlag = 0;
    		overflow = 0;
    		while(JOYSTICK_SW == 0){}
    	}*/
		if(BUTTON_1 == 0)
    	{
    		while(BUTTON_1 == 0){}
    		buttonValue = 200;
    		counterMS2 = -500;
    		quartSecFlag = 0;
    		overflow = 0;
    	}
    	else if(BUTTON_2 == 0)
    	{
    		while(BUTTON_2 == 0){}
    		buttonValue = 250;
    		counterMS2 = -500;
    		quartSecFlag = 0;
    		overflow = 0;
    	}
    	else if(BUTTON_3 == 0)
    	{
    		while(BUTTON_3 == 0){}
    		buttonValue = 300;
    		counterMS2 = -500;
    		quartSecFlag = 0;
    		overflow = 0;
    	}
    	/*	
	    if(LED_SENSOR == 0)
	    {
	        counterMS = 0;
	        quartSecFlag = 0;
	        while(LED_SENSOR == 0)
	        {
	            counterMS++;
	            Timer3us(10);
	        }
	        
	        if(counterMS <= 500) {}
	        else if(counterMS <= 1200)
	        {
	        	translated_v[0] = counterMS - normalized[0];
	        	
	        	if(translated_v[0] >=118)
	        		translated_v[0] = 120;
	        	else if(translated_v[0] <= -118)
	        		translated_v[0] = -120;
	        	else if(translated_v[0] <= 3 && translated_v[0] >= -3)
	        		translated_v[0] = 0;
	        }
	        else if(counterMS <=  2000)
	        {
	        	translated_v[1] = counterMS - normalized[1];
	        	
	        	if(translated_v[1] >=118)
	        		translated_v[1] = 120;
	        	else if(translated_v[1] <= -118)
	        		translated_v[1] = -120;
	        	else if(translated_v[1] <= 3 && translated_v[1] >= -3)
	        		translated_v[1] = 0;
	        }
	        else if(counterMS <=  3400)
	        {
	        	printf("\rAUTOMODE                               ");
	        	goto end;
	        }
	        
	        else if(counterMS <=  4000)
	    	{
	    		counterNormalize = 3;
	    	}
	        
	        if(counterNormalize > 0)
	    	{
	    		counterNormalize--;
	    		if(counterMS <= 1200)
					normalized[0] = counterMS;
				else if(counterMS <= 2000)
					normalized[1] = counterMS;
	    	}
	        
	    	
	    	
	        printf("\r%5.2f ms %.0f x, %.0f y               ", counterMS/100, translated_v[0], translated_v[1]);
	    }
	    */
	    
	    
	    if(quartSecFlag == 1)
	    {
	    	overflow = 0;
	    	quartSecFlag = 0;
	    	if(alternatingVolt == 0)
	    	{
	    		current_v[0] = Volts_at_Pin(QFP32_MUX_P2_2);
	    		alternatingVolt = 1;
	    		buttonValue = 25+10*current_v[0];
	    	}
	    	else if(alternatingVolt == 1)
	    	{
				current_v[1] = Volts_at_Pin(QFP32_MUX_P2_1);
				alternatingVolt = 0;
				buttonValue = 100+10*current_v[1];
			}
			
			
	    }
	    
	    end:
		
    }
	
}


 
