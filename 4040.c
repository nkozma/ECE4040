/*
 * 4040.c
 *
 *  Created on: Mar 7, 2018
 *      Author: ryanm
 */

#include "fsl_device_registers.h"
#include "4040.h"

/*
Authors: Nicholas Kozma, Ryan Main		Date: 07/03/2018
Notes:
	Initializes up FTM to be used as a timer
*/
void FTM_init(void)
{
	/*Enable port B, multiplex the timer into port B*/
	SIM_SCGC5 |= 0x01<<10; //enable port B
	PORTB_PCR18 &= ~(PORT_PCR_MUX(7));  //multiplex the FTM0_CH0 to port B18
	PORTB_PCR18 |= PORT_PCR_MUX(3);
	/*Turn on the timer*/
	SIM_SCGC6 |= 0x01<<26;

	/*Set up the timer*/
	FTM2_C0SC|=(0x01<<4); //set channel 0 of the flex timer module to output compare mode
	FTM2_C0SC|=(0x01<<2); //set C0 of FTM0 to toggle on match

	FTM2_MODE|=0x1<<2; //disable write protection
	FTM2_SC|=FTM_SC_CLKS(0x1);
	FTM2_SC|=FTM_SC_PS(0x7); //set the readings to be taken from the system clock (50 MHz) at 128 division
}

/*
Author: Nicholas Kozma, Ryan Main		Date: 07/03/2018
Inputs: time to delay in microseconds
Outputs: success(0)/error(1) state
Notes:
	Algorithm uses the system clock (21 MHz) this must be defined
with the name "CLOCKF" at some point as 50000000
	The algorithm reads once every 128 periods, meaning that
the minimum time detectable is 2.56 micro seconds
	Maximum time is ~0.1677696 seconds
	Outputs a toggling waveform for testing purposes from pin A2
*/
int FTM_delay(int t)
{
	int uCount;
	uCount=(int)t*CLOCKF/128; //convert the time to wait to a usable form

	if(uCount>0xffff)
	{
		return(1); //return error, out of range
	}

	FTM2_CNT=0; // clear the counter, not required if 1st init
	FTM2_C0V=uCount; //set toggle match to converted input value

	while(uCount>=FTM2_CNT); //wait for duration

	return(0); //success!
}

/*
Author: Ryan Main		Date: 26/03/2017
DAC initialization
Initializes the DAC to be output to DAC0.
*/
void DAC0_init(void)
{
	SIM_SCGC2 |= SIM_SCGC2_DAC0_MASK; //enable DAC0
	DAC0_C0 |= DAC_C0_DACEN_MASK; //enable DAC0
	//DAC0_C0 |= DAC_C0_DACTRGSEL_MASK; //select software trigger for DAC
	DAC0_C0 |= DAC_C0_DACRFS_MASK; //select DACREF_2
}

/*
Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
Initializes the RTC. Turns on crystal oscillator
*/
void RTC_init(void)
{
	int n=0;
	SIM_SCGC6|=SIM_SCGC6_RTC_MASK; //enable RTC output
	RTC_CR|=RTC_CR_OSCE_MASK; //turn on oscillator
	while(n<50){n=n+1;} //wait for startup
	RTC_SR|=RTC_SR_TCE(1); //reset TCE flag
	RTC_IER&=~(0x7); // turn off interrupts
	RTC_TSR=1; //reset TSR to reset TIF flag
}

/*
Author: Nicholas Kozma	Date: 26/03/2017
Resets RTC TSR to 0. Re-initializes the RTC to same settings as init
*/
void RTCreset(void)
{
	int n=0;

	RTC_CR|=1; //software reset
	RTC_CR&=~(0x1);
	RTC_CR|=RTC_CR_OSCE_MASK; //enable oscillator
	while(n<50){n=n+1;}
	RTC_TSR=1; //reset TIF
	RTC_SR|=RTC_SR_TCE(1); //reset TCE flag
	RTC_IER&=~(0x7); //disable interrupts
}

/*
Author: Nicholas Kozma		Date: 26/03/2017
Reads RTC TSR register. Use to avoid reading invalid times. Returns time as an integer
*/
int RTC_rtime(void)
{
	int timet1,timet2; //times for comparison
	int flag=0; //flag for equivalence
	while (flag==0)
	{
		timet1=RTC_TSR;
		timet2=RTC_TSR;
			if(timet1==timet2)
			{
				flag=1;
			}
	}
	return(timet2);
}

/*
Author: Nicholas Kozma		Date: 26/03/2017
Waits for the elapsed number of seconds (twait). Uses the RTC module
*/
void RTC_wait(int twait)
{
	int tw=twait+1; //wait for the elapsed time
	RTCreset(); //prepare RTC
	while (RTC_rtime() <tw); //wait
}

/*Author: Nicholas Kozma, Ben West	Date: 09/02/2018
Initializes the UART0 for 9600 baud rate and no parity. Used for terminal interfacing*/
 void UART0_Interface_Init()
 {
 	SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
 	SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;

 	PORTB_PCR16 |= PORT_PCR_MUX(3); //rx
 	PORTB_PCR17 |= PORT_PCR_MUX(3); //tx

 	UART0_C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);

 	UART0_C1=0x00;

 	UART0_BDH=0;
 	UART0_BDL=0x88;

 	UART0_C2 |= UART_C2_TE_MASK;
 	UART0_C2 |= UART_C2_RE_MASK;
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Place a single character on the terminal from a keyboard. Must be fed a character
 */
 void UART0_Putchar(char x)
 {
 	while(!(UART0_S1 & UART_S1_TDRE_MASK));

 	UART0_D = x;
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Uses UART4 to iteratively put all the characters in a string to a terminal. Input an array terminated with â€œ/0â€�.
 */
 void UART0_Putstring(char x[])
 {
 	int n=0;

 	while(x[n]!='\0')
 	{
 		UART0_Putchar(x[n]);
 		n=n+1;
 	}
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Receives a character from a keyboard or other attachment. Returns the character received.
 */
 char UART0_Getchar(void)
 {
 	char x;

 	while(!(UART0_S1 & UART_S1_RDRF_MASK));

 	x=UART0_D;
 	return x;
 }

 /*
 Author: Nicholas Kozma, Ben West	Date: 09/02/2018
 Initializes the UART1 for 9600 baud rate and no parity.
 Sets: PTC3=RX PTC4=TX
 */
 void UART1_Interface_Init(void)
 {
 	SIM_SCGC4 |= SIM_SCGC4_UART1_MASK;
 	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

 	PORTC_PCR3 |= PORT_PCR_MUX(3);
 	PORTC_PCR4 |= PORT_PCR_MUX(3);

 	UART1_C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);

 	UART1_C1=0x00;

 	UART1_BDH=0;
 	UART1_BDL=0x88;

 	UART1_C2 |= UART_C2_TE_MASK;
 	UART1_C2 |= UART_C2_RE_MASK;
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Place a single character on the terminal from a keyboard. Must be fed a character
 */
 void UART1_Putchar(char x)
 {
 	while(!(UART1_S1 & UART_S1_TDRE_MASK));

 	UART1_D = x;
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Receives a character from a keyboard or other attachment. Returns the character received.
 */
 char UART1_Getchar(void)
 {
 	char x;

 	while(!(UART1_S1 & UART_S1_RDRF_MASK));

 	x=UART1_D;
 	return x;
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Uses UART4 to iteratively put all the characters in a string to a terminal. Input an array terminated with â€œ/0â€�.
 */
 void UART1_Putstring(char x[])
 {
 	int n=0;

 	while(x[n]!='\0')
 	{
 		UART1_Putchar(x[n]);
 		n=n+1;
 	}
 }

 /*
 Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
 Initialize the switch for interrupts and GPIO input. Additionally, starts LED.
 */
 void HBridgeDriver(void)
 {
 	SIM_SCGC5 |= SIM_SCGC5_PORTD_MASK;
 	SIM_SCGC5 |= SIM_SCGC5_PORTC_MASK;

 	PORTD_PCR2 |= PORT_PCR_MUX(1);
 	PORTD_PCR1 |= PORT_PCR_MUX(1);
 	PORTD_PCR3 |= PORT_PCR_MUX(1);

 	PORTC_PCR0 |= PORT_PCR_MUX(1);
 	PORTC_PCR1 |= PORT_PCR_MUX(1);

 	GPIOD_PDDR |= 0x0E; //set port D1, D2, and D3 to output
 	GPIOC_PDDR |= 0x03; //set port C0, C1 to output
 }

 /*
 Author:Ryan Main		Date: 26/02/2018
 Converts byte array to floating point value
 */
 float byteArrayToFloat(char c[])
 {
 	float *f;

 	f = c;

 	return *f;
 }

 /*
  Author:Ryan Main		Date: 26/02/2018
  Converts integer to string representation
 */
 int intToStr(int x, char c[], int i){
 	char temp;

 	if(x > 0){
 		while(x > 0){
 			c[i++] = (x % 10) + '0';
 			x = x / 10;
 		}
 		if(i > 1){
 			temp = c[i-2];
 			c[i-2] = c[i-1];
 			c[i-1] = temp;
 		}
 	}
 	else{
 		x = -x;
 		c[i++] = '-';
 		while(x > 0){
 			c[i++] = (x % 10) + '0';
 			x = x / 10;
 		}
 		if(i > 2){
 			temp = c[i-2];
 			c[i-2] = c[i-1];
 			c[i-1] = temp;
 		}
 	}

 	return i;
 }

 /*
  Author:Ryan Main		Date: 26/02/2018
  Converts float to string representation
  */
 void ftoa(char c[], float f){
 	int iPart = (int)f;
 	float fPart = f - (float)iPart;
 	int i = 0;

 	i = intToStr(iPart, c, i);

 	c[i++] = '.';

 	if(fPart > 0){
 		fPart = fPart * 100;
 	}
 	else{
 		fPart = -fPart * 100;
 	}

 	i = intToStr((int)fPart, c, i);

 	c[i++] = '\0';
 }

 int DAC_DeadZone_Check(int* DACout)
 {
     if(*DACout<DAC_MIN_VOLTAGE)
     {
         *DACout=DAC_MIN_VOLTAGE;
         return 1; //return the error that we know is caused by the requirement to enter the deadzone.
     }
     return 0;
 }

 void safetyDACOut(int* output)
 {
 	int deadZoneFlag = 0;

 	if(*output < 0){
 		*output = -1* *output;
 	}

	if (*output > 4095||*output<-4095)
 	{
 		*output = 4095;
 	}

 	deadZoneFlag = DAC_DeadZone_Check(output);
 	if(deadZoneFlag)
 	{
 		//send error message to GUI
 	}
 }

 int convertVotagetoDAC(double Vout)
 {
 	int DACOut = (int)(Vout * DAC_SIZE / TTL_V);

 	if ((Vout * DAC_SIZE / TTL_V) - DACOut > 0.5)
 	{
 		DACOut = DACOut + 1;
 	}

 	return DACOut;
 }

 void getField(char x[])
 {
 	x[0] = UART1_Getchar();
 	x[1] = UART1_Getchar();
 	x[2] = UART1_Getchar();
 	x[3] = UART1_Getchar();
 }
