 /*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "fsl_device_registers.h"

#define TTL_V 3.3
#define DAC_SIZE 4096
#define TO_MICRO_TESLA 1000000
#define CLOCKF 21 //MHz

/*
Authors: Nicholas Kozma, Ryan Main		Date: 07/03/2018
Notes:
	Sets up FTM
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
	RTC_TSR=1; //reset TSR to reset TIF flag
	RTC_SR|=RTC_SR_TCE(1); //reset TCE flag
	RTC_IER&=~(0x7); // turn off interrupts
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

 void UART0_Putchar(char x)
 {
 	while(!(UART0_S1 & UART_S1_TDRE_MASK));

 	UART0_D = x;
 }

 void UART0_Putstring(char x[])
 {
 	int n=0;

 	while(x[n]!='\0')
 	{
 		UART0_Putchar(x[n]);
 		n=n+1;
 	}
 }

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
void UART1_Interface_Init()
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
Uses UART4 to iteratively put all the characters in a string to a terminal. Input an array terminated with “/0”.
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

	PORTD_PCR2 |= PORT_PCR_MUX(1);
	PORTD_PCR1 |= PORT_PCR_MUX(1);
	PORTD_PCR3 |= PORT_PCR_MUX(1);

	GPIOD_PDDR |= 0x0E; //set port D1, D2, and D3 to output
}


float byteArrayToFloat(char c[]){
	float *f;

	f = c;

	return *f;
}

/*void I2C_init(void){
	SIM_SCGC4 |= SIM_SCGC4_I2C0_MASK;

	PORTE_PCR24 |= PORT_PCR_MUX(5); //E24 = SCL
	PORTE_PCR25 |= PORT_PCR_MUX(5); //E25 = SDA

	I2C0_C1 |= I2C_C1_IICEN_MASK;
	I2C0_C1 |= I2C_C1_MST_MASK;

	I2C0_A1 = 0xC6 << 1;	//sets slave address
	I2C0_F = 0x14; 			//sets frequency
}*/

/*
Author: Nicholas Kozma, Ryan Main		Date: 26/03/2017
Call all initialization modules
*/
void Init (void)
{
	UART1_Interface_Init(); //communication with MBed
	UART0_Interface_Init(); //communication with PC
	RTC_init();  //allows waiting
	DAC0_init(); //not used for test
	HBridgeDriver(); //not used for test
	FTM_init();
	init_I2C();
}

void safetyDACOut(int* output)
{
	if (*output > 4095||*output<-4095)
	{
		*output = 4095;
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

int axisController(double ki, double* integral, double ts, double SS, double Bn[], int* state) {
	double out;
	int DACOut;
	*integral = *integral + (ts*(SS - (Bn[1] + Bn[0]) / 2));
	out = ((*integral)*ki);
	DACOut = convertVotagetoDAC(out);

	if (DACOut < 0)
	{
		DACOut = -1 * DACOut;
		if ((*state != 1))
		{
			*state = 1;
			GPIOD_PTOR|=0x6;
		}
	}
		else
		{
		if (*state != 0)
		{
			*state = 0;
			GPIOD_PTOR|=0x6;
		}
	}

	safetyDACOut(&DACOut);

	return DACOut;
	}

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
		if(i > 1){
			temp = c[i-2];
			c[i-2] = c[i-1];
			c[i-1] = temp;
		}
	}

	return i;
}

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

void getField(char x[]){
	x[0] = UART1_Getchar();
	x[1] = UART1_Getchar();
	x[2] = UART1_Getchar();
	x[3] = UART1_Getchar();
}

int get_Directions(double reading,double goal)
{
	if ((goal-reading)<0)
		{
			GPIOD_PTOR|=0x6;
			return 1;
		}		
	else
		{
			return 0;
		}
}


int main(void)
{
	double kix = 8450000;
	double integralx = 0;
	double ts = (double)1/(double)1000; //sample time
	double xaxis[2];
	double SSx = 0/TO_MICRO_TESLA; //desired steady state x in this case 35 uT.
	int DACOUTx = 0;
	int statex = 0; //current output state 0=+x, 1=-x
	int FTM_Flag;

	char x[4], y[4], z[4];
	char fieldx[10], fieldy[10], fieldz[10];
	float fx, fy, fz;
	Init();

	DAC0_DAT0L = (0xFF);
	DAC0_DAT0H = (0xF);

	RTC_wait(5);

	GPIOD_PTOR|=0x4;

	UART1_Putchar('1');
	getField(x);
	getField(y);
	getField(z);

	fx = byteArrayToFloat(x)/10;
	ftoa(fieldx, fx);

	fy = byteArrayToFloat(y)/10;
	ftoa(fieldy, fy);

	fz = byteArrayToFloat(z)/10;
	ftoa(fieldz, fz);

	fx=fx/TO_MICRO_TESLA;
	fy=fy/TO_MICRO_TESLA;
	fz=fz/TO_MICRO_TESLA;

	xaxis[0]=fy;
	statex=get_Directions(xaxis[0],SSx);
	
	while(1){
		UART1_Putchar('1');
		getField(x);
		getField(y);
		getField(z);

		fx = byteArrayToFloat(x)/10;
		ftoa(fieldx, fx);

		fy = byteArrayToFloat(y)/10;
		ftoa(fieldy, fy);

		fz = byteArrayToFloat(z)/10;
		ftoa(fieldz, fz);

		UART0_Putstring("X-Field: ");
		UART0_Putstring(fieldx);
		UART0_Putstring("uT, ");

		UART0_Putstring("Y-Field: ");
		UART0_Putstring(fieldy);
		UART0_Putstring("uT ");

		UART0_Putstring("Z-Field: ");
		UART0_Putstring(fieldz);
		UART0_Putstring("uT\n\n\r");

		fx=fx/TO_MICRO_TESLA;
		fy=fy/TO_MICRO_TESLA;
		fz=fz/TO_MICRO_TESLA;

		xaxis[1]=fy;

		DACOUTx=axisController(kix, &integralx, ts, SSx, xaxis, &statex);
		DAC0_DAT0L = (DACOUTx & 0x0FF);
		DAC0_DAT0H = ((DACOUTx & 0x0F00)>>8);

		xaxis[0]=xaxis[1];

		FTM_Flag = FTM_delay(1000);
	}

  	return 0;
}
