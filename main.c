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
#include "i2c.h"
#include "4040.h"

void RTC_alarm_init(int alarm)
{
    int n=0;
    SIM_SCGC6|=SIM_SCGC6_RTC_MASK; //enable RTC output
    RTC_CR|=1; //software reset
    RTC_CR&=~(0x1);
    RTC_TSR=1; //reset TSR to reset TIF flag
    RTC_CR|=RTC_CR_OSCE_MASK; //enable oscillator
    while(n<50){n=n+1;}
    RTC_TSR=1; //reset TSR to reset TIF flag
    RTC_IER&=~(0x7); // turn off interrupts
    RTC_TAR=alarm;
    RTC_SR|=RTC_SR_TCE(1); //reset TCE flag
}


void readFields(double axisReading[][2], int N)
{
	float fx, fy, fz;
	char x[4], y[4], z[4];

	UART1_Putchar('1');
	getField(x);
	getField(y);
	getField(z);

	fx = byteArrayToFloat(x)/(10*TO_MICRO_TESLA);
	fy = byteArrayToFloat(y)/(10*TO_MICRO_TESLA);
	fz = byteArrayToFloat(z)/(10*TO_MICRO_TESLA);

	axisReading[0][N] = (double)fx;
	axisReading[1][N] = (double)fy;
	axisReading[2][N] = (double)fz;
}

/*
 * State 0 means IN12=10 is positive direction
 * State 1 means IN12=01 is positive direction
 * */
void Direction_Determination(double field [][2], int state[])
{
	double test[3][2];

	for (int i=0;i<3;i++){
		test[i][0] = field[i][0];
		test[i][1] = field[i][1];
	}

	int i=0;
	for(i=0;i<3;i++){
	if(field[i][0]<field[i][1])
	{
		state[i]=1;
	}
	else
	{
		state[i]=0;
	}
	}
}

void Direction_Set(double field [][2], int state[], double SS[])
{
	int i;

	double test[3][2];

	for (int i=0;i<3;i++){
		test[i][0] = field[i][0];
		test[i][1] = field[i][1];
	}

	i=0;

	//for(i=0;i<3;i++)
	//{
		if(state[i]==1)
		{
			if(SS[i]<field[i][0])
			{
				GPIOD_PCOR|=0x2;
				GPIOD_PSOR|=0x4;
			}
			else if(SS[i]<field[i][1])
			{
				//error messages for deadzone
			}
			else
			{
				GPIOD_PCOR|=0x4;
				GPIOD_PSOR|=0x2;
			}
		}
		else
		{
			if(SS[i]>field[i][0])
			{
				GPIOD_PCOR|=0x2;
				GPIOD_PSOR|=0x4;
			}
			else if(SS[i]>field[i][1])
			{
				//error message for deadzone
			}
			else
			{
				GPIOD_PCOR|=0x4;
				GPIOD_PSOR|=0x2;
			}
		}
	//}
}

void System_init(double startupReadings[][2], int state[], double SS[]){

	double test[3][2];

	DAC0_DAT0L=(0x2F);
	DAC0_DAT0H=(0x9);
	GPIOD_PCOR|=0x2;
	GPIOD_PSOR|=0x4;

	FTM_delay(30000);
	readFields(startupReadings,0);

	GPIOD_PCOR|=0x4;
	GPIOD_PSOR|=0x2;
	FTM_delay(30000);
	readFields(startupReadings, 1);

	for (int i=0;i<3;i++){
		test[i][0] = startupReadings[i][0];
		test[i][1] = startupReadings[i][1];
	}

	Direction_Determination(startupReadings,state);
	Direction_Set(startupReadings,state,SS);
}

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

int axisController(double ki, double* integral, double ts, double SS, double Bn[], int* state) {
	double out;
	int DACOut;
	*integral = *integral + (ts*(SS - (Bn[1] + Bn[0]) / 2));
	out = ((*integral)*ki);
	DACOut = convertVotagetoDAC(out);

	/*if (DACOut < 0)
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
	}*/

	safetyDACOut(&DACOut);

	return DACOut;
}

void geterror(double err[], double axisreading[][2])
{
	int i;
	for(i=0; i<3 ;i++)
	{
		err[i] = axisreading[i][1]-axisreading[i][0];
	}
}


void getStates(double reading[][2], double goal[], int state[]){
	state[0]=get_Directions(reading[0][0],goal[0]);
	state[1]=get_Directions(reading[1][0],goal[1]);
	state[2]=get_Directions(reading[2][0],goal[2]);
}

void displayFields(double reading[][2], int N){
	char fieldx[10], fieldy[10], fieldz[10];
	float f[3];

	for(int i = 0; i < 3; i++){
		f[i] = (float)reading[i][N];
	}

	ftoa(fieldx, f[0]*TO_MICRO_TESLA);
	ftoa(fieldy, f[1]*TO_MICRO_TESLA);
	ftoa(fieldz, f[2]*TO_MICRO_TESLA);

	UART0_Putstring("X-Field: ");
	UART0_Putstring(fieldx);
	UART0_Putstring("uT, ");

	UART0_Putstring("Y-Field: ");
	UART0_Putstring(fieldy);
	UART0_Putstring("uT ");

	UART0_Putstring("Z-Field: ");
	UART0_Putstring(fieldz);
	UART0_Putstring("uT\n\n\r");
}

int main(void)
{
	double ki[] = {8450000,8450000,8450000};
	double integral[] = {0,0,0};
	double ts = (double)1/(double)1000; //sample time
	double axisreading[3][2];
	double goal[3] = {(double)70/(double)TO_MICRO_TESLA,0,0}; //desired steady state x in this case 35 uT.
	int DACOUT[3]={0,0,0};
	int state[3]={0,0,0}; //current output state 0=+x, 1=-x
	double err[3];
	double startupReadings[3][2];
	int FTM_Flag;

	Init();

	/*DAC0_DAT0L=(0x2F);
	DAC0_DAT0H=(0x9);
	GPIOD_PCOR|=0x2;
	GPIOD_PSOR|=0x4;

	FTM_delay(15000);
	readFields(axisreading, 0);

	GPIOD_PCOR|=0x4;
	GPIOD_PSOR|=0x2;
	FTM_delay(15000);
	readFields(axisreading, 1);*/

	System_init(startupReadings, state, goal);
	RTC_wait(2);
	RTC_alarm_init(5);

	readFields(axisreading, 0);

	while(1)
	{
		readFields(axisreading, 1);

		displayFields(axisreading, 1);

		DACOUT[0]=axisController(ki[0], &integral[0], ts, goal[0], axisreading[0], &state[0]);

		DAC0_DAT0L = (DACOUT[0] & 0x0FF);
		DAC0_DAT0H = ((DACOUT[0] >> 8) & 0xF);

		axisreading[0][0]=axisreading[0][1];

		geterror(err,axisreading);

		FTM_Flag = FTM_delay(1000);
		if(FTM_Flag)
		{
			//send FTM error
		}

		if((RTC_SR&RTC_SR_TAF_MASK))
		{
			goal[0] = (double)-117/(double)TO_MICRO_TESLA;
			Direction_Set(startupReadings, state, goal);
		}
	}

	return 0;
}
