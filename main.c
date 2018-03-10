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
#include <stdlib.h>
#include <string.h>
int STAHP=0;

/*Author: Nicholas Kozma, Ben West  Date: 09/02/2018
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
    UART0_C2&=~(0x1<<UART_C2_RIE_SHIFT);
    UART0_C2 |= UART_C2_TE_MASK;
    UART0_C2 |= UART_C2_RE_MASK;
 }
 void UART0_Interface_INTERRUPT_Init()
 {
    SIM_SCGC4 |= SIM_SCGC4_UART0_MASK;
    SIM_SCGC5 |= SIM_SCGC5_PORTB_MASK;
    PORTB_PCR16 |= PORT_PCR_MUX(3); //rx
    PORTB_PCR17 |= PORT_PCR_MUX(3); //tx
    //PORTB_PCR16 = PORT_PCR_ISF_MASK|PORT_PCR_MUX(3);
    //PORTB_PCR17 = PORT_PCR_ISF_MASK|PORT_PCR_MUX(3);
    UART0_C2 &= ~(UART_C2_TE_MASK|UART_C2_RE_MASK);
    UART0_C1=0x00;
    UART0_BDH=0;
    UART0_BDL=0x88;
    UART0_C2|=0x1<<UART_C2_RIE_SHIFT;
    UART0_C5&=~(0x1<<UART_C5_RDMAS_SHIFT);
    UART0_RWFIFO=0x1;
    UART0_C2 |= UART_C2_TE_MASK;
    UART0_C2 |= UART_C2_RE_MASK;
    NVIC_EnableIRQ(UART0_RX_TX_IRQn);
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
 void UART0_Putstring(char x[], int length)
  {
      int i;
     for(i=0; i<length; i++)
      {
         UART0_Putchar(x[i]);
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

 void UART0_RX_TX_IRQHandler(void)
  {
      char info;
      info=UART0_Getchar();
      if(info=='S')
      {
     	 STAHP=1;
      }
  }

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
	int i=0;

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

	i++;

	if(state[i]==1)
			{
				if(SS[i]<field[i][0])
				{
					GPIOC_PCOR|=0x1;
					GPIOC_PSOR|=0x2;
				}
				else if(SS[i]<field[i][1])
				{
					//error messages for deadzone
				}
				else
				{
					GPIOC_PCOR|=0x2;
					GPIOC_PSOR|=0x1;
				}
			}
			else
			{
				if(SS[i]>field[i][0])
				{
					GPIOC_PCOR|=0x1;
					GPIOC_PSOR|=0x2;
				}
				else if(SS[i]>field[i][1])
				{
					//error message for deadzone
				}
				else
				{
					GPIOC_PCOR|=0x2;
					GPIOC_PSOR|=0x1;
				}
			}
}

void System_init(double startupReadings[][2], int state[])
{
	I2CWriteDAC(DAC_MIN_VOLTAGE,DAC_MIN_VOLTAGE);

	GPIOD_PCOR|=0x2;
	GPIOD_PSOR|=0x4;

	GPIOC_PCOR|=0x1;
	GPIOC_PSOR|=0x2;

	FTM_delay(30000);
	readFields(startupReadings,0);

	GPIOD_PCOR|=0x4;
	GPIOD_PSOR|=0x2;

	GPIOC_PCOR|=0x2;
	GPIOC_PSOR|=0x1;

	FTM_delay(30000);
	readFields(startupReadings, 1);

	Direction_Determination(startupReadings,state);
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
	//DAC0_init(); //not used for test
	HBridgeDriver(); //not used for test
	FTM_init();
	//init_I2C();
}

uint16_t axisController(double ki, double* integral, double ts, double SS, double Bn[], int* state) {
	double out;
	int DACOut;

	*integral = *integral + (ts*(SS - (Bn[1] + Bn[0]) / 2));
	out = ((*integral)*ki);
	DACOut = convertVotagetoDAC(out);

	safetyDACOut(&DACOut);

	return (uint16_t)DACOut;
}

void geterror(double err[], double axisreading[][2],double goal[])
{
	int i;
	for(i=0; i<3 ;i++)
	{
		err[i] = goal[i]-axisreading[i][1];
	}
}


void getStates(double reading[][2], double goal[], int state[]){
	state[0]=get_Directions(reading[0][0],goal[0]);
	state[1]=get_Directions(reading[1][0],goal[1]);
	state[2]=get_Directions(reading[2][0],goal[2]);
}

void data_r(double desired[])
{
    int i,j;
    char read[6];
    for(i=0;i<4;i++)
            {
            for(j=0; j<5; j++)
                {
                    read[j]=UART0_Getchar();
                }
            read[5]='\0';
            if(read[0]=='S')
            {
            	desired[0]=0;
            	desired[1]=0;
            	desired[2]=0;
            	desired[3]=0;
            	STAHP=1;
            	break;
            }
            desired[i]=atof(read);
            }
}

void error_s(double err[])
{
	 int c;
	 char to_print[5];
	 int i,j;
	 for(i=0;i<3;i++)
	 {
		 c=(int)err[i];
		 for(j=0;j<3;j++)
		 {
			 to_print[2-j]=(char)(c%10)+0x30;
			 c=c/10;
		 }

		 to_print[3]='\n';
		 to_print[4]='\r';

		 UART0_Putstring(to_print, 5);
	 }
}

int main(void)
{
	int looper=0;
	int itr;
	double ki[] = {8450000,8450000,8450000};
	double integral[] = {0,0,0};
	double ts = (double)1/(double)1000; //sample time
	double axisreading[3][2];
	double goal[4]; //desired steady state x in this case 35 uT.
	uint16_t DACOUT[3]={0,0,0};
	int state[3]={0,0,0}; //current output state 0=+x, 1=-x
	double err[3];
	double startupReadings[3][2];
	char TYPE[2];
	int FTM_Flag;

    TYPE[0]='b';
    TYPE[1]='b';

    char msngr[3];

    //system init
	Init();
	//I2CWriteDAC(0x0,0x0);

	//System_init(startupReadings, state);
	RTC_wait(2);

	//Gui stuff
	TYPE[0]=UART0_Getchar();
	if(TYPE[0]=='s')
		    {
		    	TYPE[1]=UART0_Getchar();
		    	looper=0; //repeated data read
		    }
		    else
		    {
		    	UART0_Getchar();
		        TYPE[1]='n';
		        looper=1; //repeated data read
		    }


	UART0_Interface_Init();
	msngr[0]='d';
	msngr[1]='\n';
	msngr[2]='\r';
	STAHP=0;

	while(1)
	{
		msngr[0]='d';
		UART0_Putstring(msngr, (int)3);
		data_r(goal);
		if(STAHP==1)
		{
			break;
		}
		RTC_alarm_init(goal[3]);
		msngr[0]='q';
		//start of control system
		//Direction_Set(startupReadings, state, goal);
		for(itr=0;itr<3;itr++)
		{
			integral[itr] = 0;
			err[itr] = 0;
		}
		//readFields(axisreading, 0);
		//looping crtl
	while(!(RTC_SR & RTC_SR_TAF_MASK))
	{
		//readFields(axisreading, 1);

		DACOUT[0]=axisController(ki[0], &integral[0], ts, goal[0], axisreading[0], &state[0]);
		DACOUT[1]=axisController(ki[1], &integral[1], ts, goal[1], axisreading[1], &state[1]);

		//I2CWriteDAC(DACOUT[0],DACOUT[1]);

		axisreading[0][0]=axisreading[0][1];
		axisreading[1][0]=axisreading[1][1];

		geterror(err,axisreading,goal);
		UART0_Putstring(msngr, 3);
		error_s(err);


		FTM_Flag = FTM_delay(1000);
		if(FTM_Flag)
		{
			//send FTM error
		}
		//crtl system end
	}
	}

	return 0;
}
