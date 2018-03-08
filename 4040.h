/*
 * 4040.h
 *
 *  Created on: Mar 7, 2018
 *      Author: ryanm
 */

#ifndef SOURCES_4040_H_
#define SOURCES_4040_H_

#define TTL_V 3.3
#define DAC_SIZE 4096
#define TO_MICRO_TESLA 1000000
#define CLOCKF 21 //MHz

void FTM_init(void);
int FTM_delay(int t);
void DAC0_init(void);
void RTC_init(void);
void RTCreset(void);
int RTC_rtime(void);
void RTC_wait(int twait);
void UART0_Interface_Init();
void UART0_Putchar(char x);
void UART0_Putstring(char x[]);
char UART0_Getchar(void);
void UART1_Interface_Init(void);
void UART1_Putchar(char x);
char UART1_Getchar(void);
void UART1_Putstring(char x[]);
void HBridgeDriver(void);
float byteArrayToFloat(char c[]);
int intToStr(int x, char c[], int i);
void ftoa(char c[], float f);
int DAC_DeadZone_Check(int* DACout);
void safetyDACOut(int* output);
int convertVotagetoDAC(double Vout);
void getField(char x[]);
int get_Directions(double reading,double goal);

#endif /* SOURCES_4040_H_ */
