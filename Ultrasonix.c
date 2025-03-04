#include "TM4C123GH6PM.h"
#include <stdint.h>

//look from back of the car
//for left sensor
#define lTrig (1 << 7) //PA7
#define lEcho (1 << 6) //PA6

//for center sensor
#define cTrig (1 << 4) //PE4
#define cEcho (1 << 5) //PE5

//for right sensor
#define rTrig (1 << 1) //PE1
#define rEcho (1 << 2) //PE2

void delayUs(int us);
void delayMs(int ms);
void ultrasonicInit(void);
uint32_t measureDistance(uint32_t trig, uint32_t echo, GPIOA_Type* gpio);

//Functions for UART communication
void UART0_Init(void);
void UART0_SendChar(char c);
void UART0_SendString(char *str);
void UART0_SendNumber(uint32_t num);

int main(void){
	ultrasonicInit();
	
	while(1){
			uint32_t lDist = measureDistance(lTrig, lEcho, GPIOA);  //Distance for left sensor
			uint32_t cDist = measureDistance(cTrig, cEcho, GPIOE);  //Distance for center sensor
			uint32_t rDist = measureDistance(rTrig, rEcho, GPIOE);  //Distance for right sensor
		
		//DO something with Distance values.
		
	}
}

void ultrasonicInit(void){
	SYSCTL->RCGCGPIO |= (1 << 0) | (1 << 4);  //Enable clock for Port A and Port E
	while((SYSCTL->PRGPIO & ((1 << 0) | (1 << 4))) == 0);  //Wait until port A and E are ready after enabling clock
	
	GPIOA->DIR |= lTrig;  //PA7 as output
	GPIOA->DIR &= ~lEcho; //PA6 as input
	GPIOE->DIR |= cTrig | rTrig;  //PE4, PE1 as output
	GPIOE->DIR &= ~(cEcho | rEcho); //PE5, PE2 as input
	
	GPIOA->DEN |= lTrig | lEcho;
	GPIOE->DEN |= cTrig | cEcho | rTrig | rEcho;
}

uint32_t measureDistance(uint32_t trig, uint32_t echo, GPIOA_Type* gpio) {
    //Send trigger pulse
		gpio->DATA &= ~trig;  //Set TRIG pin LOW
		delayUs(2);           //Wait for 2 microseconds
		gpio->DATA |= trig;   //Set TRIG pin HIGH
		delayUs(10);          //Wait for 10 microseconds
		gpio->DATA &= ~trig;  //Set TRIG pin LOW

    //Wait for echo pulse
    while ((gpio->DATA & echo) == 0);
    uint32_t startTime = TIMER0->TAR;
    while (gpio->DATA & echo);
    uint32_t endTime = TIMER0->TAR;

    // Convert time to distance (assuming 16MHz clock)
    uint32_t pulseDuration = startTime - endTime;
    uint32_t distance = (pulseDuration * 0.0343) / 2;

    return distance;
}



void delayUs(int us) {
    int i;
    for (i = 0; i < (16 * us); i++);
}

void delayMs(int ms) {
    int i;
    for (i = 0; i < (16000 * ms); i++);
}