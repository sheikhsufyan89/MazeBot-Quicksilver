#include "TM4C123GH6PM.h"

void Delay_ms(int time_ms);    //function declarations till line 7
int duty_cycle = 4999;   //motors start from LOW state (0 speed) (reason: our logic is actually a count down logic, look line 63)
void PWM_init(void);
void LW_Turn_AntiClockwise(void);
void LW_Turn_Clockwise(void);
void RW_Turn_AntiClockwise(void);
void RW_Turn_Clockwise(void);

int main(void)
{
   
    	  PWM_init();
	
        LW_Turn_Clockwise();
		
    for(int i = 0;i<8;i++)
    {
        duty_cycle = duty_cycle - 500;  //speed up
        PWM1->_1_CMPA = duty_cycle;
        Delay_ms(500);
    }
		
		LW_Turn_AntiClockwise();
		for(int j = 0;j<8;j++)
    {
        duty_cycle = duty_cycle + 500; //slow down
        PWM1->_1_CMPA = duty_cycle;
        Delay_ms(500);
    }
		
		
		
		
}



//Spin motor in one direction by giving IN3 and IN4 signals to L298N
void LW_Turn_Clockwise(void)
{
		 SYSCTL->RCGCGPIO |= (1<<1);  /* enable clock to PORTB */
	   GPIOB->DIR |= (1<<6)|(1<<7); /* Make PB6 and PB7 an output pin*/
     GPIOB->DEN |=  (1<<6)|(1<<7);	/* Make PB6 and PAB7 an digitally enabled*/
     GPIOB->DATA |=	(1<<6);	/* IN3 Should be HIGH */    
		 GPIOB->DATA &=	~(1<<7);	/* IN4 Should be Low*/
}

//Spin motor in one direction by giving IN1 and IN2 signals to L298N
void RW_Turn_Clockwise(void)
{
		 SYSCTL->RCGCGPIO |= (1<<0);  /* enable clock to PORTA */
	   GPIOA->DIR |= (1<<2)|(1<<3); /* Make PA3 and PA2 an output pin*/
     GPIOA->DEN |=  (1<<2)|(1<<3);	/* Make PA3 and PA2 an digitally enabled*/
     GPIOA->DATA |=	(1<<2);	/* IN2 Should be HIGH */    
		 GPIOA->DATA &=	~(1<<3);	/* IN1 Should be Low*/
}




//Spin motor in other direction by giving IN3 and IN4 signals to L298N
void LW_Turn_AntiClockwise(void)
{
     SYSCTL->RCGCGPIO |= (1<<1);  /* enable clock to PORTB */
	   GPIOB->DIR |= (1<<6)|(1<<7);   /* Make PB6 and PAB7 an output pin*/
     GPIOB->DEN |= (1<<6)|(1<<7);	/* Make PB6 and PB7 an digitally enabled*/
     GPIOB->DATA |=	~(1<<6);	/* IN3 Should be LOW */    
		 GPIOB->DATA &=	(1<<7);	/* IN4 Should be HIGH*/}



//Spin motor in other direction by giving IN1 and IN2 signals to L298N
void RW_Turn_AntiClockwise(void)
{
     SYSCTL->RCGCGPIO |= (1<<0);  /* enable clock to PORTA */
	   GPIOA->DIR |= (1<<2)|(1<<3);   /* Make PA3 and PA2 an output pin*/
     GPIOA->DEN |= 	(1<<2)|(1<<3);	/* Make PA3 and PA2 an digitally enabled*/
     GPIOA->DATA |=	~(1<<2);	/* IN2 Should be LOW */    
		 GPIOA->DATA &=	(1<<3);	/* IN1 Should be HIGH*/}

		 
void PWM_init(void)
{
     /* Clock setting for PWM and GPIO PORT */
    SYSCTL->RCGCPWM |= (1<<1); /* Enable clock to PWM1 module Page 354 */
    SYSCTL->RCGCGPIO|= (1<<5); /* Enable system clock to PORTF. See page 406 */
	  SYSCTL->RCGCGPIO|= (1<<2); /* Enable system clock to PORTC.
	  SYSCTL->RCC |= (1<<20);    /* Enable System Clock Divisor function. We want to divide the clock with 64 and set it as the source clock for PWM. See page 254  */
    SYSCTL->RCC |= 0x000E0000; /* Use pre-divider valur of 64 and after that feed clock to PWM1 module*/

 /* Setting of PF2 pin for M1PWM6 channel output pin */
   	GPIOF->AFSEL |= (1<<2);       /* PF2 sets a alternate function*/
    GPIOF->PCTL &= ~0x00000F00; /*set PF2 as output pin */
    GPIOF->PCTL |= 0x00000500; /* make PF2 PWM output pin */
    GPIOF->DEN |= (1<<2);          /* set PF2 as a digital pin */
    
	 /* Setting of PC4 pin for M1PWM6 channel output pin */
   	GPIOC->AFSEL |= (1<<4);       /* PC4 sets a alternate function*/
    GPIOC->PCTL &= ~0x000F0000; /*set PC4 as output pin */
    GPIOC->PCTL |= 0x00050000; /* make PC4 PWM output pin */
    GPIOC->DEN |= (1<<4);          /* set PC4 as a digital pin */
    
	
    /* Set up PWM Generator 3 for Right Wheel */
    PWM1->_3_CTL &= ~(1<<0);      /* Disable Generator 3 */
    PWM1->_3_GENA = 0x0000008C;   /* PWM signal setup */
    PWM1->_3_LOAD = 5000;         /* Set PWM frequency */
    PWM1->_3_CMPA = 4999;         /* Start with lowest duty cycle */
    PWM1->_3_CTL |= 1;            /* Enable Generator 3 */

    /* Set up PWM Generator 1 for Left Wheel */
    PWM1->_1_CTL &= ~(1<<0);      /* Disable Generator 1 */
    PWM1->_1_GENA = 0x0000008C;   /* PWM signal setup */
    PWM1->_1_LOAD = 5000;         /* Set PWM frequency */
    PWM1->_1_CMPA = 4999;         /* Start with lowest duty cycle */
    PWM1->_1_CTL |= 1;            /* Enable Generator 1 */

    /* Enable PWM output */
    PWM1->ENABLE = (1<<6) | (1<<2); /* Enable M1PWM6 (PF2) and M1PWM2 (PC4) */
}
/* This function generates delay in ms */
/* calculations are based on 16MHz system clock frequency */

void Delay_ms(int time_ms)
{
    int i, j;
    for(i = 0 ; i < time_ms; i++)
        for(j = 0; j < 3180; j++)
            {}  /* excute NOP for 1ms */
}