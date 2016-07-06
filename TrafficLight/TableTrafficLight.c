// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 15, 2016

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "SysTick.h"

// ***** 2. Global Declarations Section *****
#define GPIO_PORTB_DATA_R       (*((volatile unsigned long *)0x400053FC)) 
#define GPIO_PORTB_DIR_R        (*((volatile unsigned long *)0x40005400))
#define GPIO_PORTB_AFSEL_R      (*((volatile unsigned long *)0x40005420))
#define GPIO_PORTB_DEN_R        (*((volatile unsigned long *)0x4000551C))
#define GPIO_PORTB_AMSEL_R      (*((volatile unsigned long *)0x40005528))
#define GPIO_PORTB_PCTL_R       (*((volatile unsigned long *)0x4000552C))
#define GPIO_PORTE_DATA_R       (*((volatile unsigned long *)0x400243FC)) 
#define GPIO_PORTE_DIR_R        (*((volatile unsigned long *)0x40024400))
#define GPIO_PORTE_AFSEL_R      (*((volatile unsigned long *)0x40024420))
#define GPIO_PORTE_DEN_R        (*((volatile unsigned long *)0x4002451C))
#define GPIO_PORTE_AMSEL_R      (*((volatile unsigned long *)0x40024528))
#define GPIO_PORTE_PCTL_R       (*((volatile unsigned long *)0x4002452C))
#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	

//states definition
#define AllRed 0
#define SouthGo 1
#define SouthWait 2
#define WestGo 3
#define WestWait 4
#define WalkOn 5
#define DontWalkOn1 6
#define DontWalkOff1 7
#define DontWalkOn2 8
#define DontWalkOff2 9

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void PortF_Init(void);
void PortB_Init(void);
void PortE_Init(void);

// ***** 3. Subroutines Section *****
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x20;     // 1) activate Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x0A;           // allow changes to PF3 and PF1
  GPIO_PORTF_AMSEL_R &= ~0x0A;        // 3) disable analog on PF3 and PF1
  GPIO_PORTF_PCTL_R &= ~0x0000F00F;   // 4) PCTL GPIO on PF3 and PF1
  GPIO_PORTF_DIR_R |= 0x0A;          // 5) PF3 and PF1 out
  GPIO_PORTF_AFSEL_R &= ~0x0A;        // 6) disable alt funct on PF3 and PF1
  GPIO_PORTF_DEN_R |= 0x0A;          // 7) enable digital I/O on PF3 and PF1
}

void PortB_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x02;          // 1) activate Port B
  delay = SYSCTL_RCGC2_R;          // allow time for clock to stabilize
  GPIO_PORTB_AMSEL_R &= ~0x3F;     // 3) disable analog functionality on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF;  // 4) configure PB5-0 as GPIO
  GPIO_PORTB_DIR_R |= 0x3F;        // 5) make PB5-0 out
  GPIO_PORTB_AFSEL_R &= ~0x3F;     // 6) disable alt funct on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;        // 7) enable digital I/O on PB5-0
}

void PortE_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x10;          // 1) activate Port E
  delay = SYSCTL_RCGC2_R;          // allow time for clock to stabilize
  GPIO_PORTE_AMSEL_R &= ~0x07;     // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x00000FFF;// 4) configure PE2-0 as GPIO
  GPIO_PORTE_DIR_R &= ~0x07;       // 5) make PE2-0 in
  GPIO_PORTE_AFSEL_R &= ~0x07;     // 6) disable alt funct on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;        // 7) enable digital I/O on PE2-0
}

struct State{
	unsigned long PBout; 						 //6-bit pattern to Output to Traffic Lights
	unsigned long PFout;						 //Output to Walk and Don't Walk Lights
	unsigned long wait;							 //Time to wait in units of 10ms
	unsigned long next[8];           //Array of next states based on inputs 0....8
};

typedef const struct State SType;
SType FSM[10] = {
	{0x24, 0x02, 50, {AllRed, WestGo, SouthGo, SouthGo, WalkOn, WalkOn, WalkOn, WalkOn} },
	{0x21, 0x02, 50, {SouthWait, SouthWait, SouthGo, SouthWait, SouthWait, SouthWait, SouthWait, SouthWait} },
	{0x22, 0x02, 50, {AllRed, WestGo, AllRed, WestGo, WalkOn, WestGo, WalkOn, WestGo} },
	{0x0C, 0x02, 50, {WestGo, WestGo, WestWait, WestWait, WestWait, WestWait, WestWait, WestWait} },
	{0x14, 0x02, 50, {AllRed, AllRed, SouthGo, SouthGo, WalkOn, WalkOn, SouthGo, WalkOn} },
	{0x24, 0x08, 50, {DontWalkOn1, DontWalkOn1, DontWalkOn1, DontWalkOn1, WalkOn, DontWalkOn1, DontWalkOn1, DontWalkOn1} },
	{0x24, 0x02, 50, {DontWalkOff1, DontWalkOff1, DontWalkOff1, DontWalkOff1, WalkOn, DontWalkOff1, DontWalkOff1, DontWalkOff1} },
	{0x24, 0x00, 50, {DontWalkOn2, DontWalkOn2, DontWalkOn2, DontWalkOn2, WalkOn, DontWalkOn2, DontWalkOn2, DontWalkOn2} },
	{0x24, 0x02, 50, {DontWalkOff2, DontWalkOff2, DontWalkOff2, DontWalkOff2, WalkOn, DontWalkOff2, DontWalkOff2, DontWalkOff2} },
	{0X24, 0x00, 50, {AllRed, WestGo, SouthGo, SouthGo, WalkOn, WestGo, SouthGo, SouthGo} }
};
int main(void){ 
	unsigned long cState = 0;									//initialize state of the FSM to State0
	unsigned long input;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff); // activate grader and set system clock to 80 MHz
	SysTick_Init();														//Initialize SysTick Timer
  PortF_Init();															//Initialize PortF
	PortB_Init();															//Initialize PortB
	PortE_Init();															//Initialize PortE
  EnableInterrupts();
  while(1){
		//Implementation of FSM controller
		GPIO_PORTB_DATA_R = FSM[cState].PBout;		//output Traffic lights for current state
		GPIO_PORTF_DATA_R = FSM[cState].PFout;		//output Walk lights for current state
		SysTick_Wait10ms(FSM[cState].wait);				//wait for time associated with current state
		input = GPIO_PORTE_DATA_R&0x07; 					//read input from PORTE 
		cState = FSM[cState].next[input];					//use input as index to next state
  }
}

