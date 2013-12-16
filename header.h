// 11-05-06 Giott-O-matic
// IEEESB of Unipd

// PWMX			RC2
// PWMY			RC1
// BUS_HCTL		PORTJ
// CMD_HCTL		RB3:RB7
// WATER 		RF6
// LINEFOLLOWER RF0
// BUTTON 		RB0
// RIGHT_FIRE   RF2
// LEFT_FIRE	RF4
// ENABLE ENG	RF6
// NORTH FIREUP RH7
// SOUTH FIREUP RH5
// WEST FIREUP  RH3
// EST FIREUP   RH1
// GREEN BUTTON RE7

// Definitions
#define ZERO 0x00
#define DECIMATION 4
#define PWMX CCPR1L		// output register PWMX pin RC2
#define PWMY CCPR2L 	// output register PWMY pin RC1
#define DIM 1			// bufferADC[DIM] size

// HCTL
#define XOY  PORTBbits.RB7	// B7 = !X/Y
#define RSTY PORTBbits.RB6	// B6 = RSTY
#define RSTX PORTBbits.RB5	// B5 = RSTX
#define SEL2 PORTBbits.RB4	// B4 = SEL2
#define SEL1 PORTBbits.RB3	// B3 = SEL1

// Line Follower
#define AN5 0x15 		// enable conversion CH5 in RF0 (signal coming from the line follower)

// Speed controller
#define KP 					32					// proportional gain (REMEMBER TO CHANGE THE THRESHOLDS!)
#define OFFSET 				127
#define THRESHOLD_INF		-3					// THRESHOLD_INF = -FLOOR(OFFSET/KP)
#define THRESHOLD_SUP 		4					// THRESHOLD_SUP = FLOOR((255-OFFSET)/KP)
#define EN_ENGINES 			PORTFbits.RF6		// enable the power relè
#define VEL_OUTER 			30					// external wheels speed
#define VEL_INNER 			10					// internal wheels speed 
#define VEL_LINEFOLLOWER 	20					// speed in line following mode
#define A_LITTLE_BIT 		10               	// low speed to point the target

// "Position controller"
#define NINETY_DEGREES 55

// Estintore
#define INTFDX 	  PORTFbits.RF2
#define INTFSX	  PORTFbits.RF4
#define SOUTH  	  PORTHbits.RH7
#define NORTH     PORTHbits.RH5
#define WEST  	  PORTHbits.RH3
#define EST    	  PORTHbits.RH1
#define EXTINGUISHER PORTEbits.RE2

// Stati
#define GREENBUTTON 	PORTEbits.RE7
#define LINEFOLLOWER 	98
#define ROTATEDX 		97
#define RETURNDX 		96
#define ROTATESX 		95
#define RETURNSX 		94
#define RIGHT_FIRE 		93
#define LEFT_FIRE 		92
#define STOP 			91

// Time constants (1,6 ms x)
#define LITTLE_DELAY 	1000
#define TIMEOUT 		3128	// max time to extinguish the fire (3.2 sec)
#define ONE_SECOND 		800

// Variables
unsigned char state;		// current state of the robot
unsigned char lastState;	// previous state of the robot
char position;
char lastPosX;				// value of the X encoder
char lastPosY;				// value of the Y encoder
char rifVel;				// reference for the robot speed
char rifVelX;				// reference for X speed	
char rifVelY;				// reference for Y speed
char bufferADC[DIM];		// buffer for the ADC
char lastD;					// saved linefollower value
unsigned char countdown;  	// decimator for the ADC
int stateUpdate;			// time update of the lastSate
#include <LUT.h>			// line follower lookup table
int time; 					// time

// Functions
void main (void);
void initADC(void);
void initPWM(void);
void initTimer(void);
void initWater(void);
void initHCTL(void);
void initInterrupt(void);
void InterruptHandlerHigh (void);
void InterruptHandlerLow (void);
char getDeltaPos(void);
char getVelX(void);
char getVelY(void);
char posRequest(void);
void runADC(char);
char saveADC(void);
void lineFollowing(void);

// A/D
//#define AN0 0x01 	// enable conversion CH0 (already taken by potentiometer!)
//#define AN1 0x05 	// enable conversion CH1 
//#define AN2 0x09 	// enable conversion CH2
//#define AN3 0x0D 	// enable conversion CH3
//#define AN4 0x11 	// enable conversion CH4
//#define AN6 0x19 	// enable conversion CH6 
//#define AN7 0x1D 	// enable conversion CH7
//#define AN8 0x21 	// enable conversion CH8
//#define AN9 0x25 	// enable conversion CH9
//#define AN10 0x29 // enable conversion CH10
//#define AN11 0x2D // enable conversion CH11
//#define AN12 0x31 // enable conversion CH12		
//#define AN13 0x35 // enable conversion CH13
