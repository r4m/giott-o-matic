// 11-05-06 Giott-O-matic
// IEEESB of Unipd

#include <p18f8722.h>
#include <header.h>
#pragma config WDT = ON, WDTPS = 512
#pragma config OSC = HSPLL
#pragma config BOREN = OFF
#pragma config PWRT = ON
#pragma config MODE = MC
#pragma config LVP = OFF
#pragma config DEBUG = OFF
#pragma config XINST = OFF 

//----------------------------------------------------------------------------
// Main
void main()
{
	unsigned char i;
	char targetPosition;
	char move;
	char rotate;
	move = 0;
	rotate = 0;
	EN_ENGINES = 0;
	lastPosX = ZERO;
	lastPosY = ZERO;
	lastD = ZERO;
	refVelX = ZERO;
	refVelY = ZERO;
	countdown = ZERO;
	stateUpdate = ZERO;
	state = STOP;
	lastState = STOP;
	time = ZERO;
	initHCTL();
	initPWM();
	initTimer();
	initADC();
	initInterrupt();
	initWater();

	// delayed engine connection
	time = 1250;				// wait 2 seconds...
	while(time > ZERO)
	{
	}
	EN_ENGINES = 1;				// ... before enabling engines and neon

  	while (1)
	{
		if (state == STOP)
		{
			// move to LINEFOLLOWER state when the green button is pressed
			if (lastState==STOP && !GREENBUTTON)				
			{
				state = LINEFOLLOWER;
				stateUpdate = 0;
			}
		}	
		if (state == LINEFOLLOWER)
		{
			if(INTFDX && (lastState!=RETURNSX))	// if the robot detects a fire on the right side...
			{
				state = STOP;					// ... it stops ...
				time = LITTLE_DELAY;				// ... and waits
				while(time > 0)
				{}
				state = ROTATEDX;				// ... and then it moves to ROTATEDX state
				stateUpdate = 0;
			}
			if(INTFSX && (lastState!=RETURNDX))					// if it detects a fire on the left side...
			{
				state = STOP;					// ... it stops ...	
				time = LITTLE_DELAY;				// ... and waits
				while(time > 0)
				{}
				state = ROTATESX;				// ... and then it moves ROTATESX
				stateUpdate = 0;
			}
			else refVel = VEL_LINEFOLLOWER;
		}
		if (state == ROTATEDX)
		{
			targetPosition = getDeltaPos()- NINETY_DEGREES;
			time = TIMEOUT;
			while (position > targetPosition && time>0)
			{
			}
			state = RIGHT_FIRE;				
			stateUpdate = 0;
		}
		if (state == ROTATESX)
		{
			targetPosition = getDeltaPos() + NINETY_DEGREES;
			time = TIMEOUT;
			while (position < targetPosition && time>0)
			{
			}
			state = LEFT_FIRE;				
			stateUpdate = 0;
		}
		if (state == RIGHT_FIRE)
		{
			time = 6000;
			while (time > 0) 
			{
				if (NORTH == 0 && SOUTH == 0 && EST == 0 && WEST == 0)
				{ break; }
				if (NORTH == 1)
					move = A_LITTLE_BIT<<1;
				else if (SOUTH == 1)
					move = -(A_LITTLE_BIT<<1);
				if (EST == 1)
					rotate = A_LITTLE_BIT;
				else if (WEST == 1)
					rotate = -A_LITTLE_BIT;
				
				refVelX = move - rotate;
				refVelY = move + rotate; 
					
			}
			refVelX = refVelY = 0;
			EXTINGUISHER = 1;
			time = 2000;				
			while(time > 0)
			{}
			EXTINGUISHER = 0;
			state = RETURNSX;
			stateUpdate = 0;
		}
		if (state == LEFT_FIRE)
		{
			time = 6000;
			while (time > 0) 
			{
				if (NORTH == 0 && SOUTH == 0 && EST == 0 && WEST == 0)
				{ break; }
				if (NORTH == 1)
					move = A_LITTLE_BIT;
				else if (SOUTH == 1)
					move = -A_LITTLE_BIT;
				if (EST == 1)
					rotate = A_LITTLE_BIT;
				else if (WEST == 1)
					rotate = -A_LITTLE_BIT;
				
				refVelX = move - rotate;
				refVelY = move + rotate; 
					
			}
			refVelX = refVelY = 0;
			EXTINGUISHER = 1;
			time = 2000;				
			while(time > 0)
			{}
			EXTINGUISHER = 0;
			state = RETURNDX;
			stateUpdate = 0;
		}
		if (state == RETURNDX)
		{
			targetPosition = getDeltaPos() - NINETY_DEGREES;
			time = TIMEOUT;
			while (position > targetPosition && time>0)
			{
			}
			lastState = RETURNDX;
			state = LINEFOLLOWER;
			stateUpdate = 0;			
		}
		if (state == RETURNSX)
		{
			targetPosition = getDeltaPos() + NINETY_DEGREES;
			time = TIMEOUT;
			while (position < targetPosition && time>0)
			{
			}
			lastState = RETURNSX;
			state = LINEFOLLOWER;
			stateUpdate = 0;			
		}
		if (lastState!=STOP && !GREENBUTTON)
		{
			state = STOP;
			stateUpdate = 0;
		}
	}
}

//----------------------------------------------------------------------------
// Init timer
void initTimer()
{
	// init interrupt on timer 0 (every 1.6 ms)
	TMR0H = 0;                    // clear timer
	TMR0L = 0;                    // clear timer
	T0CON = 0xC5;                 // set up timer0 - prescaler 1:64
}

//----------------------------------------------------------------------------
// Inizializza gli interrupt
void initInterrupt()
{
	INTCONbits.GIEH = 0;		// disable global interrupts
	RCONbits.IPEN = 1;			// enable priority management

	// init interrupt on timer 0 (every 1.6 ms)
	INTCONbits.TMR0IE = 1;		// enable TMR0 interrupt
	INTCON2bits.TMR0IP = 1;		// TMR0 high priority

	// init pheriperals interrupt
	INTCONbits.PEIE = 1;    	// enable Periph. interrupt

	// init ADC interrupt
	PIR1bits.ADIF = 0; 			// reset interrupt ADC
	PIE1bits.ADIE = 1;			// enable interrupt A/D
	IPR1bits.ADIP = 0;			// set low priority

	// init interrupt of RB0 button
	//INTCONbits.INT0IF = 0;    // enable INT0 interrupt
	//INTCONbits.INT0IE = 1;    // enable INT0 interrupt

	// init interrupt RB1 and RB2 ports
	//INTCON3bits.INT1IF = 0;      // reset INT1 interrupt flag
	//INTCON3bits.INT2IF = 0;      // reset INT2 interrupt flag
	//INTCON3 = 0x18;		       // enable INT1 and INT2 and set low priority

	INTCON2bits.NOT_RBPU = 1;	// disable pullup B ports
	INTCONbits.GIEH = 1;      	// enable global interrupts	
}

//----------------------------------------------------------------------------
// Init interface with HCTL-2032
void initHCTL()
{
	TRISJ = 0xFF;	// porta J tutte digital input dall'HCTL
	TRISB = 0x07;	// porte RB0:RB2 ingressi, RB3:RB7 output per l'HTCL
	SEL2 = 0;		// set SEL2 to 0
	SEL1 = 1;		// set SEL1 to 1
	TRISD = 0x00;	// port D output (led)
	RSTX = 0;		// counter X to 0
	RSTY = 0;		// counter Y to 0
	RSTY = 1;		// set RSTY to 1
	RSTX = 1;		// set RSTX to 1
}

//----------------------------------------------------------------------------
// Init water sensors (upper)
void initWater()
{	
	// Reset as inputs not as outputs???
	TRISH = 0xAF;
	TRISEbits.TRISE2 = 0;
}

//----------------------------------------------------------------------------
// High priority interrupt vector
#pragma code InterruptVectorHigh = 0x0808

void InterruptVectorHigh()
{
  _asm
    goto InterruptHandlerHigh 		// jump to interrupt routine
  _endasm
}

//----------------------------------------------------------------------------
// High priority interrupt routine
#pragma code
#pragma interrupt InterruptHandlerHigh

void InterruptHandlerHigh()
{
	char velX;
	char velY;
	char errVelX;
	char errVelY;

	// check for TMR0 overflow 
	if (INTCONbits.TMR0IF)
    {                                   
		INTCONbits.TMR0IF = 0;       // clear interrupt flag
		ClrWdt();					 // reset del WDT
		time--;						 
		if (stateUpdate==UNSECONDO) lastState = state;
		if (stateUpdate!=0xFFFF) stateUpdate++;

		// ---- SPEED CONTROLLER ----
		velX = getVelX();				// speed of X engine reading
		velY = getVelY();				// speed of Y engine reading

		// P controller of X
		errVelX = (refVelX - velX);		// compute error X
		if (errVelX > THRESHOLD_SUP)
		{
			errVelX = THRESHOLD_SUP;
		} 
		else if (errVelX < THRESHOLD_INF)
		{
			errVelX = THRESHOLD_INF;
		}
		errVelX = errVelX * KP;			// computation of proportional gain
		PWMX = errVelX + OFFSET;		// write it on PWMX output

		// P controller of Y
		errVelY = (refVelY - velY);		
		if (errVelY > THRESHOLD_SUP)
		{
			errVelY = THRESHOLD_SUP;
		} 
		else if (errVelY < THRESHOLD_INF)
		{
			errVelY = THRESHOLD_INF;
		}
		errVelY = errVelY * KP;			
		PWMY = errVelY + OFFSET;		
		// ---- END SPEED CONTROLLER ----

		switch(state)
		{
			case LINEFOLLOWER:
				// read analog inputs
				if(countdown==DECIMATION)
				{
					countdown=0;
					runADC(AN5);	// line follower analog signal conversion
				}
				PORTD = 0x02;
			break;
			case ROTATEDX:
			case RETURNDX:
				position = getDeltaPos();		
				refVelX = VEL_INNER;
				refVelY = VEL_OUTER;
				PORTD = 0x04;
			break;
			case ROTATESX:
			case RETURNSX:
				position = getDeltaPos();		
				refVelX = VEL_OUTER;
				refVelY = VEL_INNER;
				PORTD = 0x08;
			break;
			case RIGHT_FIRE:
			case LEFT_FIRE:
				PORTD = 0x10;
			break;
			case STOP:
			default:
				refVelX = 0;	
				refVelY = 0;
				PORTD = 0x01;
		}
		countdown++;
    }
}

//----------------------------------------------------------------------------
// Low priority interrupt vector
#pragma code InterruptVectorLow = 0x0818

void InterruptVectorLow()
{
  _asm
    goto InterruptHandlerLow // jump to interrupt routine
  _endasm
}

//----------------------------------------------------------------------------
// Low priority interrupt routine
#pragma code
#pragma interrupt InterruptHandlerLow

void InterruptHandlerLow()
{
	unsigned char channel;			// selected ADC channel

	if (PIR1bits.ADIF)				// ADC interrupt
    {                               
		PIR1bits.ADIF = 0;          // clear interrupt flag
		channel = saveADC();		
		switch(channel)
		{
			case (AN5>>2):
				lineFollowing();
				break;
		}
	}
}

//----------------------------------------------------------------------------
// Compute the engine speed via a fuzzy logic to follow the line
void lineFollowing()
{
	unsigned char d = ZERO;			// position on the line follower
	char deltaD = ZERO;				// derivative of the position on the line follower
	char temprefVel;				// speed of the robot saturated
	unsigned char tempDeltaV;		// absolute difference VR-VL
	char deltaVRel;					// relative difference VR-VL

	d = bufferADC[0];
	if (d > 218) 					// saturate d in 48 ... 218
	{
		d = 218;
	} else if (d < 48)
	{
		d = 48;
	} 
	d = d - 0x30;		// offset -48
	d = d + (d>>1);					// scaling 1.5
	if (d & 0x08)					// rounding
	{
		d = (d + 0x08)>>4;
	} else
	{
		d = d>>4;
	}
	deltaD = bufferADC[0] - lastD;	
	lastD = bufferADC[0];
	if (deltaD > 31) 					// saturate deltaD a +-32
	{
		deltaD = 31;
	} else if (deltaD < -32)
	{
		deltaD = -32;
	}
	if (deltaD & 0x02)					// rounding
	{
		deltaD = (deltaD + 0x02);
	}
	deltaD = (deltaD<<2) & 0xF0;		// move deltaD to the most significant 4 bit
	d = deltaD + d;						// d is an index that indicates with the first 4 bits its derivative 
										// and with the other 4 bits its current value
	// Beginning the fuzzy line-follower routine
	if (refVel<(Vmax[d]<<1))
	{
		temprefVel = refVel;
	}
	else 
	{
		temprefVel = (Vmax[d]<<1);
	}
	deltaVRel = deltaV[d];
	if ( !(temprefVel & 0x80) && !(deltaVRel & 0x80) ) // both positive
	{
		tempDeltaV = deltaVRel * temprefVel;
		tempDeltaV = PRODH<<2;
		refVelX = temprefVel + tempDeltaV;
		refVelY = temprefVel - tempDeltaV;
	}
	if ( !(temprefVel & 0x80) && (deltaVRel & 0x80) ) // positive speed and negative difference
	{
		tempDeltaV = -deltaVRel * temprefVel;
		tempDeltaV = PRODH<<2;
		refVelX = temprefVel - tempDeltaV;
		refVelY = temprefVel + tempDeltaV;
	}
	if ( (temprefVel & 0x80) && (deltaVRel & 0x80) ) // both negative
	{
		tempDeltaV = -deltaVRel * -temprefVel;
		tempDeltaV = PRODH<<2;
		refVelX = temprefVel - tempDeltaV;
		refVelY = temprefVel + tempDeltaV;
	}
	if ( (temprefVel & 0x80) && !(deltaVRel & 0x80) ) // negative speed and positive difference
	{
		tempDeltaV = deltaVRel * -temprefVel;
		tempDeltaV = PRODH<<2;
		refVelX = temprefVel + tempDeltaV;
		refVelY = temprefVel - tempDeltaV;	
	}
}

//----------------------------------------------------------------------------
// read the differece (second byte) betweend the two encoders
char getDeltaPos()
{
	char posizioneX;
	char posizioneY;
	char delta;
	SEL1 = 0;
	XOY = 0;
	posizioneX = posRequest();
	XOY = 1;
	posizioneY = posRequest();
	SEL1 = 1;
	delta = posizioneX - posizioneY;
	return delta;
}

//----------------------------------------------------------------------------
// get the current X speed of the engines 
char getVelX()
{
	char posX;
	char vel;
	XOY= 0;					// X input reading
	posX = posRequest();	// X position reading
	vel = posX - lastPosX;	// computation of X speed
	lastPosX = posX;		// update of old X position
	return vel;
}

//----------------------------------------------------------------------------
// get the current Y speed of the engines 
char getVelY()
{
	char posY;
	char vel;
	XOY = 1;				
	posY =  posRequest();	
	vel = posY - lastPosY;	
	lastPosY = posY;		
	return vel;
}

//----------------------------------------------------------------------------
// Returns the requested position, checking that the data is correct
char posRequest()
{	
	char tempPos1;
	char tempPos2;
	while(1)
	{
		tempPos1 = PORTJ;
		tempPos2 = PORTJ;
		if(tempPos1 == tempPos2)
		{
			return tempPos1;
		}
	} 
}

//----------------------------------------------------------------------------
// ADC init
void initADC()
{
	unsigned char i;
	TRISA = 0xFF; 			// set A port as input
	TRISF = 0xBF; 			// F port configuration (7,5,3,1,0 input 6,4,2 output)
	ADCON0 = 0x00;
	ADCON1 = 0x09;			// analog inputs AN0:AN5 activation
	ADCON2 = 0x2E;			// configuration of the acquisition times: left alignment, 12TAD, FOSC/64
	for(i=0;i<DIM;i++)
	{
		bufferADC[i] = 0;	
	}
}

//----------------------------------------------------------------------------
// A/D input acquisition
void runADC(char ing)
{
	ADCON0 = ing; 			// input selection 
	ADCON0bits.GO = 1; 		// start A/D conversion
	return;
}  

//----------------------------------------------------------------------------
// Save the data read by the ADC in the corresponding location of the buffer and returns 
// the acquired channel. This routine is executed at every Low priority interrupt
char saveADC()
{	
	char channel;
	char valore;
	channel = (ADCON0 >> 2);
	valore = ADRESH;
	switch(channel)
	{
		case (AN5>>2):
			bufferADC[0] = valore;
			break;
	}
	return channel;
}

//----------------------------------------------------------------------------
// Init PWM (ECCP1 (PWMX) on RC2, ECCP2 (PWMY) on RC1)
void initPWM() 
{ 
	TRISCbits.TRISC1 = 1;	// disabling output on ECCP1
	TRISCbits.TRISC2 = 1;	// disabling output on ECCP1
	PR2 = 0xFF;				// set of the PWM period
	CCP1CON = 0x0C;			// ECCP1 configuration
	CCP2CON = 0x0C;			// ECCP2 configuration
	CCPR1L = 127;			// set of PWM1 = PWMX = 127
	CCPR2L = 127;			// set of PWM2 = PWMY = 127
	// set TMR2 to use it for the PWM
	T3CONbits.T3CCP2 = 0;	
	T3CONbits.T3CCP1 = 0;
	PIR1bits.TMR2IF = 0;	// reset flag interrupt TMR2
	T2CON=0x04;				// set TMR2 prescaler
	// attesa del primo overflow del timer2
	while (PIR1bits.TMR2IF == 0)
	{	
	}
	TRISCbits.TRISC1 = 0;	// enabling output on ECCP1 
	TRISCbits.TRISC2 = 0;	// enabling output on ECCP2
} 

