/* Name: main.c
 * Project: USB-dialplate-keyboard
 * Author: Jonas "AberDerBart" Grosse-Holz
 * Creation Date: August 2014
 * Based on V-USB drivers from Objective Developments - http://www.obdev.at/products/vusb/index.html
 * and 4-Key-Keyboard by Flip van den Berg - http://blog.flipwork.nl/?x=entry:entry100224-003937
 */


/*

Working fuse setting on ATTiny85:

EXTENDED: 0xFF
HIGH:	 0xDF
LOW:	 0xC1

*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdint.h>

#include "usbdrv.h"


#define PORT_TICK_SWITCH PORTB       /* PORTx - register for tick switch output */
#define PIN_TICK_SWITCH PINB         /* PINx - register for tick switch input */
#define PB_TICK_SWITCH PB4          /* bit for tick switch input/output */


#define PORT_DIAL_SWITCH PORTB       /* PORTx - register for dial switch output */
#define PIN_DIAL_SWITCH PINB         /* PINx - register for dial switch input */
#define PB_DIAL_SWITCH PB3          /* bit for dial switch input/output */

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[8] = {0,0,0,0,0,0,0,0};    /* buffer for HID reports */

/* Reportbuffer format:

	0  Modifier byte
	1  reserved
	2  keycode array (0)
	3  keycode array (1)
	4  keycode array (2)
	5  keycode array (3)
	6  keycode array (4)
	7  keycode array (5)
	
	<< This is the standard usb-keyboard reportbuffer. It allows for 6 simultaneous keypresses to be detected (excl. modifier keys). In this application we only use 1, so the last 5 bytes in this buffer will always remain 0. >>
	<< I decided not to optimize this in order to make it easy to add extra keys that can be pressed simultaneously>>
	
   Modifier byte: 8 bits, each individual bit represents one of the modifier keys.

   	bit0  LEFT CTRL		(1<<0)
	bit1  LEFT SHIFT	(1<<1)
	bit2  LEFT ALT		(1<<2)
	bit3  LEFT GUI		(1<<3)
	bit4  RIGHT CTRL	(1<<4)
	bit5  RIGHT SHIFT	(1<<5)
	bit6  RIGHT ALT		(1<<6)
	bit7  RIGHT GUI		(1<<7)

	an example of a reportBuffer for a CTRL+ALT+Delete keypress:

	{((1<<0)+(1<<2)),0,76,0,0,0,0,0}

	the first byte holds both the LEFT CTRL and LEFT  modifier keys the 3rd byte holds the delete key (== decimal 76)

*/

static uchar    idleRate;           /* in 4 ms units */
static uchar    newReport = 0;		/* current report */

static uchar    counter;			//counts ticks
static uchar    timerCnt;		//a counter that is increasing with roughly 161Hz (see function timerInit), used for debouncing



/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)	** Modifier Byte **
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x03,                    //   INPUT (Cnst,Var,Abs)	** Reserved Byte **
    0x95, 0x05,                    //   REPORT_COUNT (5)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x05, 0x08,                    //   USAGE_PAGE (LEDs)
    0x19, 0x01,                    //   USAGE_MINIMUM (Num Lock)
    0x29, 0x05,                    //   USAGE_MAXIMUM (Kana)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)	** LED Report **
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x03,                    //   REPORT_SIZE (3)
    0x91, 0x03,                    //   OUTPUT (Cnst,Var,Abs)	** LED Report Padding **
    0x95, 0x06,                    //   REPORT_COUNT (6)		** here we define the maximum number of simultaneous keystrokes we can detect ** 
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)	** Key arrays (6 bytes) **
    0xc0                           // END_COLLECTION  
};
 
static void timerPoll(void)
{
	//increases the counter every time timer1 is resetted
	if(TIFR & (1 << OCF1A)){
        	TIFR = (1 << OCF1A); /* clear pwm flag */
		++timerCnt;
	}
}

#define STATE_NONE 0
#define STATE_TOGGLE 1
#define STATE_BOUNCE 2

#define DEBOUNCE_COUNT 2

struct pinState {
	int pin;
	int pb;
	uchar state;
	uchar level;
	uint16_t lastBounce;
};

struct pinState stateTICK={
	.pin=0,
	.pb=0,
	.state=STATE_NONE,
	.level=3,
	.lastBounce=0
};

struct pinState stateDIAL={
	.pin=0,
	.pb=0,
	.state=STATE_NONE,
	.level=3,
	.lastBounce=0
};

static void buildReport(void){

	if (stateDIAL.level != 0 && counter>0 &&  counter<11){		// if dial switch is opened (=dialing a digit finished) and the counter is valid (values 1 to 10 are valid)
		reportBuffer[4] = 29+counter;				//key is the number key with the value of counter, except when counter=10, key='0'
		counter=0;
	}else {								
		reportBuffer[4] = 0;					//send the 0 to explain the computer, that no key is pressed anymore
		counter=0;
		newReport=1;
	}
}

static void updatePin(struct pinState* s){
	uchar newLevel=bit_is_set(s->pin,s->pb);
	
	if(s->state == STATE_BOUNCE){

		//if the debounce timer ran out, save the new state and set the STATE_TOGGLE flag if neccessary
		if((s->lastBounce-timerCnt)%256 >= DEBOUNCE_COUNT){
			if(s->level != newLevel){
				s->state = STATE_TOGGLE;
			}else{
				s->state = STATE_NONE;
			}
		}else{
			//if there was another toggle, we're still bouncing, adjust the timestamp of the last bounce
			if(s->level != newLevel){
				s->lastBounce = timerCnt;
			}
		}
	}else{
		// if the level changed, change state to STATE_BOUNCE
		if(s->level != newLevel){
			s->state = STATE_BOUNCE;
			s->lastBounce = timerCnt;
		}else{
			s->state = STATE_NONE;
		}
	}

	//update the level
	s->level = newLevel;
}

static void checkButtonChange(void) {
	
	updatePin(&stateTICK);
	updatePin(&stateDIAL);

	if(stateTICK.state == STATE_TOGGLE && stateTICK.level != 0){
		counter++;
	}
	if(stateDIAL.state == STATE_TOGGLE){
		newReport=0;
	}
}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
	//set timer1 speed to 16.5 MHZ/2048 = 8056.64Hz, enable reset on TCNT1 = OCR1C and PWM A
	TCCR1 = 0xcc;
	//disable comparator b 
	GTCCR = 0x00;
	//reset counter after 50 ticks (8056.64 Hz/50 = 161.13 Hz reset rate)
	OCR1C = 50; 
	//set OCF1A flag after 25 ticks, roughly the half of a cycle
	OCR1A = 25;
}

/* -------------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* -------------------------------------------------------------------------------- */

uchar	usbFunctionSetup(uchar data[8])
{
usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){    /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
	return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
uchar       step = 128;
uchar       trialValue = 0, optimumValue;
int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
            optimumDev = x;
            optimumValue = OSCCAL;
        }
    }
    OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void    hadUsbReset(void)
{
    calibrateOscillator();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM byte 0*/
}


/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void)
{
uchar   i;
uchar   calibrationValue;

	do {} while (!eeprom_is_ready());
    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    
    usbInit();
    usbDeviceDisconnect();  /* enforce re-enumeration, do this while interrupts are disabled! */
    i = 0;
    while(--i){             /* fake USB disconnect for > 250 ms */
        wdt_reset();
        _delay_ms(1);
    }
    usbDeviceConnect();

    wdt_enable(WDTO_2S);

	//initialize input pins
	stateTICK.pin=PIN_TICK_SWITCH;
	stateTICK.pb=PB_TICK_SWITCH;
	stateDIAL.pin=PIN_DIAL_SWITCH;
	stateDIAL.pb=PB_DIAL_SWITCH;
	/* turn on internal pull-up resistor for the switches */
	PORT_TICK_SWITCH |= _BV(PB_TICK_SWITCH);
	PORT_DIAL_SWITCH |= _BV(PB_DIAL_SWITCH);

    timerInit();
	
	counter=0;

    sei();

    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
	checkButtonChange();
	if(usbInterruptIsReady() && newReport == 0){ /* we can send another report */
		buildReport();
		usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
	}
        
	timerPoll();
	}
   	return 0;
}
