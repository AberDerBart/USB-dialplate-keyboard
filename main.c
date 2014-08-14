/* Name: main.c
 * Project: 4-Key-Keyboard
 * Author: Flip van den Berg - www.flipwork.nl
 * Creation Date: February 2010
 * Based on V-USB drivers from Objective Developments - http://www.obdev.at/products/vusb/index.html
 */


/*

IMPORTANT: 	This project uses fuse settings that disable the reset pin in order to use it as an IO pin.
			This means that if you can only re-program the AVR afterwards using High Voltage Serial Programming (HVSP)

			If you have a programmer that only supports ISP make sure to upload the firmware before setting the reset-disable fuse!

Working fuse setting on ATTiny45/85:

EXTENDED: 0xFF
HIGH:	 0x5F
LOW:	 0xC1

*/

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdlib.h>

#include "usbdrv.h"


#define BUTTON_PORT_TICK PORTB       /* PORTx - register for BUTTON 2 output */
#define BUTTON_PIN_TICK PINB         /* PINx - register for BUTTON 2 input */
#define BUTTON_BIT_TICK PB4          /* bit for BUTTON 2 input/output */


#define BUTTON_PORT_DIAL PORTB       /* PORTx - register for BUTTON 3 output */
#define BUTTON_PIN_DIAL PINB         /* PINx - register for BUTTON 3 input */
#define BUTTON_BIT_DIAL PB3          /* bit for BUTTON 3 input/output */

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

static uchar    buttonState_TICK = 3;		/*  stores state of button 1 */
static uchar    buttonState_DIAL = 3;		/*  stores state of button 2 */

static uchar    buttonChanged_DIAL;		

static uchar	debounceTimeIsOverDial = 1;
static uchar    counter;	//counts ticks
static uchar timerCnt;


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

    if(TIFR & (1 << OCF1A)){
        TIFR = (1 << OCF1A); /* clear pwm flag */
	++timerCnt;
    }
}

static void buildReport(void){

	uchar key; 

	if (buttonState_DIAL != 0 && counter>0 &&  counter<11){ // if dial switch is opened (=dialing a digit finished) and the counter is valid (values 1 to 10 are valid)
		key = 29+counter; //key is the number key with the value of counter, except when counter=10, key='0'
	}else {
		key = 0; //dial switch is closed
		counter=0;
		newReport=1;
	}
	reportBuffer[4] = key;
}

static void checkButtonChange(void) {
	
	uchar tempButtonValue_TICK = bit_is_set(BUTTON_PIN_TICK, BUTTON_BIT_TICK); //status of switch is stored in tempButtonValue 
	uchar tempButtonValue_DIAL = bit_is_set(BUTTON_PIN_DIAL, BUTTON_BIT_DIAL);  //status of switch is stored in tempButtonValue 

	static uchar lastTimerTick;
	static uchar lastTimerDial;

	if (tempButtonValue_TICK != buttonState_TICK){ //if status has changed
		if(timerCnt-lastTimerTick>=2){
			if(tempButtonValue_TICK!=0){
				counter++;
			}
		}
		lastTimerTick=timerCnt;
		buttonState_TICK = tempButtonValue_TICK;	// change buttonState to new state
	}
	if (tempButtonValue_DIAL != buttonState_DIAL){ //if status has changed
		if(timerCnt-lastTimerDial >= 2){
			buttonState_DIAL = tempButtonValue_DIAL;	// change buttonState to new state
		}
		newReport = 0; // initiate new report 
	}
}

/* ------------------------------------------------------------------------- */

static void timerInit(void)
{
	//set timer1 speed to 16.5 MHZ/2048 = 8056.64Hz, enable reset on TCNT1 = OCR1C and PWM A
	TCCR1 = 0xcc;
	//disable comparator b 
	GTCCR = 0x00;
	//reset counter after 201 ticks (8056.64 Hz/201 =40.07 Hz reset rate)
	OCR1C = 100; 
	//set OCF1A flag after 100 ticks, roughly the half of a cycle
	OCR1A = 50;
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

	/* turn on internal pull-up resistor for the switches */
	BUTTON_PORT_TICK |= _BV(BUTTON_BIT_TICK);
	BUTTON_PORT_DIAL |= _BV(BUTTON_BIT_DIAL);

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
