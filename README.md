IMPORTANT NOTICE:
	USB-dialplate-keyboard is in a development state and NOT WORKING CORRECTLY in the current state


USB-dialplate-keyboard is a project to convert an old phones dialplate into a USB keyboard using an attiny85 microcontroller.

It is based on the 4-key-keyboard by Flip van den Berg (http://www.flipwork.nl).
For more information on the 4-key-keyboard, see http://blog.flipwork.nl/?x=entry:entry100224-003937

The project uses the v-usb library by Objective Development (http://www.obdev.at).
For more information on v-usb, see http://www.obdev.at/vusb/


The makefile provided is intended to use with avr-gcc.
To compile the project, just run 
	make

The makefile also provide-s functionality to flash it on the attiny using avrdude and a Raspberry Pis SPI interface as ISP.
For details on setting up avrdude for the Raspberry Pis SPI interface, see http://kevincuzner.com/2013/05/27/raspberry-pi-as-an-avr-programmer/
To flash the hex-file on the attiny, simply run 
	make flash
To set the fuses, run
	make fuses  
NOTE: the makefile is configured for a stock raspbian configuration, so you are going to be allowed to use sudo as the current user or you will have to change the makefile.
