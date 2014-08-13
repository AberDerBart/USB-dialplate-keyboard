CCFLAGS=-I. -Iusbdrv -DF_CPU=16500000
MMCU=attiny85
OBJS=main.o usbdrv/usbdrv.o usbdrv/usbdrvasm.ao
TARGET=keyboard

${TARGET}.hex:${TARGET}.elf
	avr-objcopy -O ihex -R .eeprom ${TARGET}.elf ${TARGET}.hex 

${TARGET}.elf: ${OBJS}
	avr-gcc ${CCFLAGS} -mmcu=${MMCU} -Os -o${TARGET}.elf ${OBJS}

%.o: %.c
	avr-gcc -c ${CCFLAGS} -mmcu=${MMCU} -Os -o $@ $<

%.ao: %.S
	avr-gcc -c -x assembler-with-cpp -mmcu=${MMCU} ${CCFLAGS} -o $@ $<


.PHONY: clean
clean:
	rm -f ${OBJS} ${TARGET}.hex ${TARGET}.elf

.PHONY: flash
flash: ${TARGET}.hex
	sudo avrdude -c linuxspi -p t85 -P /dev/spidev0.0 -U flash:w:${TARGET}.hex -b 50000

.PHONY: prepare
prepare:
	sudo modprobe spi_bcm2708

.PHONY: fuses
fuses:
	sudo avrdude -p t85 -P /dev/spidev0.0 -c linuxspi -b 10000 -U lfuse:w:0xc1:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m 
