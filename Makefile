CODEFILE=LoopMKII
MMCU=attiny24

CC=avr-gcc

write: $(CODEFILE).hex
#	sudo $(PROG) -vvv -P usb -c avrisp2 -p ATtiny24 -U flash:w:$(CODEFILE).hex:i
	sudo avrdude -vvv -P usb -c avrisp2 -p ATtiny24 -B 100 -U flash:w:$(CODEFILE).hex:i 

$(CODEFILE).hex: $(CODEFILE).elf
	avr-objcopy -j .text -j .data -O ihex $(CODEFILE).elf $(CODEFILE).hex
	avr-objcopy -j .text -j .data -O binary $(CODEFILE).elf $(CODEFILE).bin

$(CODEFILE).elf: $(CODEFILE).o
	$(CC) -mmcu=$(MMCU) -o $(CODEFILE).elf $(CODEFILE).o

$(CODEFILE).o: $(CODEFILE).c
	$(CC) -g -Os -mmcu=$(MMCU) -c $(CODEFILE).c

read:
	sudo avrdude -vvv -P usb -c avrisp2 -p t24 -U flash:r:flashdump.hex:i

setfuse:
	sudo avrdude -vvv -P usb -c avrisp2 -p t24 -B 100 -U lfuse:w:0xe4:m

readfuse:
	sudo avrdude -vvv -P usb -c avrisp2 -p t24 -U lfuse:r:-:h

clean:
	rm -f *.o
	rm -f *.elf
	rm -f *.hex
