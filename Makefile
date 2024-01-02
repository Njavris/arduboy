CC = avr-gcc
CFLAGS = -Os -mmcu=atmega32u4 -DF_CPU=16000000UL -DMUTE
OBJCOPY = avr-objcopy
PROGRAMMER = -c usbasp -p $(MCU)
PROG=test

FILES= io.o main.o

all: $(PROG).hex

$(PROG).hex: $(PROG).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

%.o : %.c
	$(CC) -c $(CFLAGS) $< -o $@


$(PROG).elf: $(FILES)
	$(CC) $(CFLAGS) -o $@ $(FILES)

upload: $(PROG).hex
	avrdude -Cavrdude.conf -v -patmega32u4 -cavr109 -P/dev/ttyACM0 -b57600 -D -Uflash:w:$(PROG).hex:i

clean:
	rm -f *.o *.elf *.hex
