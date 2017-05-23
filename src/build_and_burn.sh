#!/bin/sh
if avr-gcc -mmcu=attiny13 -Os main.c -o test.o;
  then
    avr-objcopy -j .text -j .data -O ihex  test.o  test.hex
    avrdude -pt13 -cusbasp -U flash:w:test.hex;
fi
