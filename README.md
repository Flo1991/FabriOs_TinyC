# FabriOs_TinyC
C Project implementing a very tiny scheduler for AVR Attiny2313. This mcu is one
of the smallest to the AVR family - but it is possible to have a scheduling mechanism 
written in C.

This project is just for fun to see what performance can be achieved from pure C bare metal code.

# Pure C implementation
This project uses a pure C implementation - with several inline assembly where necessary.
This means no external libs are used. Even the startup code is written in C.
In order to get the necessary rjmp instruction for the avr core, some linker script magic is done:
The interrupt handler functions are located at known addresses from that the linker
calulates matching rjmp instructions, which can be used in C code in a interrupt vector array.
Due to avoiding all external libs one will find even basic types and registers defined in the code.

# Memory Usage
avr-size --format=berkeley FabriOs_TinyC/FabriOs_TinyC.elf <br>
   text    data     bss     dec     hex filename <br>
    672      42      42     756     2f4 FabriOs_TinyC/FabriOs_TinyC.elf <br><br>
In addition to the shown memory usage one must add the stack of the main process
which is also around 40 bytes, so have an overall usage of 122 bytes - so this
project proves that even very small devices can have a scheduler - written in C.
If moving to assembly language one could further optimize, because one could limit the
usage of the core registers so that a process context can be smaller.
