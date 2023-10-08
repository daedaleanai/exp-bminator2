# Tool names
PREFIX=arm-none-eabi-
CC          := $(PREFIX)gcc
OBJCOPY     := $(PREFIX)objcopy
SIZE        := $(PREFIX)size

REVISION := $(shell git log -1 --format="%h" || echo "<NONE>")

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4
LTO_FLAGS	 = -O3 -flto -fuse-linker-plugin -ffunction-sections -fdata-sections -fverbose-asm -ffat-lto-objects -fno-exceptions -fno-unwind-tables
WARN_FLAGS   = -Werror -Wfatal-errors -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef  -Wno-pedantic -Wno-enum-conversion -Wno-deprecated
DEBUG_FLAGS	 = -ggdb3 -DNDEBUG -D__REVISION__='"$(REVISION)"' 
CFLAGS 		 = -std=gnu99 $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS		 = -nostartfiles -lnosys -static $(ARCH_FLAGS) $(LTO_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS) -Wl,-gc-sections,-Map,main.map -Wl,--cref

.DEFAULT_GOAL := main.hex

OBJS = \
	vectors.o \
	boot.o \
	fault.o \
	clock.o \
	gpio2.o \
	ringbuffer.o \
	tprintf.o \
	usart.o \
	main.o \

$(OBJS): Makefile 

# Compile
%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

%.elf: stm32l43x.ld
%.elf: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) -Tstm32l43x.ld

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex --set-start 0x08000000 $< $@

flash: main.hex
	st-flash --connect-under-reset --format ihex write $<

debug: main.elf
	st-util &
	$(PREFIX)gdb $^
	killall st-util
	
clean:
	rm -f *~ *.o *.hex *.bin *.elf *.map

depend:
	makedepend -Y. -w150 *.c
# DO NOT DELETE

