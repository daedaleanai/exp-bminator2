# Tool names
PREFIX=arm-none-eabi-
CC          := $(PREFIX)gcc
OBJCOPY     := $(PREFIX)objcopy
SIZE        := $(PREFIX)size

REVISION := $(shell git log -1 --format="%h" || echo "0000000")

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4
OPT_FLAGS	 = -O3 -flto -fuse-linker-plugin -ffunction-sections -fdata-sections -fverbose-asm -ffat-lto-objects -fno-exceptions -fno-unwind-tables
WARN_FLAGS   = -Werror -Wfatal-errors -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef  -Wno-pedantic -Wno-enum-conversion -Wno-deprecated
DEBUG_FLAGS	 = -ggdb3 -DNDEBUG -D__REVISION__='0x$(REVISION)'  
CFLAGS 		 = -std=gnu17 $(ARCH_FLAGS) $(OPT_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS)
LDFLAGS		 = -nostartfiles -lnosys -static $(ARCH_FLAGS) $(OPT_FLAGS) $(WARN_FLAGS) $(DEBUG_FLAGS) -Wl,-gc-sections,-Map,main.map -Wl,--cref

.DEFAULT_GOAL := main.hex

OBJS = \
	vectors.o \
	boot.o \
	fault.o \
	gpio2.o \
	ringbuffer.o \
	tprintf.o \
	runtimer.o \
	spi.o \
	bmxspi.o \
	bme280.o \
	output.o \
	input.o \
	crc16.o \
	main.o \


$(OBJS): Makefile 

# Compile
%.o: %.c
	$(CC) -c -o $@ $(CFLAGS) $<

%.elf: stm32l432kc.ld
%.elf: $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS) -Tstm32l432kc.ld

%.hex: %.elf
	$(SIZE) $<
	$(OBJCOPY) -O ihex --set-start 0x08000000 $< $@

flash: main.hex
	st-flash --connect-under-reset --format ihex write $<
	
flash2: main.elf
	openocd \
		-f interface/stlink.cfg \
		-f target/stm32l4x.cfg \
		-f openocd.cfg

clean:
	rm -f *~ *.o *.hex *.bin *.elf *.map

depend:
	makedepend -Y. -w150 *.c


# DO NOT DELETE

bme280.o: bme280.h
bmxspi.o: stm32l4xx.h binary.h bme280.h bmi08x.h bmxspi.h spi.h clock.h tprintf.h
boot.o: cortex_m4.h stm32l4xx.h clock.h
crc16.o: crc16.h
fault.o: cortex_m4.h stm32l4xx.h
gpio2.o: gpio2.h stm32l4xx.h
input.o: clock.h stm32l4xx.h crc16.h input.h msgq.h binary.h spi.h tprintf.h
main.o: cortex_m4.h stm32l4xx.h binary.h bmi08x.h bmxspi.h bme280.h spi.h clock.h crc16.h gpio2.h input.h msgq.h nvic.h output.h runtimer.h tprintf.h
main.o: usart.h ringbuffer.h
output.o: output.h msgq.h binary.h spi.h stm32l4xx.h bme280.h bmi08x.h bmxspi.h
ringbuffer.o: ringbuffer.h
runtimer.o: runtimer.h clock.h stm32l4xx.h cortex_m4.h tprintf.h
spi.o: spi.h stm32l4xx.h cortex_m4.h nvic.h
tprintf.o: tprintf.h
vectors.o: cortex_m4.h stm32l4xx.h
