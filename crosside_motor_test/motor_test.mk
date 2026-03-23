SHELL=cmd

TARGET=motor_test
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

CPUFLAGS=-mcpu=cortex-m0plus -mthumb
CFLAGS=$(CPUFLAGS) -O2 -g -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -Wall -I../Common/Include
LDFLAGS=$(CPUFLAGS) -nostdlib -Wl,--gc-sections -Wl,--cref -Wl,-Map,$(TARGET).map -T ../Common/LDscripts/stm32l051xx_simple.ld -lgcc

OBJS=main.o startup.o

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET).elf
	$(SIZE) $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex
	@echo Success!

main.o: main.c
	$(CC) -c $(CFLAGS) main.c -o main.o

startup.o: ../Common/Source/startup.c
	$(CC) -c $(CFLAGS) ../Common/Source/startup.c -o startup.o

clean:
	@del $(OBJS) 2>NUL
	@del $(TARGET).elf $(TARGET).hex $(TARGET).map 2>NUL

Flash_Load: $(TARGET).hex
	@echo Flash target expects your course stm32flash setup and COM port config.
