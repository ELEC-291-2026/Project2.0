ifeq ($(OS),Windows_NT)
SHELL=cmd
RM_CMD=del /Q
NULL_OUT=NUL
else
SHELL=/bin/sh
RM_CMD=rm -f
NULL_OUT=/dev/null
endif

TARGET=auto_mode
CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
SIZE=arm-none-eabi-size

CPUFLAGS=-mcpu=cortex-m0plus -mthumb
CFLAGS=$(CPUFLAGS) -O2 -g -ffunction-sections -fdata-sections -ffreestanding -fno-builtin -Wall -Wextra -I. -I../Common/Include
LDFLAGS=$(CPUFLAGS) -nostdlib -Wl,--gc-sections -Wl,--cref -Wl,-Map,$(TARGET).map -T ../Common/LDscripts/stm32l051xx_simple.ld -lgcc

OBJS=main.o board.o field_sensor_adc.o hbridge_motor.o robot_auto_mode.o startup.o

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET).elf
	$(SIZE) $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex
	@echo Success!

main.o: main.c
	$(CC) -c $(CFLAGS) main.c -o main.o

board.o: board.c
	$(CC) -c $(CFLAGS) board.c -o board.o

field_sensor_adc.o: field_sensor_adc.c
	$(CC) -c $(CFLAGS) field_sensor_adc.c -o field_sensor_adc.o

hbridge_motor.o: hbridge_motor.c
	$(CC) -c $(CFLAGS) hbridge_motor.c -o hbridge_motor.o

robot_auto_mode.o: robot_auto_mode.c
	$(CC) -c $(CFLAGS) robot_auto_mode.c -o robot_auto_mode.o

startup.o: ../Common/Source/startup.c
	$(CC) -c $(CFLAGS) ../Common/Source/startup.c -o startup.o

clean:
	@$(RM_CMD) $(OBJS) 2>$(NULL_OUT)
	@$(RM_CMD) $(TARGET).elf $(TARGET).hex $(TARGET).map 2>$(NULL_OUT)

Flash_Load: $(TARGET).hex
ifeq ($(OS),Windows_NT)
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL 2>NUL
	@echo .\stm32flash\stm32flash.exe -w $(TARGET).hex -v -g 0x0 ^^>loadf.bat
	@.\stm32flash\BO230\BO230.exe -b >>loadf.bat
	@loadf.bat
else
	@echo Flash_Load is only configured for the Windows stm32flash flow.
endif
