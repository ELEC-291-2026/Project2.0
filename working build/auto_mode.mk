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

OBJS=main.o startup.o robot_auto_mode.o collision_detector.o vl53l0x.o

all: $(TARGET).hex

$(TARGET).elf: $(OBJS)
	$(CC) $(OBJS) $(LDFLAGS) -o $(TARGET).elf
	$(SIZE) $(TARGET).elf

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $(TARGET).elf $(TARGET).hex
	@echo Success!

main.o: main.c
	$(CC) -c $(CFLAGS) main.c -o main.o

robot_auto_mode.o: robot_auto_mode.c
	$(CC) -c $(CFLAGS) robot_auto_mode.c -o robot_auto_mode.o

startup.o: ../Common/Source/startup.c
	$(CC) -c $(CFLAGS) ../Common/Source/startup.c -o startup.o

collision_detector.o: collision_detector.c
	$(CC) -c $(CFLAGS) collision_detector.c -o collision_detector.o

vl53l0x.o: ../vl53l0x.c
	$(CC) -c $(CFLAGS) -I.. ../vl53l0x.c -o vl53l0x.o

clean:
	@$(RM_CMD) $(OBJS) 2>$(NULL_OUT)
	@$(RM_CMD) $(TARGET).elf $(TARGET).hex $(TARGET).map 2>$(NULL_OUT)

Flash_Load: $(TARGET).hex
ifeq ($(OS),Windows_NT)
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL 2>NUL
	@for /f %%P in ('.\stm32flash\BO230\BO230.exe -b') do .\stm32flash\stm32flash.exe -w $(TARGET).hex -v -g 0x0 %%P
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@.\stm32flash\BO230\BO230.exe -r >>sputty.bat
	@sputty
else
	@echo Flash_Load is only configured for the Windows stm32flash flow.
endif

putty:
ifeq ($(OS),Windows_NT)
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL 2>NUL
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@.\stm32flash\BO230\BO230.exe -r >>sputty.bat
	@sputty
else
	@echo putty is only configured for the Windows serial flow.
endif
