SHELL=cmd
CC=c51
PROG = EFM8_prog
COMPORT = $(shell type COMPORT.inc)
OBJS=main.obj
PORTN=$(shell type COMPORT.inc)

main.hex: $(OBJS)
	$(CC) $(OBJS)
	@echo Done!
	
main.obj: main.c
	$(CC) -c main.c

clean:
	@del $(OBJS) *.asm *.lkr *.lst *.map *.hex *.map 2> nul

LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	$(PROG) -ft230 -r main.hex
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

Dummy: main.hex main.Map
	@echo Nothing to see here!
	
explorer:
	cmd /c start explorer .
