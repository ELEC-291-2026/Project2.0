#include <stdio.h>
#include "../Common/Include/stm32l051xx.h"
#include "../Common/Include/serial.h"
#include "vl53l0x.h"

// LQFP32 pinout
//              ----------
//        VDD -|1       32|- VSS
//       PC14 -|2       31|- BOOT0
//       PC15 -|3       30|- PB7 (I2C1_SDA)
//       NRST -|4       29|- PB6 (I2C1_SCL)
//       VDDA -|5       28|- PB5
//        PA0 -|6       27|- PB4
//        PA1 -|7       26|- PB3
//        PA2 -|8       25|- PA15
//        PA3 -|9       24|- PA14
//        PA4 -|10      23|- PA13
//        PA5 -|11      22|- PA12
//        PA6 -|12      21|- PA11
//        PA7 -|13      20|- PA10 (Reserved for RXD)
//        PB0 -|14      19|- PA9  (Reserved for TXD)
//        PB1 -|15      18|- PA8
//        VSS -|16      17|- VDD
//              ----------

#define F_CPU 32000000L
#define NUNCHUK_ADDRESS 0x52

void wait_1ms(void)
{
	// For SysTick info check the STM32l0xxx Cortex-M0 programming manual.
	SysTick->LOAD = (F_CPU/1000L) - 1;  // set reload register, counter rolls over from zero, hence -1
	SysTick->VAL = 0; // load the SysTick counter
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk; // Enable SysTick IRQ and SysTick Timer */
	while((SysTick->CTRL & BIT16)==0); // Bit 16 is the COUNTFLAG.  True when counter rolls over from zero.
	SysTick->CTRL = 0x00; // Disable Systick counter
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

void I2C_init (void)
{
	RCC->IOPENR  |= BIT1;// peripheral clock enable for port B (I2C pins are in port B)
	RCC->APB1ENR  |= BIT21; // peripheral clock enable for I2C1 (page 177 of RM0451 Reference manual)
	
	//Configure PB6 for I2C1_SCL, pin 29 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT12|BIT13)) | BIT13; // PB6 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT24; // AF1 selected (page 204 of RM0451 Reference manual)
	
	//Configure PB7 for I2C1_SDA, pin 30 in LQFP32 package (page 44 of datasheet en.DM00108219.pdf)
	GPIOB->MODER = (GPIOB->MODER & ~(BIT14|BIT15)) | BIT15; // PB7 AF-Mode (page 200 of RM0451 Reference manual)
	GPIOB->AFR[0] |= BIT28; // AF1 selected (page 204 of RM0451 Reference manual)
	
	GPIOB->OTYPER   |= BIT6 | BIT7; // I2C pins (PB6 and PB7)  must be open drain
	GPIOB->OSPEEDR  |= BIT12 | BIT14; // Medium speed for PB6 and PB7

	// This configures the I2C clock (Check page 564 of RM0451).  SCLK must be 100kHz or less.
	I2C1->TIMINGR = (uint32_t)0x70420f13;
}

unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char * value)
{
	
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | I2C_CR2_RD_WRN | (0x52); // Read one byte from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR; // Reading 'receive' register clears RXNE flag

	waitms(1);

	return 1;
}

unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short * value)
{
	// First we send the address we want to read from:
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START; // Go
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);
	
	// Second: we gatter the data sent by the slave device
	I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2<<16) | I2C_CR2_RD_WRN | (0x52); // Read two bytes from slave 0x52
	I2C1->CR2 |= I2C_CR2_START; // Go
    
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value=I2C1->RXDR*256; // Reading 'receive' register clears RXNE flag
   
	while ((I2C1->ISR & I2C_ISR_RXNE) != I2C_ISR_RXNE) {} // Wait for data to arrive
	*value+=I2C1->RXDR; // Reading 'receive' register clears RXNE flag
    
	waitms(1);
	
	return 1;
}

unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
	I2C1->CR1 = I2C_CR1_PE;
	I2C1->CR2 = I2C_CR2_AUTOEND | (2 << 16) | (0x52);
	I2C1->CR2 |= I2C_CR2_START;
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Wait for address to go

	I2C1->TXDR = address; // send register address
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	I2C1->TXDR = value; // send data
	while ((I2C1->ISR & I2C_ISR_TXE) != I2C_ISR_TXE) {} // Check Tx empty

	waitms(1);

	return 1;
}

// From the VL53L0X datasheet:
//
// The registers shown in the table below can be used to validate the user I2C interface.
// Address (After fresh reset, without API loaded)
//    0xC0 0xEE
//    0xC1 0xAA
//    0xC2 0x10
//    0x51 0x0099 (0x0096 after initialization)
//    0x61 0x0000
//
// Not needed, but it was useful to debug the I2C interface, so left here.
void validate_I2C_interface (void)
{
	unsigned char val8 = 0;
	unsigned short val16 = 0;
	
    printf("\n");   
    
    i2c_read_addr8_data8(0xc0, &val8);
    printf("Reg(0xc0): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc1, &val8);
    printf("Reg(0xc1): 0x%02x\r\n", val8);

    i2c_read_addr8_data8(0xc2, &val8);
    printf("Reg(0xc2): 0x%02x\r\n", val8);
    
    i2c_read_addr8_data16(0x51, &val16);
    printf("Reg(0x51): 0x%04x\r\n", val16);

    i2c_read_addr8_data16(0x61, &val16);
    printf("Reg(0x61): 0x%04x\r\n", val16);
    
    printf("\r\n");
}

void main(void)
{
	unsigned char success;
	unsigned short range=0;
	
	waitms(500);
	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printf ("STM32L051 I2C vl53l0x test program\r\n"
	        "File: %s\r\n"
	        "Compiled: %s, %s\r\n",
	        __FILE__, __DATE__, __TIME__);
	
	I2C_init();

    validate_I2C_interface();

	success = vl53l0x_init();
	if(success)
	{
		printf("VL53L0x initialization succeeded.\r\n");
	}
	else
	{
		printf("VL53L0x initialization failed.\r\n");
	}
	
    while (1)
    {
        success = vl53l0x_read_range_single(&range);
		if(success)
		{
	        printf("D: %4u (mm)\r", range);
	        // The implementation of printf() in newlib doesn't transmit until a '\n'
	        // is found or the buffer is full, so flush(stdout); to transmit now!!!
	        fflush(stdout); 
	    	waitms(100);
		}
	}	
}
