#include <stdio.h>
#include <stdint.h>
#include "../../Common/Include/stm32l051xx.h"
#include "vl53l0x.h"

#define F_CPU 32000000L

void wait_1ms(void)
{
    SysTick->LOAD = (F_CPU / 1000L) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
    while ((SysTick->CTRL & BIT16) == 0);
    SysTick->CTRL = 0x00;
}

void waitms(int ms)
{
    while (ms--) wait_1ms();
}

void I2C_init(void)
{
    RCC->IOPENR  |= 0x02;              // GPIOB clock on
    RCC->APB1ENR |= (1 << 21);        // I2C1 clock on

    GPIOB->MODER &= ~(0x3 << 12);     // PB6 alternate function
    GPIOB->MODER |=  (0x2 << 12);
    GPIOB->AFR[0] &= ~(0xF << 24);    // PB6 = AF1 (I2C1)
    GPIOB->AFR[0] |=  (0x1 << 24);

    GPIOB->MODER &= ~(0x3 << 14);     // PB7 alternate function
    GPIOB->MODER |=  (0x2 << 14);
    GPIOB->AFR[0] &= ~(0xF << 28);    // PB7 = AF1 (I2C1)
    GPIOB->AFR[0] |=  (0x1 << 28);

    GPIOB->OTYPER |= (1 << 6) | (1 << 7);  // open-drain
    GPIOB->OSPEEDR &= ~(0x3 << 12);         // medium speed PB6
    GPIOB->OSPEEDR |=  (0x1 << 12);
    GPIOB->OSPEEDR &= ~(0x3 << 14);         // medium speed PB7
    GPIOB->OSPEEDR |=  (0x1 << 14);

    I2C1->TIMINGR = 0x70420f13;        // 100kHz @ 32MHz
}

unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (2 << 16) | 0x52;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->TXDR = address;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->TXDR = value;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    waitms(1);
    return 1;
}

unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char *value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | 0X52;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->TXDR = address;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    waitms(1);
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | 0X52 | I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    *value = I2C1->RXDR;
    waitms(1);
    return 1;
}

unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short *value)
{
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (1 << 16) | 0X52;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    I2C1->TXDR = address;
    while (!(I2C1->ISR & I2C_ISR_TXE));
    waitms(1);
    I2C1->CR1 = I2C_CR1_PE | I2C_CR1_RXIE;
    I2C1->CR2 = I2C_CR2_AUTOEND | (2 << 16) | 0X52 | I2C_CR2_RD_WRN;
    I2C1->CR2 |= I2C_CR2_START;
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    *value = I2C1->RXDR * 256;
    while (!(I2C1->ISR & I2C_ISR_RXNE));  // wait for second byte
    *value+= I2C1->RXDR;
    waitms(1);
    return 1;
}

// -------------------------------------------------------
// main: init everything, then read distance in a loop
// -------------------------------------------------------
void main(void)
{
    uint16_t distance;
    waitms(500);

    I2C_init();

    if (!vl53l0x_init())
    {
        // init failed - check your wiring
        while (1);
    }

    vl53l0x_start_continuous();

    while (1)
    {
        if (vl53l0x_measurement_ready())
        {
            vl53l0x_read_range_continuous(&distance);

            if (distance == VL53L0X_OUT_OF_RANGE)
            {
                printf("Out of range\r\n");
            }
            else
            {
                printf("Distance: %d mm\r\n", distance);
                // obstacle threshold: if distance < 200, stop motors, need to write code to stop motors
                if(distance < 200){
                    printf("OBSTACLE\r\n");
                    hbridge_motor_stop_all();
                }
            }
        }
    }
}
