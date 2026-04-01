#include "stm32l051xx.h"
#include "collision_detector.h"

extern void delayms(unsigned int ms);
#include "collision_detector_config.h"
#include "../vl53l0x.h"

enum
{
    COLLISION_I2C_TIMEOUT_LOOPS = 500000U
};

static void collision_detector_xshut_write(int enabled)
{
    if (enabled)
    {
        COLLISION_DETECTOR_XSHUT_GPIO_PORT->BSRR = COLLISION_DETECTOR_XSHUT_GPIO_PIN;
    }
    else
    {
        COLLISION_DETECTOR_XSHUT_GPIO_PORT->BRR = COLLISION_DETECTOR_XSHUT_GPIO_PIN;
    }
}

static void collision_detector_xshut_init(void)
{
    COLLISION_DETECTOR_XSHUT_GPIO_CLK_ENABLE();

    GPIOA->MODER &= ~GPIO_MODER_MODE7;
    GPIOA->MODER |= GPIO_MODER_MODE7_0;
    GPIOA->OTYPER &= ~GPIO_OTYPER_OT_7;
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED7;
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7;

    collision_detector_xshut_write(0);
}

static void collision_detector_i2c_init(void)
{
    COLLISION_DETECTOR_I2C_GPIO_CLK_ENABLE();
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    GPIOB->MODER &= ~(GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
    GPIOB->MODER |= GPIO_MODER_MODE6_1 | GPIO_MODER_MODE7_1;

    GPIOB->AFR[0] &= ~((0xFUL << 24U) | (0xFUL << 28U));
    GPIOB->AFR[0] |= (0x1UL << 24U) | (0x1UL << 28U);

    GPIOB->OTYPER |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7;
    GPIOB->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED6 | GPIO_OSPEEDER_OSPEED7);
    GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED6_0 | GPIO_OSPEEDER_OSPEED7_0;
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);

    I2C1->CR1 = 0U;
    I2C1->TIMINGR = COLLISION_DETECTOR_I2C_TIMINGR;
    I2C1->ICR = I2C_ICR_STOPCF |
        I2C_ICR_NACKCF |
        I2C_ICR_BERRCF |
        I2C_ICR_ARLOCF |
        I2C_ICR_OVRCF |
        I2C_ICR_TIMOUTCF;
}

static int collision_i2c_wait_for_flag(uint32_t flag_mask)
{
    unsigned int timeout = COLLISION_I2C_TIMEOUT_LOOPS;

    while (timeout-- > 0U)
    {
        uint32_t status = I2C1->ISR;

        if ((status & flag_mask) != 0U)
        {
            return 1;
        }

        if ((status & (I2C_ISR_NACKF |
            I2C_ISR_BERR |
            I2C_ISR_ARLO |
            I2C_ISR_OVR |
            I2C_ISR_TIMEOUT)) != 0U)
        {
            break;
        }
    }

    return 0;
}

static void collision_i2c_clear_flags(void)
{
    I2C1->ICR = I2C_ICR_STOPCF |
        I2C_ICR_NACKCF |
        I2C_ICR_BERRCF |
        I2C_ICR_ARLOCF |
        I2C_ICR_OVRCF |
        I2C_ICR_TIMOUTCF;
}

unsigned char i2c_write_addr8_data8(unsigned char address, unsigned char value)
{
    collision_i2c_clear_flags();

    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND |
        (2UL << 16U) |
        COLLISION_DETECTOR_I2C_ADDRESS;
    I2C1->CR2 |= I2C_CR2_START;

    if (!collision_i2c_wait_for_flag(I2C_ISR_TXIS))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    I2C1->TXDR = address;

    if (!collision_i2c_wait_for_flag(I2C_ISR_TXIS))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    I2C1->TXDR = value;

    if (!collision_i2c_wait_for_flag(I2C_ISR_STOPF))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    collision_i2c_clear_flags();
    return 1U;
}

unsigned char i2c_read_addr8_data8(unsigned char address, unsigned char *value)
{
    collision_i2c_clear_flags();

    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND |
        (1UL << 16U) |
        COLLISION_DETECTOR_I2C_ADDRESS;
    I2C1->CR2 |= I2C_CR2_START;

    if (!collision_i2c_wait_for_flag(I2C_ISR_TXIS))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    I2C1->TXDR = address;

    if (!collision_i2c_wait_for_flag(I2C_ISR_STOPF))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    collision_i2c_clear_flags();
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND |
        I2C_CR2_RD_WRN |
        (1UL << 16U) |
        COLLISION_DETECTOR_I2C_ADDRESS;
    I2C1->CR2 |= I2C_CR2_START;

    if (!collision_i2c_wait_for_flag(I2C_ISR_RXNE))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    *value = (unsigned char)I2C1->RXDR;

    I2C1->ICR = I2C_ICR_NACKCF; /* master NACK before STOP is normal in read mode */

    if (!collision_i2c_wait_for_flag(I2C_ISR_STOPF))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    collision_i2c_clear_flags();
    return 1U;
}

unsigned char i2c_read_addr8_data16(unsigned char address, unsigned short *value)
{
    collision_i2c_clear_flags();

    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND |
        (1UL << 16U) |
        COLLISION_DETECTOR_I2C_ADDRESS;
    I2C1->CR2 |= I2C_CR2_START;

    if (!collision_i2c_wait_for_flag(I2C_ISR_TXIS))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    I2C1->TXDR = address;

    if (!collision_i2c_wait_for_flag(I2C_ISR_STOPF))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    collision_i2c_clear_flags();
    I2C1->CR1 = I2C_CR1_PE;
    I2C1->CR2 = I2C_CR2_AUTOEND |
        I2C_CR2_RD_WRN |
        (2UL << 16U) |
        COLLISION_DETECTOR_I2C_ADDRESS;
    I2C1->CR2 |= I2C_CR2_START;

    if (!collision_i2c_wait_for_flag(I2C_ISR_RXNE))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    *value = (unsigned short)(I2C1->RXDR << 8U);

    if (!collision_i2c_wait_for_flag(I2C_ISR_RXNE))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    *value = (unsigned short)(*value | (unsigned short)I2C1->RXDR);

    I2C1->ICR = I2C_ICR_NACKCF; /* master NACK before STOP is normal in read mode */

    if (!collision_i2c_wait_for_flag(I2C_ISR_STOPF))
    {
        collision_i2c_clear_flags();
        return 0U;
    }

    collision_i2c_clear_flags();
    return 1U;
}

void collision_detector_reset(collision_detector_t *detector)
{
    detector->distance_mm = 0U;
    detector->initialized = 0;
    detector->measurement_valid = 0;
    detector->obstacle_detected = 0;
}

int collision_detector_init(collision_detector_t *detector)
{
    collision_detector_reset(detector);
    collision_detector_xshut_init();
    collision_detector_i2c_init();

    delayms(COLLISION_DETECTOR_RESET_DELAY_MS);
    collision_detector_xshut_write(1);
    delayms(COLLISION_DETECTOR_BOOT_DELAY_MS);

    if (!vl53l0x_init())
    {
        return 0;
    }

    if (!vl53l0x_start_continuous())
    {
        return 0;
    }

    detector->initialized = 1;
    return 1;
}

void collision_detector_update(collision_detector_t *detector)
{
    uint16_t distance_mm;

    if ((detector == 0) || !detector->initialized)
    {
        return;
    }

    if (!vl53l0x_measurement_ready())
    {
        return;
    }

    if (!vl53l0x_read_range_continuous(&distance_mm))
    {
        detector->measurement_valid = 0;
        detector->obstacle_detected = 0;
        return;
    }

    detector->distance_mm = distance_mm;
    detector->measurement_valid = (distance_mm != VL53L0X_OUT_OF_RANGE);
    detector->obstacle_detected = detector->measurement_valid &&
        (distance_mm < COLLISION_DETECTOR_OBSTACLE_THRESHOLD_MM);
}
