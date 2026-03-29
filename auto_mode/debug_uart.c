#include "main.h"
#include "debug_uart.h"

enum
{
    DEBUG_UART_BAUDRATE = 115200U,
    DEBUG_UART_CLOCK_HZ = 32000000U
};

void debug_uart_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED9 | GPIO_OSPEEDER_OSPEED10;
    GPIOA->OTYPER &= ~(BIT9 | BIT10);

    GPIOA->MODER &= ~(GPIO_MODER_MODE9 | GPIO_MODER_MODE10);
    GPIOA->MODER |= GPIO_MODER_MODE9_1 | GPIO_MODER_MODE10_1;

    GPIOA->AFR[1] &= ~((0xFUL << 4U) | (0xFUL << 8U));
    GPIOA->AFR[1] |= (0x4UL << 4U) | (0x4UL << 8U);

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = DEBUG_UART_CLOCK_HZ / DEBUG_UART_BAUDRATE;
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void debug_uart_write_char(char character)
{
    while ((USART1->ISR & USART_ISR_TXE) == 0U)
    {
    }

    USART1->TDR = (uint32_t)(unsigned char)character;
}

void debug_uart_write_string(const char *text)
{
    while (*text != '\0')
    {
        debug_uart_write_char(*text);
        ++text;
    }
}

void debug_uart_write_uint(unsigned int value)
{
    char digits[10];
    unsigned int digit_count;

    if (value == 0U)
    {
        debug_uart_write_char('0');
        return;
    }

    digit_count = 0U;

    while (value > 0U)
    {
        digits[digit_count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }

    while (digit_count > 0U)
    {
        --digit_count;
        debug_uart_write_char(digits[digit_count]);
    }
}

void debug_uart_write_int(int value)
{
    unsigned int magnitude;

    if (value < 0)
    {
        debug_uart_write_char('-');
        magnitude = (unsigned int)(-(value + 1)) + 1U;
    }
    else
    {
        magnitude = (unsigned int)value;
    }

    debug_uart_write_uint(magnitude);
}
