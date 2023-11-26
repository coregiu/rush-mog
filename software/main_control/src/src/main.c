#include <stdlib.h>
#include "stm32f10x.h"

static volatile uint32_t s_tick_count = 0;

#define LED_PIN         GPIO_Pin_13

void SysTick_Handler(void)
{
    s_tick_count++;
}

static void led_init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitTypeDef gpio_out = {
        .GPIO_Pin = LED_PIN,
        .GPIO_Mode = GPIO_Mode_Out_PP,
        .GPIO_Speed = GPIO_Speed_2MHz,
    };
    GPIO_Init(GPIOC, &gpio_out);
}

static void clock_init(void)
{
    /* Reset the RCC clock configuration to the default reset state. */
    /* HSI ON, PLL OFF, HSE OFF, system clock = 72 MHz, cpu_clock = 72 MHz */
    RCC_DeInit();
    SystemCoreClockUpdate();  /* BluePill board runs at 72 MHz */

    if (SysTick_Config(SystemCoreClock / 1000)) {
        /* Capture error */
        while (1);
    }
}

static void delay_us(int32_t us)
{
    // Each loop takes very roughly one microsecond on a Blue Pill.
    volatile int32_t count = us;
    for (; count > 0; --count)
        ;
}

void delay_ms(uint32_t ms)
{
    uint32_t end_tick = s_tick_count + ms;
    while (s_tick_count < end_tick) {
        __WFE();
    }
}

int main(void)
{
    clock_init();
    led_init();

    while (1) {
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_RESET);
        delay_ms(250);
        GPIO_WriteBit(GPIOC, GPIO_Pin_13, Bit_SET);
        delay_us(500000);
    }
}