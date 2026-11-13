/**
  ******************************************************************************
  * JGB37-520 四路增量式编码器：EXIT 四倍频计数、速度/里程计算、UART 遥测。
  ******************************************************************************
**/

#include <encoder_sensor.h>

#define ENCODER_FL 0
#define ENCODER_FR 1
#define ENCODER_RL 2
#define ENCODER_RR 3

#define ENCODER_TELEMETRY_BUFFER_SIZE 96

/* 编码器 EXTI 抢占优先级。必须比 USART2/USART3(3) 低（数值更大），
 * 否则编码器边沿密集时会压住 USART3 的收数中断，命令可能丢字节。 */
#define ENCODER_EXTI_PREEMPT_PRIORITY 4

struct encoder_pins
{
    GPIO_TypeDef *port_a;
    uint16_t pin_a;
    GPIO_TypeDef *port_b;
    uint16_t pin_b;
};

/* A/B 相与电机对应关系：FL, FR, RL, RR，依次使用 PA0~PA7。
 * 注意：PA2/PA3 同时是 USART2 的 TX/RX（语音板 ASR PRO / JQ6500），
 *       改用这组引脚后语音与 MP3 功能会失效，详见 encoder_sensor.h。 */
static const struct encoder_pins s_encoder_pins[ENCODER_COUNT] = {
    {GPIOA, GPIO_Pin_0, GPIOA, GPIO_Pin_1},
    {GPIOA, GPIO_Pin_2, GPIOA, GPIO_Pin_3},
    {GPIOA, GPIO_Pin_4, GPIOA, GPIO_Pin_5},
    {GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7},
};

static const int8_t s_encoder_sign[ENCODER_COUNT] = {
    ENCODER_SIGN_FL,
    ENCODER_SIGN_FR,
    ENCODER_SIGN_RL,
    ENCODER_SIGN_RR,
};

/*
 * 正交解码表，索引 = (上一状态 << 2) | 当前状态。
 * 状态编码：(A << 1) | B。
 * 正常正转序列：00 -> 01 -> 11 -> 10 -> 00。
 */
static const int8_t s_quadrature_table[16] = {
    0, 1, -1, 0,
    -1, 0, 0, 1,
    1, 0, 0, -1,
    0, -1, 1, 0,
};

static volatile int32_t s_encoder_counts[ENCODER_COUNT];
static volatile uint8_t s_encoder_state[ENCODER_COUNT];

/* 项目没有编译标准库的 stm32f10x_exti.c，这里直接操作 EXTI 寄存器，
 * 避免为几个小函数额外链接整个外设库。 */
static inline void encoder_exti_enable(uint32_t lines)
{
    EXTI->IMR |= lines;
    EXTI->EMR &= ~lines;
    EXTI->RTSR |= lines;
    EXTI->FTSR |= lines;
    EXTI->PR = lines;
}

static inline uint8_t encoder_exti_pending(uint32_t line)
{
    return (EXTI->PR & line) ? 1 : 0;
}

static inline void encoder_exti_clear(uint32_t line)
{
    EXTI->PR = line;
}

static inline uint8_t encoder_read_state(uint8_t index)
{
    const struct encoder_pins *pins = &s_encoder_pins[index];
    uint8_t a = GPIO_ReadInputDataBit(pins->port_a, pins->pin_a) ? 1 : 0;
    uint8_t b = GPIO_ReadInputDataBit(pins->port_b, pins->pin_b) ? 1 : 0;
    return (uint8_t)((a << 1) | b);
}

/* 任一路 A/B 相跳变时调用：读当前 A/B，按状态表累加 ±1。 */
static inline void encoder_update(uint8_t index)
{
    uint8_t current = encoder_read_state(index);
    uint8_t previous = s_encoder_state[index];
    int8_t delta = s_quadrature_table[(previous << 2) | current];
    s_encoder_state[index] = current;
    if (delta != 0)
    {
        s_encoder_counts[index] += delta * s_encoder_sign[index];
    }
}

void encoder_sensor_get_counts(int32_t counts[ENCODER_COUNT])
{
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        counts[i] = s_encoder_counts[i];
    }
    taskEXIT_CRITICAL();
}

/* 阻塞发送一帧；9600 波特率下约 40~55ms。遥测任务优先级较低，
 * 发送期间命令任务仍可抢占 CPU，不影响电机控制。 */
static void encoder_uart_write(const char *data, uint16_t length)
{
    for (uint16_t i = 0; i < length; i++)
    {
        uint32_t guard = 0;
        while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET)
        {
            if (++guard > 1000000U)
            {
                return;
            }
        }
        USART_SendData(USART3, (uint8_t)data[i]);
    }

    uint32_t guard = 0;
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        if (++guard > 1000000U)
        {
            return;
        }
    }
}

/* 固件不链接 newlib printf，手动格式化，避免几 KB 的 stdio/syscall 代码。 */
static uint16_t encoder_append_uint(char *buffer, uint16_t position, uint32_t value)
{
    char digits[10];
    uint8_t count = 0;
    if (value == 0)
    {
        buffer[position++] = '0';
        return position;
    }
    while (value > 0 && count < sizeof(digits))
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    while (count > 0)
    {
        buffer[position++] = digits[--count];
    }
    return position;
}

static uint16_t encoder_append_int(char *buffer, uint16_t position, int32_t value)
{
    if (value < 0)
    {
        buffer[position++] = '-';
        return encoder_append_uint(buffer, position, (uint32_t)(-(int64_t)value));
    }
    return encoder_append_uint(buffer, position, (uint32_t)value);
}

static void encoder_telemetry_task(void *argument)
{
    (void)argument;

    int32_t last_counts[ENCODER_COUNT];
    encoder_sensor_get_counts(last_counts);

    TickType_t last_tick = xTaskGetTickCount();
    uint32_t sequence = 0;
    int64_t total_abs_counts = 0;
    char line[ENCODER_TELEMETRY_BUFFER_SIZE];

    for (;;)
    {
        vTaskDelay(pdMS_TO_TICKS(ENCODER_TELEMETRY_PERIOD_MS));

        TickType_t now = xTaskGetTickCount();
        uint32_t dt_ms = (uint32_t)((now - last_tick) * portTICK_PERIOD_MS);
        if (dt_ms == 0)
        {
            continue;
        }
        last_tick = now;

        int32_t counts[ENCODER_COUNT];
        encoder_sensor_get_counts(counts);

        int32_t wheel_mm_s[ENCODER_COUNT];
        for (uint8_t i = 0; i < ENCODER_COUNT; i++)
        {
            int32_t delta = counts[i] - last_counts[i];
            last_counts[i] = counts[i];

            /* delta_um 是本次周期内该轮走过的微米数；
             * um / ms 数值上恰好等于 mm/s。 */
            int32_t delta_um = (int32_t)(((int64_t)delta * ENCODER_UM_X10_PER_COUNT) / 10);
            wheel_mm_s[i] = delta_um / (int32_t)dt_ms;

            total_abs_counts += (delta >= 0) ? delta : -delta;
        }

        /* 四轮绝对位移取平均，既能统计前进/后退，也能统计横移/原地转向。 */
        int32_t distance_mm = (int32_t)((total_abs_counts * ENCODER_UM_X10_PER_COUNT) / 40000);

        sequence++;
        uint32_t uptime_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        uint16_t position = 0;
        line[position++] = '$';
        line[position++] = 'B';
        line[position++] = ',';
        position = encoder_append_uint(line, position, sequence);
        line[position++] = ',';
        position = encoder_append_uint(line, position, uptime_ms);
        line[position++] = ',';
        position = encoder_append_int(line, position, wheel_mm_s[ENCODER_FL]);
        line[position++] = ',';
        position = encoder_append_int(line, position, wheel_mm_s[ENCODER_FR]);
        line[position++] = ',';
        position = encoder_append_int(line, position, wheel_mm_s[ENCODER_RL]);
        line[position++] = ',';
        position = encoder_append_int(line, position, wheel_mm_s[ENCODER_RR]);
        line[position++] = ',';
        position = encoder_append_int(line, position, distance_mm);
        line[position++] = '\n';
        encoder_uart_write(line, position);
    }
}

static void encoder_exti_init(void)
{
    GPIO_InitTypeDef gpio;
    NVIC_InitTypeDef nvic;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* PA0~PA7 全部作为编码器输入，内部上拉。
     * 不再使用 PA15/PB3/PB4，因此不需要关闭 JTAG。 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    for (uint8_t i = 0; i < ENCODER_COUNT; i++)
    {
        s_encoder_state[i] = encoder_read_state(i);
        s_encoder_counts[i] = 0;
    }

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource3);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource4);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource5);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource7);

    encoder_exti_enable(EXTI_Line0 | EXTI_Line1 | EXTI_Line2 | EXTI_Line3 |
                        EXTI_Line4 | EXTI_Line5 | EXTI_Line6 | EXTI_Line7);

    /* 与 audio/ps2 初始化保持一致：4 位抢占优先级，无子优先级。 */
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    nvic.NVIC_IRQChannelPreemptionPriority = ENCODER_EXTI_PREEMPT_PRIORITY;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    nvic.NVIC_IRQChannel = EXTI0_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = EXTI2_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = EXTI3_IRQn;
    NVIC_Init(&nvic);

    nvic.NVIC_IRQChannel = EXTI4_IRQn;
    NVIC_Init(&nvic);

    /* EXTI5~EXTI7 共用 EXTI9_5 向量。 */
    nvic.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_Init(&nvic);
}

void encoder_sensor_init(void)
{
    encoder_exti_init();

    BaseType_t created = xTaskCreate(encoder_telemetry_task, "ENC_TLM", 192, NULL, 1, NULL);
    if (created != pdPASS)
    {
        /* 堆不足时保持编码器计数与中断工作，只是不发送遥测。 */
        return;
    }
}

/* ---------------- EXTI 中断服务函数 ----------------
 * FL = PA0(A) / PA1(B) -> EXTI0 / EXTI1
 * FR = PA2(A) / PA3(B) -> EXTI2 / EXTI3
 * RL = PA4(A) / PA5(B) -> EXTI4 / EXTI9_5(line5)
 * RR = PA6(A) / PA7(B) -> EXTI9_5(line6/line7)
 */

void EXTI0_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line0))
    {
        encoder_exti_clear(EXTI_Line0);
        encoder_update(ENCODER_FL);
    }
}

void EXTI1_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line1))
    {
        encoder_exti_clear(EXTI_Line1);
        encoder_update(ENCODER_FL);
    }
}

void EXTI2_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line2))
    {
        encoder_exti_clear(EXTI_Line2);
        encoder_update(ENCODER_FR);
    }
}

void EXTI3_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line3))
    {
        encoder_exti_clear(EXTI_Line3);
        encoder_update(ENCODER_FR);
    }
}

void EXTI4_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line4))
    {
        encoder_exti_clear(EXTI_Line4);
        encoder_update(ENCODER_RL);
    }
}

void EXTI9_5_IRQHandler(void)
{
    if (encoder_exti_pending(EXTI_Line5))
    {
        encoder_exti_clear(EXTI_Line5);
        encoder_update(ENCODER_RL);
    }
    if (encoder_exti_pending(EXTI_Line6))
    {
        encoder_exti_clear(EXTI_Line6);
        encoder_update(ENCODER_RR);
    }
    if (encoder_exti_pending(EXTI_Line7))
    {
        encoder_exti_clear(EXTI_Line7);
        encoder_update(ENCODER_RR);
    }
}
