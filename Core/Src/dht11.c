#include "dht11.h"
// [RTOS 改动] 引入 uC/OS-II 总头文件，用于在 20ms 复位脉冲期间释放 CPU，以及在微秒级采样时锁定调度器
#include "includes.h"

/**
 * @brief 微秒级高精度延时函数 (基于 Cortex-M4 硬件 SysTick 滴答定时器倒计数)
 * @param nus 延时微秒数
 */
/**
 * @brief 微秒级高精度延时函数 (基于 Cortex-M4 内核 DWT 周期计数器 CYCCNT)
 * @note  【面向 RTOS 架构精髓】：
 *        绝不争抢 SysTick！SysTick 完整留给 uC/OS-II 操作系统作为调度心跳 (OS Tick)
 *        微秒延时由 DWT 硬件计数器独立提供，每走 1 个数精准对应 1/(168MHz) = 5.95ns！
 * @param nus 延时微秒数
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks = nus * (SystemCoreClock / 1000000); // 1us 对应 168 个 CPU 周期
    uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < ticks);
}

/**
 * @brief 动态切换 PC13 为推挽输出模式 (主机发起始信号)
 */
void DHT11_IO_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief 动态切换 PC13 为上拉输入模式 (接收 DHT11 响应与 40-bit 数据)
 */
void DHT11_IO_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT11_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief DHT11 20ms 低电平复位脉冲 (RTOS 下主动让出 CPU)
 */
static void DHT11_ResetLow20ms(void)
{
    DHT11_IO_OUT();
    DHT11_DQ_OUT_LOW();
    // [原裸机代码保留] HAL_Delay(20);
    // [RTOS 改动] OS 运行后调用 OSTimeDlyHMSM 释放 CPU 20ms；OSStart() 前初始化仍用 HAL_Delay
    if (OSRunning == OS_TRUE)
    {
        OSTimeDlyHMSM(0, 0, 0, 20);
    }
    else
    {
        HAL_Delay(20);
    }
}

/**
 * @brief 复位 DHT11 (主机拉低至少 18ms 发起握手)
 */
void DHT11_Rst(void)
{
    DHT11_ResetLow20ms();
    DHT11_DQ_OUT_HIGH();
    delay_us(30);           // 主机拉高等待 20~40us
}

/**
 * @brief 等待 DHT11 应答响应
 * @retval 0-成功检测到响应, 1-未检测到 (超时)
 */
uint8_t DHT11_Check(void)
{
    uint8_t retry = 0;
    DHT11_IO_IN();
    // 等待 DHT11 拉低 (40~50us)
    while (DHT11_DQ_IN() && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    if (retry >= 100) return 1;
    
    retry = 0;
    // 等待 DHT11 释放拉高 (40~50us)
    while (!DHT11_DQ_IN() && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    if (retry >= 100) return 1;
    
    return 0;
}

/**
 * @brief 从 DHT11 接收读取 1 个位 (bit)
 * @retval 0 或 1
 */
uint8_t DHT11_Read_Bit(void)
{
    uint8_t retry = 0;
    // 每 bit 开始前 DHT11 会有 50us 低电平
    while (DHT11_DQ_IN() && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    retry = 0;
    // 等待引脚变高电平
    while (!DHT11_DQ_IN() && retry < 100)
    {
        retry++;
        delay_us(1);
    }
    // 延时 40us 进行采样：
    // 如果是 '0'，高电平持续 26~28us，40us 后引脚已回到低电平
    // 如果是 '1'，高电平持续 70us，40us 后引脚仍为高电平
    delay_us(40);
    if (DHT11_DQ_IN()) return 1;
    else return 0;
}

/**
 * @brief 从 DHT11 读取 1 个字节 (8 bits)
 * @retval 读取到的字节数据
 */
uint8_t DHT11_Read_Byte(void)
{
    uint8_t i, dat = 0;
    for (i = 0; i < 8; i++)
    {
        dat <<= 1;
        dat |= DHT11_Read_Bit();
    }
    return dat;
}

/**
 * @brief 读取一次完整温湿度数据 (40-bit 数据帧包含 5 个字节)
 * @param temp 返回温度整数值 (℃)
 * @param humi 返回湿度整数值 (%RH)
 * @retval 0-成功, 1-校验或通信失败
 */
/* ==================== [原裸机代码注释保留 - DHT11_Read_Data] ====================
 * [改动原因] DHT11 单总线位判决依赖 DWT 精确 40us 采样窗口；在抢占式 RTOS 下若读 bit 期间被
 *            高优先级任务或软定时器抢占切走，会直接导致时序超差、校验和失败。
 *            RTOS 下将 20ms 复位让出 CPU，并对后续 ~4ms 微秒级握手+收帧加 OSSchedLock() 保护
 *            (不关闭硬件中断，TIM6/TIM7 报警与 SysTick 仍正常触发)。
 */
#if 0
uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5];
    uint8_t i;
    DHT11_Rst();
    if (DHT11_Check() == 0)
    {
        for (i = 0; i < 5; i++)
        {
            buf[i] = DHT11_Read_Byte();
        }
        // 校验和校验：前4字节累加求和后取低8位必须等于第5字节
        if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0]; // 湿度整数部分
            *temp = buf[2]; // 温度整数部分
            return 0;
        }
    }
    return 1;
}
#endif

uint8_t DHT11_Read_Data(uint8_t *temp, uint8_t *humi)
{
    uint8_t buf[5];
    uint8_t i;
    uint8_t ok = 1;

    // 1. 主机拉低 >=18ms 阶段允许任务切换，释放 CPU 20ms
    DHT11_ResetLow20ms();

    // 2. 从释放总线等待 DHT11 响应开始，到 40-bit 接收结束 (~4ms)，锁定任务调度防抢占
    if (OSRunning == OS_TRUE)
    {
        OSSchedLock();
    }

    DHT11_DQ_OUT_HIGH();
    delay_us(30);
    if (DHT11_Check() == 0)
    {
        for (i = 0; i < 5; i++)
        {
            buf[i] = DHT11_Read_Byte();
        }
        if ((uint8_t)(buf[0] + buf[1] + buf[2] + buf[3]) == buf[4])
        {
            *humi = buf[0];
            *temp = buf[2];
            ok = 0;
        }
    }

    if (OSRunning == OS_TRUE)
    {
        OSSchedUnlock();
    }

    return ok;
}

/**
 * @brief DHT11 初始化检测
 * @retval 0-正常在线, 1-离线或异常
 */
uint8_t DHT11_Init(void)
{
    // 开启 Cortex-M4 内核调试跟踪与 DWT 周期计数器 (CYCCNT)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    DHT11_IO_OUT();
    DHT11_DQ_OUT_HIGH();
    DHT11_Rst();
    return DHT11_Check();
}
