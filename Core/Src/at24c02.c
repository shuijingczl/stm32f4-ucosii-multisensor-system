#include "at24c02.h"

/**
 * @brief 向 AT24C02 指定内存地址写入一个字节
 * @param mem_addr 目标内存地址 (0~255)
 * @param data 要写入的单字节数据
 * @retval 0-成功, 1-失败
 */
uint8_t AT24C02_Write_Byte(uint8_t mem_addr, uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c1, AT24C02_DEV_ADDR, mem_addr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
    // 关键答辩考点：AT24C02 内部浮栅电荷擦写周期 (Write Cycle Time) 最大为 5ms！
    // 发送完数据后芯片会进入内部编程阶段并对 I2C 产生 NACK，必须延时 5ms 等待烧写完成！
    HAL_Delay(5);
    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief 从 AT24C02 指定内存地址读取一个字节
 * @param mem_addr 目标内存地址 (0~255)
 * @param data 用于存放读出数据的指针
 * @retval 0-成功, 1-失败
 */
uint8_t AT24C02_Read_Byte(uint8_t mem_addr, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c1, AT24C02_DEV_ADDR, mem_addr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
    return (status == HAL_OK) ? 0 : 1;
}
