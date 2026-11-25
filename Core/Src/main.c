/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "fsmc.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dht11.h"
#include "at24c02.h"
#include "lcd.h"
#include "flash.h"
#include "includes.h"   // [RTOS 新增] 引入 uC/OS-II 内核总头文件 (ucos_ii.h / os_cpu.h / os_cfg.h)
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Flash 历史记录扇区基地址 (W25Q128 扇区 16: 0x00010000 = 64KB)
#define FLASH_HISTORY_ADDR      0x00010000
#define FLASH_HISTORY_MAGIC     0x55AAAA55
#define FLASH_HISTORY_MAX_NUM   10

typedef struct {
    float    volt;       // 电压 (V)
    uint8_t  humi;       // 湿度 (%RH)
    uint8_t  temp;       // 温度 (℃)
    uint8_t  fan_duty;   // 风扇占空比 (%)
    uint8_t  alarm_flag; // 报警状态 (0:正常, 1:电压报警, 2:湿度报警, 3:双重报警)
} HistoryRecord_t;

typedef struct {
    uint32_t magic;      // 0x55AAAA55
    uint8_t  count;      // 0~10
    HistoryRecord_t records[FLASH_HISTORY_MAX_NUM];
} HistoryStorage_t;

typedef enum {
    UI_PAGE_REALTIME = 0, // 0: 实时监控仪表盘
    UI_PAGE_HISTORY  = 1  // 1: 历史 10 组记录表格
} UIPage_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// ==================== [RTOS 新增: 任务优先级与栈配置] ====================
// 优先级规则：数值越小优先级越高 (0 被 OS_TASK_TMR_PRIO 占用, 62/63 被统计/空闲任务占用)
#define TASK_SENSOR_PRIO      1u     // 任务1: 传感器采样 (ADC 50ms + DHT11 1s)
#define TASK_CTRL_PRIO        2u     // 任务2: 风扇/PWM/报警裁决 (邮箱触发)
#define TASK_I2C_PRIO         3u     // 任务3: AT24C02 I2C 写校验 (邮箱触发, 5ms 让出 CPU)
#define TASK_KEY_PRIO         4u     // 任务4: K1 按键状态机 (20ms) + 3s Flash 归档
#define TASK_UI_PRIO          5u     // 任务5: LCD 切屏与 200ms 局部刷新
#define TASK_UART_PRIO        6u     // 任务6: USART1 每 1s 遥测报文上报
#define TASK_START_PRIO       7u     // 启动引导任务 (初始化内核对象并创建子任务后自删)

// 堆栈深度 (单位: 32位 OS_STK, 即字数; Cortex-M4F 需为浮点寄存器保存与 printf 预留余量)
#define TASK_START_STK_SIZE   128u
#define TASK_SENSOR_STK_SIZE  256u
#define TASK_CTRL_STK_SIZE    256u
#define TASK_I2C_STK_SIZE     256u
#define TASK_KEY_STK_SIZE     256u
#define TASK_UI_STK_SIZE      512u
#define TASK_UART_STK_SIZE    512u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float    g_light_volt        = 0.0f; // 当前光敏采集电压 (V)
volatile uint8_t  g_alarm_en          = 0;    // 电压超标报警标志 (1Hz 蜂鸣器): 1-开启, 0-关闭
volatile uint8_t  g_fan_duty          = 0;    // 当前风扇 PWM 占空比百分比 (0~100)
volatile uint8_t  g_humidity          = 0;    // 从 DHT11 直接采集到的实时湿度 (%RH)
volatile uint8_t  g_temperature       = 0;    // 从 DHT11 直接采集到的实时温度 (℃)
volatile uint8_t  g_eeprom_humi       = 0;    // 从 AT24C02 I2C 读回校验的湿度 (%RH)
volatile uint8_t  g_humi_alarm_en     = 0;    // 湿度超标报警标志 (10Hz LED1): 1-开启, 0-关闭

volatile UIPage_t g_current_page      = UI_PAGE_REALTIME; // 当前 LCD 显示页面
volatile uint8_t  g_flash_ok          = 0;    // W25Q128 硬件握手就绪标志 (1-OK, 0-FAIL)
volatile uint8_t  g_page_need_draw    = 1;    // 页面重绘标志 (切屏时置 1)
volatile uint32_t g_clear_prompt_tick = 0;    // 清除提示显示计时
HistoryStorage_t  g_hist_storage      = {0};  // 历史数据内存缓存

// ==================== [RTOS 新增: 内核同步通信对象与任务堆栈] ====================
// Cortex-M4 AAPCS 与硬件 FPU 要求任务栈起始及运行时严格 8 字节对齐 (__align(8))
__align(8) OS_STK TASK_START_STK[TASK_START_STK_SIZE];
__align(8) OS_STK TASK_SENSOR_STK[TASK_SENSOR_STK_SIZE];
__align(8) OS_STK TASK_CTRL_STK[TASK_CTRL_STK_SIZE];
__align(8) OS_STK TASK_I2C_STK[TASK_I2C_STK_SIZE];
__align(8) OS_STK TASK_KEY_STK[TASK_KEY_STK_SIZE];
__align(8) OS_STK TASK_UI_STK[TASK_UI_STK_SIZE];
__align(8) OS_STK TASK_UART_STK[TASK_UART_STK_SIZE];

OS_EVENT *g_sem_data      = NULL; // 二值信号量: 保护共享传感器与控制变量 (初值 1: 0被占/1可用)
OS_EVENT *g_sem_storage   = NULL; // 二值信号量: 保护 g_hist_storage 缓存与 W25Q128 SPI 访问 (初值 1)
OS_EVENT *g_mbox_ctrl     = NULL; // 消息邮箱: Sample -> Control (实时采样帧通知)
OS_EVENT *g_mbox_i2c      = NULL; // 消息邮箱: Sample -> I2C_Comm (新湿度值待存入 EEPROM)

typedef struct {
    float   volt;
    uint8_t humi;
    uint8_t temp;
    uint8_t eeprom_humi;
    uint8_t fan_duty;
    uint8_t alarm_en;
    uint8_t humi_alarm_en;
} SensorSnapshot_t;

typedef struct {
    float   volt;
    uint8_t humi;
} CtrlMailMsg_t;

static CtrlMailMsg_t g_ctrl_mail_msg = {0.0f, 0};
static uint8_t       g_i2c_mail_humi = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Storage_Init(void);
void Storage_AddRecord(float volt, uint8_t humi, uint8_t temp, uint8_t fan_duty, uint8_t alarm);
void Storage_ClearHistory(void);
void AppTask_ADC_Process(void);
// void AppTask_Fan_Process(void); // [老函数保留] 已由 AppTask_Fan_Update 替代
void AppTask_Fan_Update(float volt);
uint8_t AppTask_Sensor_Process(void);
void AppTask_I2C_Process(uint8_t humi);
void AppTask_Key_Process(void);
void AppTask_Storage_Process(void);
void AppTask_LCD_Init(void);
void AppTask_LCD_DrawRealtimePage(void);
void AppTask_LCD_DrawHistoryPage(void);
void AppTask_LCD_Process(void);
void AppTask_UART_Process(void);

// [RTOS 新增任务入口与保护辅助函数声明]
void Task_Start(void *p_arg);
void Task_Sensor_Sample(void *p_arg);
void Task_Control_Alarm(void *p_arg);
void Task_I2C_Comm(void *p_arg);
void Task_Key_Storage(void *p_arg);
void Task_UI_Display(void *p_arg);
void Task_UART_Report(void *p_arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// ==================== [RTOS 新增: 线程安全数据快照与二值信号量辅助函数] ====================
// 使用二值信号量 (OSSem): 初值设为 1 (0 代表被占用, 1 代表空闲可用)
static void App_LockSem(OS_EVENT *sem)
{
    INT8U err;
    if ((OSRunning == OS_TRUE) && (sem != (OS_EVENT *)0))
    {
        OSSemPend(sem, 0, &err);
    }
}

static void App_UnlockSem(OS_EVENT *sem)
{
    if ((OSRunning == OS_TRUE) && (sem != (OS_EVENT *)0))
    {
        OSSemPost(sem);
    }
}

static void App_GetSensorSnapshot(SensorSnapshot_t *snap)
{
    App_LockSem(g_sem_data);
    snap->volt          = g_light_volt;
    snap->humi          = g_humidity;
    snap->temp          = g_temperature;
    snap->eeprom_humi   = g_eeprom_humi;
    snap->fan_duty      = g_fan_duty;
    snap->alarm_en      = g_alarm_en;
    snap->humi_alarm_en = g_humi_alarm_en;
    App_UnlockSem(g_sem_data);
}

/**
 * @brief 初始化 W25Q128 并恢复掉电历史记录 (OSStart() 前单线程执行，与裸机完全一致)
 */
void Storage_Init(void)
{
    uint16_t flashid = 0;

    /* SPI1 Mode3(CPOL=1) 首字节预热：CS 保持高电平，发空时钟使 SCK 稳定空闲高电平 */
    __Deselect_Flash();
    SPI1_ReadWriteByte(0xFF);

    /* 释放掉电并拉高片选，复位可能因热下载中断的 W25Q128 通信状态 */
    __Select_Flash();
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);
    __Deselect_Flash();
    HAL_Delay(2);

    /* 丢弃首次 ID 读取，再最多重试 5 次读取芯片 ID */
    W25QXX_ReadID();
    for (uint8_t retry = 0; retry < 5U; retry++)
    {
        flashid = W25QXX_ReadID();
        if ((flashid == W25Q128) || (flashid == 0xC817))
        {
            break;
        }
        HAL_Delay(10);
    }

    if ((flashid == W25Q128) || (flashid == 0xC817))
    {
        g_flash_ok = 1;
        W25QXX_Read((uint8_t *)&g_hist_storage, FLASH_HISTORY_ADDR, sizeof(HistoryStorage_t));
        if (g_hist_storage.magic != FLASH_HISTORY_MAGIC || g_hist_storage.count > FLASH_HISTORY_MAX_NUM)
        {
            g_hist_storage.magic = FLASH_HISTORY_MAGIC;
            g_hist_storage.count = 0;
            memset(g_hist_storage.records, 0, sizeof(g_hist_storage.records));
            W25QXX_Write((uint8_t *)&g_hist_storage, FLASH_HISTORY_ADDR, sizeof(HistoryStorage_t));
        }
        printf("[FLASH] W25Q128 OK (ID=0x%04X, Records=%u)\r\n", flashid, (unsigned int)g_hist_storage.count);
    }
    else
    {
        g_flash_ok = 0;
        printf("[FLASH] NOT FOUND (ID=0x%04X)\r\n", flashid);
    }
}

/**
 * @brief 新增一条历史遥测记录 (滚动 FIFO 环形队列)
 */
void Storage_AddRecord(float volt, uint8_t humi, uint8_t temp, uint8_t fan_duty, uint8_t alarm)
{
    if (!g_flash_ok) return;

    // [原裸机状态] 单线程顺序轮询，直接修改 g_hist_storage 并写 Flash
    // [RTOS 改动] 申请二值信号量保护，防止写盘/移位期间 Task_UI_Display 读出撕裂的历史表格
    App_LockSem(g_sem_storage);

    if (g_hist_storage.count < FLASH_HISTORY_MAX_NUM)
    {
        g_hist_storage.count++;
    }
    else
    {
        for (int i = 0; i < FLASH_HISTORY_MAX_NUM - 1; i++)
        {
            g_hist_storage.records[i] = g_hist_storage.records[i + 1];
        }
    }
    
    HistoryRecord_t *r = &g_hist_storage.records[g_hist_storage.count - 1];
    r->volt       = volt;
    r->humi       = humi;
    r->temp       = temp;
    r->fan_duty   = fan_duty;
    r->alarm_flag = alarm;
    
    g_hist_storage.magic = FLASH_HISTORY_MAGIC;
    W25QXX_Write((uint8_t *)&g_hist_storage, FLASH_HISTORY_ADDR, sizeof(HistoryStorage_t));

    App_UnlockSem(g_sem_storage);
}

/**
 * @brief 清空 Flash 历史记录 (响应 K1 长按)
 */
void Storage_ClearHistory(void)
{
    if (!g_flash_ok) return;

    // [RTOS 改动] 二值信号量保护 W25Q128 擦写及 g_hist_storage 缓存清零
    App_LockSem(g_sem_storage);

    g_hist_storage.magic = FLASH_HISTORY_MAGIC;
    g_hist_storage.count = 0;
    memset(g_hist_storage.records, 0, sizeof(g_hist_storage.records));
    W25QXX_Write((uint8_t *)&g_hist_storage, FLASH_HISTORY_ADDR, sizeof(HistoryStorage_t));

    App_UnlockSem(g_sem_storage);
}

/**
 * @brief  绘制 Page 0: 实时监控仪表盘底框与静态标签
 */
void AppTask_LCD_DrawRealtimePage(void)
{
    LCD_Clear(BLACK);
    
    // --- 顶部导航标题栏 (深度蓝背景 + 金黄色居中文字) ---
    LCD_Fill(0, 0, 319, 26, DARKBLUE);
    LCD_ShowString(16, 5, (const u8 *)"STAR-SPARK EMBEDDED MONITOR", YELLOW, DARKBLUE, 16, 0);
    LCD_DrawLine(0, 27, 319, 27, GRAYBLUE);
    
    // --- 卡片1: 光照电压采集模块 (PA3 ADC1) ---
    LCD_DrawRectangle(4, 31, 315, 75, GRAYBLUE);
    LCD_ShowString(10, 34, (const u8 *)"[1] LIGHT SENSOR (PA3)", CYAN, BLACK, 16, 0);
    LCD_ShowString(10, 54, (const u8 *)"Volt :       V", WHITE, BLACK, 16, 0);
    
    // --- 卡片2: 温湿度环境模块 (PC13 DHT11) ---
    LCD_DrawRectangle(4, 80, 315, 124, GRAYBLUE);
    LCD_ShowString(10, 83, (const u8 *)"[2] HUMIDITY SENSOR (DHT11)", CYAN, BLACK, 16, 0);
    LCD_ShowString(10, 103, (const u8 *)"Humi :    %RH", WHITE, BLACK, 16, 0);
    
    // --- 卡片3: 风扇调速与执行器 (PWM TIM10) ---
    LCD_DrawRectangle(4, 129, 315, 173, GRAYBLUE);
    LCD_ShowString(10, 132, (const u8 *)"[3] FAN CONTROLLER (TIM10)", CYAN, BLACK, 16, 0);
    LCD_ShowString(10, 152, (const u8 *)"State: ", WHITE, BLACK, 16, 0);
    
    // --- 卡片4: I2C & SPI 存储状态 ---
    LCD_DrawRectangle(4, 178, 315, 218, GRAYBLUE);
    LCD_ShowString(10, 181, (const u8 *)"[4] AT24C02 & W25Q128 STORAGE", CYAN, BLACK, 16, 0);
    if (g_flash_ok)
    {
        LCD_ShowString(10, 199, (const u8 *)"I2C Save:    %RH  FLASH: [W25Q128 OK]", LGRAY, BLACK, 12, 0);
    }
    else
    {
        LCD_ShowString(10, 199, (const u8 *)"I2C Save:    %RH  FLASH: [NOT FOUND ]", RED, BLACK, 12, 0);
    }
    
    // --- 底部状态指示栏 ---
    LCD_DrawLine(0, 222, 319, 222, GRAYBLUE);
    LCD_ShowString(6, 225, (const u8 *)"PAGE:[REALTIME] | K1:SWITCH TO HISTORY", GRAY, BLACK, 12, 0);
}

/**
 * @brief  绘制 Page 1: 历史 10 组遥测数据表格底框与表头
 */
void AppTask_LCD_DrawHistoryPage(void)
{
    LCD_Clear(BLACK);
    
    // --- 顶部导航标题栏 ---
    LCD_Fill(0, 0, 319, 24, DARKBLUE);
    LCD_ShowString(16, 4, (const u8 *)"W25Q128 HISTORICAL RECORDS (10-PTS)", YELLOW, DARKBLUE, 16, 0);
    LCD_DrawLine(0, 25, 319, 25, GRAYBLUE);
    
    // --- 表头区域 (深沉蓝灰背景 + 纯白文字) ---
    LCD_Fill(4, 28, 315, 44, 0x18E3);
    LCD_ShowString(8, 30, (const u8 *)"NO    VOLT     HUMI    TEMP     FAN      STATUS", WHITE, 0x18E3, 12, 0);
    LCD_DrawLine(4, 45, 315, 45, GRAYBLUE);
    
    // --- 底部按键指引栏 ---
    LCD_DrawLine(0, 222, 319, 222, GRAYBLUE);
    LCD_ShowString(6, 225, (const u8 *)"[K1] Short: Monitor | Long (1.5s): Clear", LGRAY, BLACK, 12, 0);
}

/**
 * @brief  初始化 3.2 寸 TFT-LCD 液晶屏
 */
void AppTask_LCD_Init(void)
{
    LCD_Init();                    // 1. 初始化 ILI9341 控制器
    LCD_Clear(BLACK);              // 2. 先清屏为纯黑，避免显存随机杂点
    g_page_need_draw = 1;          // 3. 标记在首次任务调度时绘制首屏 UI
}

/**
 * @brief  TFT-LCD 双页面平滑无闪烁动态刷新任务
 */
/* ==================== [原裸机代码注释保留 - AppTask_LCD_Process] ====================
 * [改动原因] 1) 原版直接读取全局变量 g_light_volt / g_hist_storage 渲染屏幕；多任务抢占下
 *               可能导致刚画到一半时数据或历史队列被高优先级任务修改，出现数据撕裂。
 *            2) RTOS 版先在互斥锁内微秒级拷贝出本地快照 (snap / hist_snap) 后立刻解锁再慢速刷屏，
 *               既保证一致，又绝不因 FSMC 刷屏阻塞传感器与 Flash 任务；切屏标志亦加临界区保护。
 */
#if 0
void AppTask_LCD_Process(void)
{
    static uint32_t last_lcd_tick = 0;
    char str_buf[64];
    
    // 1. 如果切屏标志置位，先执行一次静态底框全屏重绘
    if (g_page_need_draw)
    {
        g_page_need_draw = 0;
        if (g_current_page == UI_PAGE_REALTIME)
        {
            AppTask_LCD_DrawRealtimePage();
        }
        else
        {
            AppTask_LCD_DrawHistoryPage();
        }
    }
    
    // 2. 200ms 分频增量非阻塞刷新 (零屏闪)
    if (HAL_GetTick() - last_lcd_tick < 200)
    {
        return;
    }
    last_lcd_tick = HAL_GetTick();
    
    // =========================================================================
    //                            PAGE 0: 实时监控仪表盘
    // =========================================================================
    if (g_current_page == UI_PAGE_REALTIME)
    {
        // ----- 卡片1: 光敏电压动态刷新 -----
        sprintf(str_buf, "%4.2f", g_light_volt);
        LCD_ShowString(64, 54, (const u8 *)str_buf, YELLOW, BLACK, 16, 0);
        
        if (g_light_volt > 1.5f)
        {
            LCD_ShowString(160, 54, (const u8 *)"[ALARM: BEEP 1Hz]", RED, BLACK, 16, 0);
        }
        else
        {
            LCD_ShowString(160, 54, (const u8 *)"[STATUS: NORMAL ]", GREEN, BLACK, 16, 0);
        }
        
        // ----- 卡片2: 湿度与温度动态刷新 -----
        sprintf(str_buf, "%2d", g_humidity);
        LCD_ShowString(64, 103, (const u8 *)str_buf, YELLOW, BLACK, 16, 0);
        
        sprintf(str_buf, "(%2dC)", g_temperature);
        LCD_ShowString(110, 103, (const u8 *)str_buf, LIGHTBLUE, BLACK, 16, 0);
        
        if (g_humidity > 50)
        {
            LCD_ShowString(160, 103, (const u8 *)"[ALARM: LED 10Hz]", RED, BLACK, 16, 0);
        }
        else
        {
            LCD_ShowString(160, 103, (const u8 *)"[STATUS: NORMAL ]", GREEN, BLACK, 16, 0);
        }
        
        // ----- 卡片3: 风扇启停与占空比动态刷新 -----
        if (g_light_volt > 1.5f)
        {
            sprintf(str_buf, "[RUN] Duty:%3d%%", g_fan_duty);
            LCD_ShowString(64, 152, (const u8 *)str_buf, CYAN, BLACK, 16, 0);
        }
        else
        {
            sprintf(str_buf, "[OFF] Duty:  0%%");
            LCD_ShowString(64, 152, (const u8 *)str_buf, GRAY, BLACK, 16, 0);
        }
        
        // ----- 卡片4: AT24C02 存储回读值动态刷新 -----
        sprintf(str_buf, "%2d", g_eeprom_humi);
        LCD_ShowString(76, 199, (const u8 *)str_buf, YELLOW, BLACK, 12, 0);
    }
    // =========================================================================
    //                        PAGE 1: 历史 10 组记录表格
    // =========================================================================
    else if (g_current_page == UI_PAGE_HISTORY)
    {
        uint16_t y_pos = 48;
        
        for (int i = 0; i < FLASH_HISTORY_MAX_NUM; i++)
        {
            y_pos = 48 + i * 16;
            if (i < g_hist_storage.count)
            {
                HistoryRecord_t *r = &g_hist_storage.records[i];
                const char *alm_str = "NORMAL  ";
                uint16_t alm_color = GREEN;
                
                if (r->alarm_flag == 3)      { alm_str = "DUAL_ALM"; alm_color = RED; }
                else if (r->alarm_flag == 1) { alm_str = "V_ALARM "; alm_color = YELLOW; }
                else if (r->alarm_flag == 2) { alm_str = "H_ALARM "; alm_color = MAGENTA; }
                
                sprintf(str_buf, "#%02d   %4.2fV    %3d%%    %3dC     %3d%%     ",
                        i + 1, r->volt, r->humi, r->temp, r->fan_duty);
                LCD_ShowString(8, y_pos, (const u8 *)str_buf, WHITE, BLACK, 12, 0);
                LCD_ShowString(254, y_pos, (const u8 *)alm_str, alm_color, BLACK, 12, 0);
            }
            else
            {
                sprintf(str_buf, "#%02d   -.--V     --%%     --C     ---%%     --------", i + 1);
                LCD_ShowString(8, y_pos, (const u8 *)str_buf, GRAY, BLACK, 12, 0);
            }
        }
        
        // 响应清空提示显示
        if (g_clear_prompt_tick > 0)
        {
            if (HAL_GetTick() - g_clear_prompt_tick < 1500)
            {
                LCD_ShowString(36, 208, (const u8 *)">>> FLASH HISTORY CLEARED! <<<", RED, BLACK, 12, 0);
            }
            else
            {
                LCD_ShowString(36, 208, (const u8 *)"                              ", BLACK, BLACK, 12, 0);
                g_clear_prompt_tick = 0;
            }
        }
    }
}
#endif

void AppTask_LCD_Process(void)
{
    OS_CPU_SR cpu_sr = 0u;
    static uint32_t last_lcd_tick = 0;
    char str_buf[64];
    UIPage_t local_page;
    uint8_t  need_draw;
    uint32_t prompt_tick;

    // 1. 临界区内原子读取并清除切屏标志，防止被高优先级按键任务打断造成底框与页面错配
    OS_ENTER_CRITICAL();
    need_draw = g_page_need_draw;
    g_page_need_draw = 0;
    local_page = g_current_page;
    prompt_tick = g_clear_prompt_tick;
    OS_EXIT_CRITICAL();

    if (need_draw)
    {
        if (local_page == UI_PAGE_REALTIME)
        {
            AppTask_LCD_DrawRealtimePage();
        }
        else
        {
            AppTask_LCD_DrawHistoryPage();
        }
    }

    // 2. 数据刷新保持 200ms 周期无闪烁 (刚切屏重绘底框时立即补全当帧数值)
    if (!need_draw && (HAL_GetTick() - last_lcd_tick < 200U))
    {
        return;
    }
    last_lcd_tick = HAL_GetTick();

    if (local_page == UI_PAGE_REALTIME)
    {
        SensorSnapshot_t snap;
        App_GetSensorSnapshot(&snap); // 互斥锁保护拷贝，锁外慢速渲染 LCD

        sprintf(str_buf, "%4.2f", snap.volt);
        LCD_ShowString(64, 54, (const u8 *)str_buf, YELLOW, BLACK, 16, 0);

        if (snap.volt > 1.5f)
        {
            LCD_ShowString(160, 54, (const u8 *)"[ALARM: BEEP 1Hz]", RED, BLACK, 16, 0);
        }
        else
        {
            LCD_ShowString(160, 54, (const u8 *)"[STATUS: NORMAL ]", GREEN, BLACK, 16, 0);
        }

        sprintf(str_buf, "%2d", snap.humi);
        LCD_ShowString(64, 103, (const u8 *)str_buf, YELLOW, BLACK, 16, 0);

        sprintf(str_buf, "(%2dC)", snap.temp);
        LCD_ShowString(110, 103, (const u8 *)str_buf, LIGHTBLUE, BLACK, 16, 0);

        if (snap.humi > 50)
        {
            LCD_ShowString(160, 103, (const u8 *)"[ALARM: LED 10Hz]", RED, BLACK, 16, 0);
        }
        else
        {
            LCD_ShowString(160, 103, (const u8 *)"[STATUS: NORMAL ]", GREEN, BLACK, 16, 0);
        }

        if (snap.volt > 1.5f)
        {
            sprintf(str_buf, "[RUN] Duty:%3d%%", snap.fan_duty);
            LCD_ShowString(64, 152, (const u8 *)str_buf, CYAN, BLACK, 16, 0);
        }
        else
        {
            sprintf(str_buf, "[OFF] Duty:  0%%");
            LCD_ShowString(64, 152, (const u8 *)str_buf, GRAY, BLACK, 16, 0);
        }

        sprintf(str_buf, "%2d", snap.eeprom_humi);
        LCD_ShowString(76, 199, (const u8 *)str_buf, YELLOW, BLACK, 12, 0);
    }
    else if (local_page == UI_PAGE_HISTORY)
    {
        HistoryStorage_t hist_snap;
        uint16_t y_pos = 48;

        // 申请二值信号量，拷贝历史表快照后立刻释放
        App_LockSem(g_sem_storage);
        memcpy(&hist_snap, &g_hist_storage, sizeof(HistoryStorage_t));
        App_UnlockSem(g_sem_storage);

        for (int i = 0; i < FLASH_HISTORY_MAX_NUM; i++)
        {
            y_pos = 48 + i * 16;
            if (i < hist_snap.count)
            {
                HistoryRecord_t *r = &hist_snap.records[i];
                const char *alm_str = "NORMAL  ";
                uint16_t alm_color = GREEN;

                if (r->alarm_flag == 3)      { alm_str = "DUAL_ALM"; alm_color = RED; }
                else if (r->alarm_flag == 1) { alm_str = "V_ALARM "; alm_color = YELLOW; }
                else if (r->alarm_flag == 2) { alm_str = "H_ALARM "; alm_color = MAGENTA; }

                sprintf(str_buf, "#%02d   %4.2fV    %3d%%    %3dC     %3d%%     ",
                        i + 1, r->volt, r->humi, r->temp, r->fan_duty);
                LCD_ShowString(8, y_pos, (const u8 *)str_buf, WHITE, BLACK, 12, 0);
                LCD_ShowString(254, y_pos, (const u8 *)alm_str, alm_color, BLACK, 12, 0);
            }
            else
            {
                sprintf(str_buf, "#%02d   -.--V     --%%     --C     ---%%     --------", i + 1);
                LCD_ShowString(8, y_pos, (const u8 *)str_buf, GRAY, BLACK, 12, 0);
            }
        }

        if (prompt_tick > 0U)
        {
            if (HAL_GetTick() - prompt_tick < 1500U)
            {
                LCD_ShowString(36, 208, (const u8 *)">>> FLASH HISTORY CLEARED! <<<", RED, BLACK, 12, 0);
            }
            else
            {
                LCD_ShowString(36, 208, (const u8 *)"                              ", BLACK, BLACK, 12, 0);
                OS_ENTER_CRITICAL();
                g_clear_prompt_tick = 0U;
                OS_EXIT_CRITICAL();
            }
        }
    }
}

/**
 * @brief  K1 按键四态非阻塞消抖扫描与人机交互调度任务 (短按切屏 / 长按清空)
 */
void AppTask_Key_Process(void)
{
    OS_CPU_SR cpu_sr = 0u;
    typedef enum {
        KEY_STATE_IDLE = 0,         // 空闲态：按键处于稳定高电平
        KEY_STATE_PRESS_DEBOUNCE,   // 按下消抖态：刚检测到低电平，等待 20ms 稳定
        KEY_STATE_PRESSED,          // 确认按下态：已确认按下，计时判断长按
        KEY_STATE_RELEASE_DEBOUNCE  // 释放消抖态：检测到高电平，防止松手回弹误触发
    } KeyState_t;

    static KeyState_t state = KEY_STATE_IDLE;
    static uint32_t debounce_tick = 0;    // 消抖阶段起点时间戳
    static uint32_t press_start_tick = 0; // 按下阶段起点时间戳
    static uint8_t  long_triggered = 0;   // 长按锁存标志，防止松手后误判短按

    uint32_t now = HAL_GetTick();
    uint8_t is_down = K1_DOWN();          // PE0 低电平按下返回 1，高电平释放返回 0

    switch (state)
    {
        case KEY_STATE_IDLE:
            if (is_down)
            {
                state = KEY_STATE_PRESS_DEBOUNCE;
                debounce_tick = now;
                press_start_tick = now;
                long_triggered = 0;
            }
            break;

        case KEY_STATE_PRESS_DEBOUNCE:
            if (!is_down)
            {
                // 20ms 内回高电平 -> 视为毛刺或弹跳抖动，退回空闲态
                state = KEY_STATE_IDLE;
            }
            else if (now - debounce_tick >= 20)
            {
                // 低电平持续稳定 >= 20ms -> 正式确认按下
                state = KEY_STATE_PRESSED;
            }
            break;

        case KEY_STATE_PRESSED:
            if (!is_down)
            {
                // 检测到释放高电平 -> 先进入释放消抖，不立即结算短按
                state = KEY_STATE_RELEASE_DEBOUNCE;
                debounce_tick = now;
            }
            else if (!long_triggered && (now - press_start_tick >= 1500))
            {
                // 持续按下超过 1500ms 触发长按：清空历史记录
                long_triggered = 1;
                Storage_ClearHistory();
                // [原裸机代码] 直接修改全局变量 (裸机单线程无竞态)
                // g_clear_prompt_tick = HAL_GetTick();
                // if (g_current_page == UI_PAGE_HISTORY) { g_page_need_draw = 1; }
                // [RTOS 改动] 临界区保护，防止与低优先级 UI 任务并发读写切屏标志
                OS_ENTER_CRITICAL();
                g_clear_prompt_tick = HAL_GetTick();
                if (g_current_page == UI_PAGE_HISTORY)
                {
                    g_page_need_draw = 1;
                }
                OS_EXIT_CRITICAL();
            }
            break;

        case KEY_STATE_RELEASE_DEBOUNCE:
            if (is_down)
            {
                // 20ms 内又回到低电平 -> 释放弹跳或短暂抖动，恢复按下态继续保持
                state = KEY_STATE_PRESSED;
            }
            else if (now - debounce_tick >= 20)
            {
                // 高电平稳定 >= 20ms -> 确认彻底松手
                state = KEY_STATE_IDLE;
                if (!long_triggered)
                {
                    // 未触发过长按，且已通过按下+释放双向消抖 -> 判定为短按并切屏
                    // [RTOS 改动] 临界区原子更新页面编号与重绘通知标志
                    OS_ENTER_CRITICAL();
                    if (g_current_page == UI_PAGE_REALTIME)
                    {
                        g_current_page = UI_PAGE_HISTORY;
                    }
                    else
                    {
                        g_current_page = UI_PAGE_REALTIME;
                    }
                    g_page_need_draw = 1;
                    OS_EXIT_CRITICAL();
                }
            }
            break;

        default:
            state = KEY_STATE_IDLE;
            break;
    }
}

/**
 * @brief 模块五: 历史数据掉电持久化归档任务函数 (由 Task_Key_Storage 每 3 秒周期调用)
 */
void AppTask_Storage_Process(void)
{
    if (g_flash_ok)
    {
        SensorSnapshot_t snap;
        App_GetSensorSnapshot(&snap);

        uint8_t alarm_state = 0;
        if (snap.alarm_en && snap.humi_alarm_en) alarm_state = 3; // 双重报警
        else if (snap.alarm_en)                  alarm_state = 1; // 仅电压超标
        else if (snap.humi_alarm_en)             alarm_state = 2; // 仅湿度超标
        
        Storage_AddRecord(snap.volt, snap.humi, snap.temp, snap.fan_duty, alarm_state);
    }
}

/**
 * @brief 定时器周期溢出中断回调函数 (TIM6 每 500ms 触发一次, TIM7 每 50ms 触发一次)
 * @param htim 定时器句柄
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    // TIM6 周期溢出中断 (每 500ms 一次，调度蜂鸣器 1Hz 慢速间歇报警)
    if (htim->Instance == TIM6)
    {
        if (g_alarm_en)
        {
            HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin);
        }
        else
        {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        }
    }
    // TIM7 周期溢出中断 (每 50ms 一次，调度 LED1 10Hz 极速闪烁报警)
    else if (htim->Instance == TIM7)
    {
        if (g_humi_alarm_en)
        {
            HAL_GPIO_TogglePin(LED1_ALARM_GPIO_Port, LED1_ALARM_Pin);
        }
        else
        {
            HAL_GPIO_WritePin(LED1_ALARM_GPIO_Port, LED1_ALARM_Pin, GPIO_PIN_SET);
        }
    }
}

/**
 * @brief  采集 PA3 光敏电阻电压，并线性换算为实际电压值 (0.00V ~ 3.30V)
 * @retval 测量电压值 (float)
 */
float Get_Light_Voltage(void)
{
    uint32_t adc_val = 0;
    
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
    {
        adc_val = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    
    return (float)adc_val * 3.3f / 4095.0f;
}

/**
 * @brief 模块一: ADC 采集光敏电压与报警使能判定
 */
void AppTask_ADC_Process(void)
{
    float volt = Get_Light_Voltage();
    App_LockSem(g_sem_data);
    g_light_volt = volt;
    if (g_light_volt > 1.5f)
    {
        g_alarm_en = 1;
    }
    else
    {
        g_alarm_en = 0;
    }
    App_UnlockSem(g_sem_data);
}

/**
 * @brief 模块二: 风扇状态指示与 TIM10 PWM 无级偏差调速 (支持入参，免锁直接更新)
 * @param volt 当前光敏采集电压 (V)
 */
void AppTask_Fan_Update(float volt)
{
    if (volt > 1.5f)
    {
        HAL_GPIO_WritePin(LED2_FAN_GPIO_Port, LED2_FAN_Pin, GPIO_PIN_RESET);
        
        float delta_v = volt - 1.5f;
        float duty_calc = delta_v * (100.0f / 1.8f);
        if (duty_calc > 100.0f) duty_calc = 100.0f;
        if (duty_calc < 0.0f)   duty_calc = 0.0f;
        
        App_LockSem(g_sem_data);
        g_fan_duty = (uint8_t)duty_calc;
        App_UnlockSem(g_sem_data);

        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, (uint8_t)duty_calc);
    }
    else
    {
        HAL_GPIO_WritePin(LED2_FAN_GPIO_Port, LED2_FAN_Pin, GPIO_PIN_SET);
        
        App_LockSem(g_sem_data);
        g_fan_duty = 0;
        App_UnlockSem(g_sem_data);

        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
    }
}

/* ==================== [原裸机代码保留 - AppTask_Fan_Process] ====================
 * [改动原因] 原裸机风扇函数为无参设计，内部通过抢锁读取全局变量 g_light_volt；
 *            接入 RTOS 邮箱解包重构后，Task_Control_Alarm 已直接解出 msg->volt 并调用
 *            AppTask_Fan_Update(msg->volt) 免锁调速，本老函数已无活跃调用，
 *            按工程规范使用 #if 0 原样保留供答辩与裸机对照。
 */
#if 0
void AppTask_Fan_Process(void)
{
    App_LockSem(g_sem_data);
    float volt = g_light_volt;
    App_UnlockSem(g_sem_data);

    AppTask_Fan_Update(volt);
}
#endif

/**
 * @brief 模块三: DHT11 湿度采集业务函数 (采集温湿度并更新全局状态)
 * @retval 1: 本次采样成功; 0: 采样失败
 */
uint8_t AppTask_Sensor_Process(void)
{
    uint8_t temp = 0;
    uint8_t humi = 0;
    uint8_t ret = 0;
    
    if (DHT11_Read_Data(&temp, &humi) == 0)
    {
        App_LockSem(g_sem_data);
        g_temperature = temp;
        g_humidity = humi;
        App_UnlockSem(g_sem_data);
        ret = 1;
    }
    
    /* ==================== [原裸机代码保留 - 湿度超标报警裁决] ====================
     * [改动原因] 裸机单线程在此处就地判定湿度报警与控制 LED1；
     *            RTOS 多任务架构下，该职责已正式移交至 Task_Control_Alarm 任务，
     *            由其解包消息邮箱中的实体 humi 数据统一执行裁决，实现采样与报警控制解耦。
     */
#if 0
    App_LockSem(g_sem_data);
    uint8_t cur_humi = g_humidity;
    if (cur_humi > 50)
    {
        g_humi_alarm_en = 1;
    }
    else
    {
        g_humi_alarm_en = 0;
        HAL_GPIO_WritePin(LED1_ALARM_GPIO_Port, LED1_ALARM_Pin, GPIO_PIN_SET);
    }
    App_UnlockSem(g_sem_data);
#endif

    return ret;
}

/**
 * @brief 模块三从属: AT24C02 I2C 存取与回读校验 (5ms 睡眠让权替代死等 HAL_Delay)
 */
void AppTask_I2C_Process(uint8_t humi)
{
    uint8_t readback = 0;
    HAL_I2C_Mem_Write(&hi2c1, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, &humi, 1, 100);
    // [RTOS 让权] 睡眠 5ms 让出 CPU 给其他任务，替代原阻塞死等 HAL_Delay(5)
    OSTimeDlyHMSM(0, 0, 0, 5);

    if (HAL_I2C_Mem_Read(&hi2c1, 0xA0, 0x10, I2C_MEMADD_SIZE_8BIT, &readback, 1, 100) != HAL_OK)
    {
        readback = humi;
    }

    App_LockSem(g_sem_data);
    g_eeprom_humi = readback;
    App_UnlockSem(g_sem_data);
}

/**
 * @brief 重定向 C 标准库 printf 输出到 USART1 (依赖 Keil MicroLIB)
 */
int fputc(int ch, FILE *f)
{
    uint8_t c = (uint8_t)ch;
    (void)f;
    HAL_UART_Transmit(&huart1, &c, 1, 100);
    return ch;
}

/**
 * @brief 模块六: 串口状态遥测报文上报业务处理函数 (由 Task_UART_Report 每 1 秒周期调用)
 */
void AppTask_UART_Process(void)
{
    uint32_t total_sec = HAL_GetTick() / 1000U;
    uint32_t min = total_sec / 60U;
    uint32_t sec = total_sec % 60U;

    SensorSnapshot_t snap;
    App_GetSensorSnapshot(&snap);

    const char *fan_str = (snap.alarm_en != 0U) ? "RUN " : "STOP";
    const char *alarm_str = "[NONE]";
    if (snap.alarm_en != 0U && snap.humi_alarm_en != 0U)
    {
        alarm_str = "[V_ALARM | H_ALARM]";
    }
    else if (snap.alarm_en != 0U)
    {
        alarm_str = "[V_ALARM (BEEP 1Hz)]";
    }
    else if (snap.humi_alarm_en != 0U)
    {
        alarm_str = "[H_ALARM (LED 10Hz)]";
    }

    char tx_buf[160];
    int len = snprintf(
        tx_buf,
        sizeof(tx_buf),
        "[Time: %02lu:%02lu] Volt: %4.2fV, Humi: %2u%%, Temp: %2uC, EEPROM: %2u%%, Fan: %s (PWM: %3u%%), Alarm: %s\r\n",
        (unsigned long)min,
        (unsigned long)sec,
        snap.volt,
        (unsigned int)snap.humi,
        (unsigned int)snap.temp,
        (unsigned int)snap.eeprom_humi,
        fan_str,
        (unsigned int)snap.fan_duty,
        alarm_str
    );

    if (len > 0)
    {
        uint16_t tx_len = (len < (int)sizeof(tx_buf)) ? (uint16_t)len : (uint16_t)(sizeof(tx_buf) - 1U);
        HAL_UART_Transmit(&huart1, (uint8_t *)tx_buf, tx_len, 100);
    }
}

// =========================================================================
//                  [RTOS 新增: uC/OS-II 多任务顶层实现]
// =========================================================================

/**
 * @brief 启动任务: 创建互斥信号量、消息邮箱及 6 个业务任务后自我删除
 */
void Task_Start(void *p_arg)
{
    OS_CPU_SR cpu_sr = 0u;
    (void)p_arg;

    // 1. 创建内核同步与通信对象 (2 个二值信号量 + 2 个消息邮箱，共 4 个 OS_EVENT)
    // 信号量初值设为 1: 0 代表资源被占用，1 代表资源空闲可用
    g_sem_data      = OSSemCreate(1u);
    g_sem_storage   = OSSemCreate(1u);
    g_mbox_ctrl     = OSMboxCreate((void *)0);
    g_mbox_i2c      = OSMboxCreate((void *)0);

    // 2. 初始化 CPU 利用率统计任务 (内部短暂延时约 100ms 标定空闲基准)
    OSStatInit();

    // 3. 临界区内批量创建 6 个业务任务，保证全部就绪后再统一参与调度
    OS_ENTER_CRITICAL();
    OSTaskCreate(Task_Sensor_Sample, (void *)0, &TASK_SENSOR_STK[TASK_SENSOR_STK_SIZE - 1u], TASK_SENSOR_PRIO);
    OSTaskCreate(Task_Control_Alarm, (void *)0, &TASK_CTRL_STK[TASK_CTRL_STK_SIZE - 1u],     TASK_CTRL_PRIO);
    OSTaskCreate(Task_I2C_Comm,      (void *)0, &TASK_I2C_STK[TASK_I2C_STK_SIZE - 1u],       TASK_I2C_PRIO);
    OSTaskCreate(Task_Key_Storage,   (void *)0, &TASK_KEY_STK[TASK_KEY_STK_SIZE - 1u],       TASK_KEY_PRIO);
    OSTaskCreate(Task_UI_Display,    (void *)0, &TASK_UI_STK[TASK_UI_STK_SIZE - 1u],         TASK_UI_PRIO);
    OSTaskCreate(Task_UART_Report,   (void *)0, &TASK_UART_STK[TASK_UART_STK_SIZE - 1u],     TASK_UART_PRIO);
    OS_EXIT_CRITICAL();

    // 4. 启动引导任务使命完成，销毁自己释放控制块与任务名额
    OSTaskDel(OS_PRIO_SELF);
}

/**
 * @brief 任务1 (优先级 1): 传感器周期采样 (调用模块一 AppTask_ADC_Process 与模块三 AppTask_Sensor_Process)
 */
void Task_Sensor_Sample(void *p_arg)
{
    uint8_t dht_div = 19u; // 设为 19 使上电首个周期即刻读取一次 DHT11，后续每 20*50ms = 1s 采集一次
    (void)p_arg;

    while (1)
    {
        uint8_t dht_ran = 0;

        // 1. 调用模块一老函数: 采样光敏电压与报警裁决
        AppTask_ADC_Process();

        // 2. 每 20 拍 (1s) 调用模块三老函数: DHT11 采集与湿度报警裁决
        if (++dht_div >= 20u)
        {
            dht_div = 0;
            dht_ran = 1;
            if (AppTask_Sensor_Process() != 0)
            {
                g_i2c_mail_humi = g_humidity;
                OSMboxPost(g_mbox_i2c, (void *)&g_i2c_mail_humi);
            }
        }

        // 3. 通过消息邮箱驱动 Task_Control_Alarm 执行风扇调速
        g_ctrl_mail_msg.volt = g_light_volt;
        g_ctrl_mail_msg.humi = g_humidity;
        OSMboxPost(g_mbox_ctrl, (void *)&g_ctrl_mail_msg);

        // 让出 CPU (若刚经历 DHT11 20ms 复位则补齐 30ms，保持整周期约 50ms)
        OSTimeDlyHMSM(0, 0, 0, dht_ran ? 30u : 50u);
    }
}

/**
 * @brief 任务2 (优先级 2): 解包消息邮箱实体数据帧，统一执行风扇 PWM 调速与湿度报警裁决
 */
void Task_Control_Alarm(void *p_arg)
{
    INT8U err;
    (void)p_arg;

    while (1)
    {
        // 1. 真正解包消息邮箱中的指针，获取采样任务投递的环境数据包 (包含 volt 与 humi)
        CtrlMailMsg_t *p_msg = (CtrlMailMsg_t *)OSMboxPend(g_mbox_ctrl, 0, &err);
        if (p_msg != (CtrlMailMsg_t *)0)
        {
            // 2. 消费 volt: 直接调用 AppTask_Fan_Update 计算风扇 TIM10 PWM 占空比与 LED2 状态
            AppTask_Fan_Update(p_msg->volt);

            // 3. 消费 humi: 正式接管湿度报警裁决 (超过 50%RH 激活 TIM7 10Hz LED1 闪烁报警)
            App_LockSem(g_sem_data);
            if (p_msg->humi > 50)
            {
                g_humi_alarm_en = 1;
            }
            else
            {
                g_humi_alarm_en = 0;
                HAL_GPIO_WritePin(LED1_ALARM_GPIO_Port, LED1_ALARM_Pin, GPIO_PIN_SET); // 恢复高电平熄灭
            }
            App_UnlockSem(g_sem_data);
        }
    }
}

/**
 * @brief 任务3 (优先级 3): 邮箱唤醒后调用模块三从属函数 AppTask_I2C_Process 写入并校验 AT24C02
 */
void Task_I2C_Comm(void *p_arg)
{
    INT8U err;
    (void)p_arg;

    while (1)
    {
        uint8_t *p_humi = (uint8_t *)OSMboxPend(g_mbox_i2c, 0, &err);
        if (p_humi != (uint8_t *)0)
        {
            // 调用模块三拆出来的 I2C 存储写入与回读校验
            AppTask_I2C_Process(*p_humi);
        }
    }
}

/**
 * @brief 任务4 (优先级 4): 20ms 节拍调用模块五老函数 AppTask_Key_Process 与 AppTask_Storage_Process
 */
void Task_Key_Storage(void *p_arg)
{
    uint16_t save_div = 0;
    (void)p_arg;

    while (1)
    {
        // 1. 调用模块五老函数: K1 四态消抖按键扫描 (长按清空 Flash)
        AppTask_Key_Process();

        // 2. 每 150 个 20ms 节拍 (= 3000ms) 调用模块五老函数: Flash 历史数据持久化归档
        if (++save_div >= 150u)
        {
            save_div = 0;
            AppTask_Storage_Process();
        }

        OSTimeDlyHMSM(0, 0, 0, 20);
    }
}

/**
 * @brief 任务5 (优先级 5): TFT-LCD 双页面切屏响应 (50ms) 与动态数据刷新 (200ms)
 */
void Task_UI_Display(void *p_arg)
{
    (void)p_arg;

    while (1)
    {
        // 调用模块四老函数: TFT-LCD 双页面切屏与动态刷新
        AppTask_LCD_Process();
        OSTimeDlyHMSM(0, 0, 0, 50);
    }
}

/**
 * @brief 任务6 (优先级 6): 1 秒整周期调用模块六老函数 AppTask_UART_Process 发送遥测报文
 */
void Task_UART_Report(void *p_arg)
{
    (void)p_arg;

    while (1)
    {
        OSTimeDlyHMSM(0, 0, 1, 0); // 整秒挂起让出 CPU
        // 调用模块六老函数: 串口遥测报文发送
        AppTask_UART_Process();
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM10_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  MX_FSMC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  *((volatile uint16_t *)0x68000000) = 0xFFFF;                            // 0. FSMC NE3 消隐共阳数码管
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);             // 1. 默认蜂鸣器高电平静音
  HAL_GPIO_WritePin(LED2_FAN_GPIO_Port, LED2_FAN_Pin, GPIO_PIN_SET);     // 2. 默认风扇指示灯 LED2 熄灭
  HAL_GPIO_WritePin(LED1_ALARM_GPIO_Port, LED1_ALARM_Pin, GPIO_PIN_SET); // 3. 默认湿度报警灯 LED1 熄灭
  
  AppTask_LCD_Init();                                                     // 4. 先初始化 LCD 屏控制器 (NE4)
  DHT11_Init();                                                           // 5. 初始化检测 DHT11 湿度传感器
  
  HAL_TIM_Base_Start_IT(&htim6);                                          // 6. 启动 TIM6 中断 (500ms 蜂鸣器 1Hz 时基)
  HAL_TIM_Base_Start_IT(&htim7);                                          // 7. 启动 TIM7 中断 (50ms LED1 10Hz 时基)
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);                              // 8. 启动 TIM10 通道 1 硬件 PWM 输出
  
  Storage_Init();                                                         // 9. 初始化 W25Q128 Flash 并加载历史记录
  printf("\r\n[SYS] ChenZhiLing uC/OS-II System Starting (USART1 115200 8-N-1)\r\n");

  // ==================== [RTOS 改动: 初始化内核并交由操作系统接管调度] ====================
  OSInit();
  OSTaskCreate(Task_Start, (void *)0, &TASK_START_STK[TASK_START_STK_SIZE - 1u], TASK_START_PRIO);
  OSStart(); // 启动多任务调度，正常情况下永不返回
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /* ==================== [原裸机代码注释保留 - main() 前后台超级循环] ====================
   * [改动原因] 裸机使用 while(1) + HAL_Delay(20) 顺序轮询 7 个函数；
   *            接入 uC/OS-II 后由 OSStart() 接管多任务抢占调度，不再运行裸机主循环。
   */
#if 0
  while (1)
  {
      AppTask_ADC_Process();     // 任务1: 采集光敏电压与蜂鸣器判定
      AppTask_Fan_Process();     // 任务2: 风扇状态指示与 PWM 偏离比例调速
      AppTask_Sensor_Process();  // 任务3: 湿度采集 + I2C 存取 AT24C02 + 10Hz LED1 报警判定
      AppTask_Key_Process();     // 任务4: K1 按键非阻塞长短按检测与切屏/清空调度
      AppTask_Storage_Process(); // 任务5: 周期 3s 向 W25Q128 Flash 归档历史数据
      AppTask_LCD_Process();     // 任务6: TFT-LCD 双页面平滑无闪烁动态刷新
      AppTask_UART_Process();    // 任务7: USART1 每 1s 定时上报综合监测遥测报文
      HAL_Delay(20);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#else
  while (1)
  {
      // 仅作防跑飞兜底 (OSStart() 后不会执行到此处)
  }
#endif
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
