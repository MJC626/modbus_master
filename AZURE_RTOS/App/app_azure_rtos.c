/* Includes ------------------------------------------------------------------*/
#include "app_azure_rtos.h"

/* USER CODE BEGIN Includes */
#include "usart.h"  // 引入UART相关的头文件
#include "tx_api.h"
#include <string.h>

#include "modbus_master.h"
#include "modbus_config.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define TIMEOUT_MS 150 // 定义超时时间
#define EVENT_RX_COMPLETE 0x01
#define EVENT_TIMEOUT     0x02

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static UCHAR tx_byte_pool_buffer[TX_APP_MEM_POOL_SIZE];
static TX_BYTE_POOL tx_app_byte_pool;
static TX_THREAD thread1;
static TX_TIMER timeout_timer;
static TX_EVENT_FLAGS_GROUP event_flags;

#define RX_BUFFER_SIZE 256
static uint8_t rx_buffer[RX_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void thread1_entry(ULONG thread_input);
void timeout_timer_callback(ULONG input);
void process_received_data(uint8_t *data, uint16_t length);

extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PFP */

/**
  * @brief  Define the initial system.
  * @param  first_unused_memory : Pointer to the first unused memory
  * @retval None
  */
VOID tx_application_define(VOID *first_unused_memory)
{
    /* USER CODE BEGIN  tx_application_define */
    VOID *memory_ptr;

    if (tx_byte_pool_create(&tx_app_byte_pool, "Tx App memory pool", tx_byte_pool_buffer, TX_APP_MEM_POOL_SIZE) != TX_SUCCESS)
    {
        // 处理内存池创建失败的情况
    }
    else
    {
        memory_ptr = (VOID *)&tx_app_byte_pool;

        // 创建串口发送任务
        tx_thread_create(&thread1, "UART Task", thread1_entry, 0,
                         memory_ptr, 1024, 1, 1, TX_NO_TIME_SLICE, TX_AUTO_START);

        // 创建超时定时器
        tx_timer_create(&timeout_timer, "Timeout Timer", timeout_timer_callback,
                        0, TIMEOUT_MS, TIMEOUT_MS, TX_AUTO_ACTIVATE);

        // 创建事件标志组
        tx_event_flags_create(&event_flags, "UART Event Flags");
    }
    /* USER CODE END  tx_application_define */
}

/* USER CODE BEGIN 0 */

// 串口空闲中断处理函数
void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart); // 清除空闲中断标志位

        // 计算接收到的数据长度
        uint16_t length = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

        if (length > 0)
        {
            // 处理接收到的Modbus报文
            process_modbus_message(rx_buffer, length);

            // 数据接收完成，设置事件标志
            tx_event_flags_set(&event_flags, EVENT_RX_COMPLETE, TX_OR);
        }

        // 暂停 DMA 接收
        HAL_UART_DMAStop(huart);
    }
}

// 超时定时器回调函数
void timeout_timer_callback(ULONG input)
{
    tx_event_flags_set(&event_flags, EVENT_TIMEOUT, TX_OR);
}

// 串口发送任务
void thread1_entry(ULONG thread_input)
{
    // 初始化Modbus Handler
    g_tModH.TxCount = 0;

    while (1)
    {
        // 发送每个Modbus命令
        for (int i = 0; i < COMMAND_COUNT; i++)
        {
            const ModbusCommand *cmd = &commands[i];
            MODH_SendCommand(cmd->addr, cmd->func, cmd->reg, cmd->num, cmd->buf);

            // 直接通过串口发送缓冲区内容
            HAL_UART_Transmit(&huart1, g_tModH.TxBuf, g_tModH.TxCount, HAL_MAX_DELAY);

            // 启动DMA接收和超时定时器
            HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
            tx_timer_activate(&timeout_timer);

            // 等待接收完成或超时
            ULONG flags;
            tx_event_flags_get(&event_flags, EVENT_RX_COMPLETE | EVENT_TIMEOUT, TX_OR_CLEAR, &flags, TX_WAIT_FOREVER);

            // 停止超时定时器
            tx_timer_deactivate(&timeout_timer);

            if (flags & EVENT_RX_COMPLETE)
            {
                // 重置并重启超时定时器
                tx_timer_change(&timeout_timer, TIMEOUT_MS, TIMEOUT_MS);

                // 计算接收到的数据长度
                uint16_t length = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

                // 重新启动 DMA 接收，确保任务能够继续执行
                HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
            }
            else if (flags & EVENT_TIMEOUT)
            {
                // 超时处理，发送原始16进制数据 0xFFFF0A
                //uint8_t timeout_data[] = {0xFF, 0xFF, 0x0A};
                //HAL_UART_Transmit(&huart1, timeout_data, sizeof(timeout_data), HAL_MAX_DELAY);
            }
        }

        // 继续循环发送下一个命令
    }
}

/* USER CODE END 0 */
