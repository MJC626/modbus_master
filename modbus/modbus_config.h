#ifndef MODBUS_CONFIG_H
#define MODBUS_CONFIG_H

#include <stdint.h>

// 定义Modbus命令参数
#define COMMAND_COUNT 4


// Modbus命令结构体
typedef struct {
    uint8_t addr;//从站号
    uint8_t func;//功能码
    uint16_t reg;//寄存器地址
    uint16_t num;//寄存器个数或者单个写入寄存器值
    const uint8_t *buf;//写入多个寄存器值
} ModbusCommand;

// Modbus命令配置数组
static const ModbusCommand commands[COMMAND_COUNT] = {
        {0x01, 0x01, 0x0000, 0x0006, NULL},
        {0x02, 0x02, 0x0000, 0x0006, NULL},
        {0x03, 0x03, 0x0000, 0x0006, NULL},
        {0x04, 0x04, 0x0000, 0x0006, NULL},
        //{0x01, 0x10, 0x0060, 0x0004, (const uint8_t[]){0x11, 0x22, 0x33, 0x44}}
};

#endif // MODBUS_CONFIG_H
