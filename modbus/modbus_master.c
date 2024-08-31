// modbus_master.c
#include "modbus_master.h"

ModbusHandler g_tModH;

/* CRC-16-CCITT 校验计算函数 */
static uint16_t CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t byte;

    while (length--) {
        byte = *data++;
        crc ^= (uint16_t)byte;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}


void MODH_SendCommand(const uint8_t _addr, const uint8_t _func, const uint16_t _reg, const uint16_t _num, const uint8_t *_buf)
{
    uint16_t i;
    uint16_t crc;

    g_tModH.TxCount = 0;
    g_tModH.TxBuf[g_tModH.TxCount++] = _addr;        /* 从站地址 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _func;        /* 功能码 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg >> 8;    /* 寄存器编号 高字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = _reg;         /* 寄存器编号 低字节 */

    /* 根据功能码处理不同的数据 */
    if (_func == 0x01 || _func == 0x02 || _func == 0x03 || _func == 0x04) {
        g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;   /* 寄存器个数 高字节 */
        g_tModH.TxBuf[g_tModH.TxCount++] = _num;        /* 寄存器个数 低字节 */
        g_tModH.RegNum = _num;
    }
    else if (_func == 0x05 || _func == 0x06) {
        g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;   /* 寄存器值 高字节 */
        g_tModH.TxBuf[g_tModH.TxCount++] = _num;        /* 寄存器值 低字节 */
    }
    else if (_func == 0x10) {
        g_tModH.TxBuf[g_tModH.TxCount++] = _num >> 8;   /* 寄存器个数 高字节 */
        g_tModH.TxBuf[g_tModH.TxCount++] = _num;        /* 寄存器个数 低字节 */
        g_tModH.TxBuf[g_tModH.TxCount++] = 2 * _num;    /* 数据字节数 */

        for (i = 0; i < 2 * _num; i++) {
            if (g_tModH.TxCount > TX_BUF_SIZE - 3) {
                return; /* 数据超过缓冲区长度，直接丢弃不发送 */
            }
            g_tModH.TxBuf[g_tModH.TxCount++] = _buf[i]; /* 数据 */
        }
    }

    /* 计算 CRC 校验码 */
    crc = CRC16(g_tModH.TxBuf, g_tModH.TxCount);
    g_tModH.TxBuf[g_tModH.TxCount++] = crc & 0xFF;    /* CRC 低字节 */
    g_tModH.TxBuf[g_tModH.TxCount++] = (crc >> 8) & 0xFF; /* CRC 高字节 */

    /* 根据功能码设置相应的标志位和寄存器地址 */
    switch (_func) {
        case 0x01:
            g_tModH.fAck01H = 0;
            g_tModH.Reg01H = _reg;
            break;
        case 0x02:
            g_tModH.fAck02H = 0;
            g_tModH.Reg02H = _reg;
            break;
        case 0x03:
            g_tModH.fAck03H = 0;
            g_tModH.Reg03H = _reg;
            break;
        case 0x04:
            g_tModH.fAck04H = 0;
            g_tModH.Reg04H = _reg;
            break;
        case 0x05:
            g_tModH.fAck05H = 0;
            break;
        case 0x06:
            g_tModH.fAck06H = 0;
            break;
        default:
            break;
    }
}
#define REG_SIZE 64

// 定义全局的Modbus寄存器数组
uint16_t coils[REG_SIZE];                // Coils
uint16_t discreteInputs[REG_SIZE];       // Discrete Inputs
uint16_t holdingRegisters[REG_SIZE];     // Holding Registers
uint16_t inputRegisters[REG_SIZE];       // Input Registers


// Modbus回复报文解析函数
void process_modbus_message(uint8_t *data, uint16_t length)
{
    if (length < 5) {
        return;  // 数据长度太短，无法处理
    }

    uint8_t functionCode = data[1];  // 功能码
    switch (functionCode)
    {
        case 0x01:  // 读线圈状态
            process_coils(data, length);
            break;

        case 0x02:  // 读离散输入状态
            process_discrete_inputs(data, length);
            break;

        case 0x03:  // 读保持寄存器
            process_holding_registers(data, length);
            break;

        case 0x04:  // 读输入寄存器
            process_input_registers(data, length);
            break;

        default:
            // 处理未识别的功能码
            break;
    }
}

void process_coils(uint8_t *data, uint16_t length)
{
    if (!g_tModH.fAck01H) {
        return;  // 标志位不正确，忽略该报文
    }

    // 根据协议，提取数据并写入对应的寄存器
    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    for (uint16_t i = 0; i < quantity; i++) {
        if (i < REG_SIZE) {
            coils[startAddress + i] = (data[7 + i / 8] >> (i % 8)) & 0x01;
        }
    }
}

void process_discrete_inputs(uint8_t *data, uint16_t length)
{
    if (!g_tModH.fAck02H) {
        return;  // 标志位不正确，忽略该报文
    }

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    for (uint16_t i = 0; i < quantity; i++) {
        if (i < REG_SIZE) {
            discreteInputs[startAddress + i] = (data[7 + i / 8] >> (i % 8)) & 0x01;
        }
    }
}

void process_holding_registers(uint8_t *data, uint16_t length)
{
    if (!g_tModH.fAck03H) {
        return;  // 标志位不正确，忽略该报文
    }

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    for (uint16_t i = 0; i < quantity; i++) {
        if (i < REG_SIZE) {
            holdingRegisters[startAddress + i] = (data[7 + i * 2] << 8) | data[8 + i * 2];
        }
    }
}

void process_input_registers(uint8_t *data, uint16_t length)
{
    if (!g_tModH.fAck04H) {
        return;  // 标志位不正确，忽略该报文
    }

    uint16_t startAddress = (data[2] << 8) | data[3];
    uint16_t quantity = (data[4] << 8) | data[5];

    for (uint16_t i = 0; i < quantity; i++) {
        if (i < REG_SIZE) {
            inputRegisters[startAddress + i] = (data[7 + i * 2] << 8) | data[8 + i * 2];
        }
    }
}
