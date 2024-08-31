// modbus_handler.h
#ifndef MODBUS_MASTER_H
#define MODBUS_MASTER_H

#include <stdio.h>
#include <stdint.h>

#define TX_BUF_SIZE 256  // 假设缓冲区大小为256字节

typedef struct {
    uint8_t TxBuf[TX_BUF_SIZE];//发送缓冲区，用于存放组装好的Modbus请求帧。
    uint16_t TxCount;//发送缓冲区计数器，用于跟踪当前已经写入缓冲区的数据字节数。
    uint16_t RegNum;//寄存器数量，用于记录本次操作的寄存器数量。
    uint8_t fAck01H;//标志位，用于指示是否接收到对应功能码的响应
    uint8_t fAck02H;
    uint8_t fAck03H;
    uint8_t fAck04H;
    uint8_t fAck05H;
    uint8_t fAck06H;
    uint16_t Reg01H;//寄存器地址，用于记录本次操作的寄存器启始地址。
    uint16_t Reg02H;
    uint16_t Reg03H;
    uint16_t Reg04H;
    uint8_t RxBuf[256];//接收缓冲区，用于存放组装好的Modbus请求帧。
} ModbusHandler;

extern ModbusHandler g_tModH;

void MODH_SendCommand(const uint8_t _addr, const uint8_t _func, const uint16_t _reg, const uint16_t _num, const uint8_t *_buf);

void process_modbus_message(uint8_t *data, uint16_t length);

void process_coils(uint8_t *data, uint16_t length);
void process_discrete_inputs(uint8_t *data, uint16_t length);
void process_holding_registers(uint8_t *data, uint16_t length);
void process_input_registers(uint8_t *data, uint16_t length);


#endif // MODBUS_MASTER_H
