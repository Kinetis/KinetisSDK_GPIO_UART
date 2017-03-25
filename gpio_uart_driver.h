#ifndef _GPIO_UART_DRIVER_H_
#define _GPIO_UART_DRIVER_H_

#include "fsl_common.h"
#include "fsl_pit.h"
#include "board.h"
#include "fsl_gpio.h"
#include "fsl_port.h"



enum 
{
   UART_IDLE = 0,
   UART_START,
   UART_TRANSFERRING,
   UART_PARITY,
   UART_STOP
};


typedef struct _gpio_uart_config
{
    GPIO_Type *tx_gpio_base;    //
    GPIO_Type *rx_gpio_base;
    PORT_Type *tx_port_base;
    PORT_Type *rx_port_base;
    uint32_t tx_pin_num; 
    uint32_t rx_pin_num;
    
    uint8_t databit;   //数据位长度，通常是8
    uint8_t stop;      //停止位长度
    uint8_t parity;    //校验位，0表示不校验，1表示奇校验，2表示偶校验  
    pit_chnl_t pitch_tx;    //占用定时器通道号，最大不能超过芯片PIT内部通道个数
    pit_chnl_t pitch_rx;    //占用定时器通道号，最大不能超过芯片PIT内部通道个数
    uint32_t baudrate;
 
} gpio_uart_config_t;


void gpio_uart_init(const gpio_uart_config_t *config);
void gpio_uart_send(uint8_t *databuf, uint32_t num);
void delayx(int x);
void gpio_uart_demo_init();

#endif