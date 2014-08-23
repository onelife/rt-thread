/***************************************************************************//**
 * @file    drv_gpio_spi.h
 * @brief   GPIO SPI driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author  onelife
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-01-31   onelife     Initial creation of GPIO simulation SPI module
 *  (master mode) driver for MiniSTM32
 * 2012-02-11   onelife     Apply new driver implementation method
 ******************************************************************************/
#ifndef __DRV_GPIO_SPI_H__
#define __DRV_GPIO_SPI_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define GPIO_SPI_TASK_STACK_SIZE        (512)
#define GPIO_SPI_RX_MESSAGE_SIZE        (4)
#define GPIO_SPI_RX_MESSAGE_QUEUE_SIZE  (1)
#define GPIO_SPI_DMA_QUEUE_SIZE         (5)

#define GPIO_SPI_WAIT_TIME_TX           (RT_TICK_PER_SECOND / 100 * 3)
#define GPIO_SPI_RETRY_TIMES_RX         (10)

/* Config options */
#define GPIO_SPI_CONFIG_MASTER          (1 << 0)    /* Master mode */
#define GPIO_SPI_CONFIG_AUTOCS          (1 << 1)    /* Auto chip select */
/*
    b'00: Clock idle low, sample on falling edge
    b'01: Clock idle low, sample on rising edge
    b'10: Clock idle high, sample on falling edge
    b'11: Clock idle high, sample on rising edge.
*/
#define GPIO_SPI_CONFIG_SAMPLE_EDGE     (1 << 2)
#define GPIO_SPI_CONFIG_IDLE_LEVEL      (1 << 3)
#define GPIO_SPI_CONFIG_CLK_MODE_GET(cfg)       ((cfg >> 2) & 0x0003)
#define GPIO_SPI_CONFIG_CLK_MODE_SET(cfg, mode) ((cfg & 0xFFF3) | ((mode & 0x0003) << 2))

/* Status options */
#define GPIO_SPI_STATUS_MASK            (0x0F)
#define GPIO_SPI_STATUS_MASTER          (1 << 0)
#define GPIO_SPI_STATUS_AUTOCS          (1 << 1)
#define GPIO_SPI_STATUS_SAMPLE_EDGE     (1 << 2)
#define GPIO_SPI_STATUS_IDLE_LEVEL      (1 << 3)
#define GPIO_SPI_STATUS_READ_ONLY       (1 << 4)
#define GPIO_SPI_STATUS_WRITE_ONLY      (1 << 5)
#define GPIO_SPI_STATUS_NONBLOCKING     (1 << 6)
#define GPIO_SPI_STATUS_TX_BUSY         (1 << 7)
#define GPIO_SPI_STATUS_RX_BUSY         (1 << 8)

/* SPI command options */
#define GPIO_SPI_COMMAND_STATUS         (0x000001)
#define GPIO_SPI_COMMAND_OPEN           (0x000002)
#define GPIO_SPI_COMMAND_CLOSE          (0x000004)
#define GPIO_SPI_COMMAND_READ           (0x000008)
#define GPIO_SPI_COMMAND_WRITE          (0x000010)
#define GPIO_SPI_COMMAND_CONTROL        (0x000020)

#define GPIO_SPI_COMMAND_WAIT_TIME      (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef        *sck_port;
    GPIO_TypeDef        *mosi_port;
    GPIO_TypeDef        *miso_port;
    GPIO_TypeDef        *cs_port;
    rt_uint16_t         sck_pin;
    rt_uint16_t         mosi_pin;
    rt_uint16_t         miso_pin;
    rt_uint16_t         cs_pin;
} GPIO_SPI_TypeDef;

struct miniStm32_gpio_spi_device
{
    rt_uint8_t          counter;
    rt_uint8_t          number;
    rt_uint32_t         delay;
    volatile rt_uint16_t state;
    GPIO_SPI_TypeDef    spi_device;
    void                *rx_mode;   // TODO: slave mode INT RX?
    struct rt_semaphore lock;
};

struct miniStm32_gpio_spi_cmd_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_uint8_t          *ptr;
};

struct miniStm32_gpio_spi_ret_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_err_t            ret;
};

union miniStm32_gpio_spi_exec_message
{
    struct miniStm32_gpio_spi_cmd_message cmd;
    struct miniStm32_gpio_spi_ret_message ret;
};

struct miniStm32_gpio_spi_task_struct
{
    struct rt_thread    thread;
    struct rt_messagequeue rx_msgs;
    struct rt_event     tx_evts;
    rt_uint8_t          stack[GPIO_SPI_TASK_STACK_SIZE];
    rt_uint8_t          rx_msg_pool[GPIO_SPI_RX_MESSAGE_QUEUE_SIZE * \
                            (GPIO_SPI_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_gpio_spi_unit_struct
{
    struct miniStm32_gpio_spi_task_struct task;
    struct rt_device    device;
    struct miniStm32_gpio_spi_device spi;
};

struct miniStm32_gpio_spi_unit_init
{
    rt_uint8_t          number;
    rt_uint32_t         config;
    const rt_uint8_t    *name;
    struct miniStm32_gpio_spi_unit_struct *unit;
};

struct miniStm32_gpio_spi_int_mode
{
    rt_uint8_t          *data_ptr;
    rt_uint8_t          data_size;
    rt_uint32_t         read_index, save_index;
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_gpio_spi_init(void);

#endif /* __DRV_GPIO_SPI_H__ */
