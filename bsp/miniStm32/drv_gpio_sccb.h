/***************************************************************************//**
 * @file    drv_gpio_sccb.h
 * @brief   GPIO simulation SCCB driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author  onelife
 * @version 1.0 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-02-20   onelife     Initial creation of GPIO simulation SCCB module
 *  (master mode) driver for MiniSTM32
 ******************************************************************************/
#ifndef __DRV_GPIO_SCCB_H__
#define __DRV_GPIO_SCCB_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define GPIO_SCCB_TASK_STACK_SIZE       (512)
#define GPIO_SCCB_RX_MESSAGE_SIZE       (4)
#define GPIO_SCCB_RX_MESSAGE_QUEUE_SIZE (1)
#define GPIO_SCCB_DMA_QUEUE_SIZE        (5)

#define GPIO_SCCB_WAIT_TIME_TX          (RT_TICK_PER_SECOND / 100 * 3)
#define GPIO_SCCB_RETRY_TIMES_RX        (10)

/* Config options */
#define GPIO_SCCB_CONFIG_MASTER         (1 << 0)    /* Master mode */

/* Status options */
#define GPIO_SCCB_STATUS_MASK           (0x01)
#define GPIO_SCCB_STATUS_MASTER         (1 << 0)
#define GPIO_SCCB_STATUS_READ_ONLY      (1 << 4)
#define GPIO_SCCB_STATUS_WRITE_ONLY     (1 << 5)
#define GPIO_SCCB_STATUS_NONBLOCKING    (1 << 6)
#define GPIO_SCCB_STATUS_TX_BUSY        (1 << 7)
#define GPIO_SCCB_STATUS_RX_BUSY        (1 << 8)

/* SCCB command options */
#define GPIO_SCCB_COMMAND_STATUS        (0x000001)
#define GPIO_SCCB_COMMAND_OPEN          (0x000002)
#define GPIO_SCCB_COMMAND_CLOSE         (0x000004)
#define GPIO_SCCB_COMMAND_READ          (0x000008)
#define GPIO_SCCB_COMMAND_WRITE         (0x000010)
#define GPIO_SCCB_COMMAND_CONTROL       (0x000020)

#define GPIO_SCCB_COMMAND_WAIT_TIME     (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef        *scl_port;
    GPIO_TypeDef        *sda_port;
    rt_uint16_t         scl_pin;
    rt_uint16_t         sda_pin;
} GPIO_SCCB_TypeDef;

struct miniStm32_gpio_sccb_device
{
    rt_uint8_t          counter;
    rt_uint8_t          number;
    rt_uint32_t         delay;
    volatile rt_uint16_t status;
    GPIO_SCCB_TypeDef   sccb_device;
    void                *rx_mode;   // TODO: slave mode INT RX?
    struct rt_semaphore lock;
};

struct miniStm32_gpio_sccb_cmd_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_uint8_t          *ptr;
};

struct miniStm32_gpio_sccb_ret_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_err_t            ret;
};

union miniStm32_gpio_sccb_exec_message
{
    struct miniStm32_gpio_sccb_cmd_message cmd;
    struct miniStm32_gpio_sccb_ret_message ret;
};

struct miniStm32_gpio_sccb_task_struct
{
    struct rt_thread    thread;
    struct rt_messagequeue rx_msgs;
    struct rt_event     tx_evts;
    rt_uint8_t          stack[GPIO_SCCB_TASK_STACK_SIZE];
    rt_uint8_t          rx_msg_pool[GPIO_SCCB_RX_MESSAGE_QUEUE_SIZE * \
                            (GPIO_SCCB_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_gpio_sccb_unit_struct
{
    struct miniStm32_gpio_sccb_task_struct task;
    struct rt_device    device;
    struct miniStm32_gpio_sccb_device sccb;
};

struct miniStm32_gpio_sccb_unit_init
{
    rt_uint8_t          number;
    rt_uint32_t         config;
    const rt_uint8_t    *name;
    struct miniStm32_gpio_sccb_unit_struct *unit;
};

struct miniStm32_gpio_sccb_int_mode
{
    rt_uint8_t          *data_ptr;
    rt_uint8_t          data_size;
    rt_uint32_t         read_index, save_index;
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_gpio_sccb_init(void);

#endif /* __DRV_GPIO_SCCB_H__ */
