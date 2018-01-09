/***************************************************************************//**
 * @file    drv_usart.h
 * @brief   USART driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-02-18   onelife     Initial creation of USART module driver for
 *  MiniSTM32 (not yet supported slave mode and H/W flow control)
s ******************************************************************************/
#ifndef __DRV_USART_H__
#define __DRV_USART_H__

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define USART_TASK_STACK_SIZE           (512)
#define USART_RX_MESSAGE_SIZE           (4)
#define USART_RX_MESSAGE_QUEUE_SIZE     (1)
#define USART_DMA_QUEUE_SIZE            (5)
#define USART_INT_RX_BUFFER_SIZE        (64)

#define USART_WAIT_TIME_TX              (RT_TICK_PER_SECOND / 100 * 3)
#define USART_RETRY_TIMES_RX            (10)

/* Config options */
#define USART_CONFIG_DIRECT_EXE         (1 << 0)    /* Direct execute */
#define USART_CONFIG_9BIT               (1 << 1)    /* Word length */
#define USART_CONFIG_REMAP_GET(cfg)     ((cfg >> 2) & 0x03)
#define USART_CONFIG_DMA_TX             (1 << 4)    /* DMA TX */
#define USART_CONFIG_INT_RX             (1 << 5)    /* INT RX */
#define USART_CONFIG_CONSOLE            (1 << 6)    /* Console device */

/* Status options */
#define USART_STATUS_MASK               (0x0003)
#define USART_STATUS_DIRECT_EXE         (1 << 0)
#define USART_STATUS_9BIT               (1 << 1)
#define USART_STATUS_START              (1 << 2)
#define USART_STATUS_READ_ONLY          (1 << 3)
#define USART_STATUS_WRITE_ONLY         (1 << 4)
#define USART_STATUS_NONBLOCKING        (1 << 5)
#define USART_STATUS_TX_BUSY            (1 << 6)
#define USART_STATUS_RX_BUSY            (1 << 7)

/* USART command options */
#define USART_COMMAND_STATUS            (0x000001)
#define USART_COMMAND_OPEN              (0x000002)
#define USART_COMMAND_CLOSE             (0x000004)
#define USART_COMMAND_READ              (0x000008)
#define USART_COMMAND_WRITE             (0x000010)
#define USART_COMMAND_CONTROL           (0x000020)

#define USART_COMMAND_WAIT_TIME         (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
struct bsp_usart_device
{
    rt_uint8_t              counter;
    rt_uint8_t              number;
    volatile rt_uint16_t    status;
    USART_TypeDef           *usart_device;
    void                    *tx_mode;
    void                    *rx_mode;
    struct rt_semaphore     lock;
};

struct bsp_usart_cmd_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_uint8_t              *ptr;
};

struct bsp_usart_ret_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_err_t                ret;
};

union bsp_usart_exec_message
{
    struct bsp_usart_cmd_message cmd;
    struct bsp_usart_ret_message ret;
};

struct bsp_usart_task_struct
{
    struct rt_thread        thread;
    struct rt_messagequeue  rx_msgs;
    struct rt_event         tx_evts;
    rt_uint8_t              stack[USART_TASK_STACK_SIZE];
    rt_uint8_t              rx_msg_pool[USART_RX_MESSAGE_QUEUE_SIZE * \
                                (USART_RX_MESSAGE_SIZE + 4)];
};

struct bsp_usart_unit_struct
{
    struct bsp_usart_task_struct task;
    struct rt_device        device;
    struct bsp_usart_device usart;
};

struct bsp_usart_unit_init
{
    rt_uint8_t              number;
    rt_uint32_t             config;
    rt_uint32_t             frequency;
    const rt_uint8_t        *name;
    struct bsp_usart_unit_struct *unit;
};

struct bsp_usart_int_mode
{
    rt_uint8_t              buffer[USART_INT_RX_BUFFER_SIZE];
    rt_uint32_t             read_index, save_index;
};

struct bsp_usart_dma_node
{
    /* buffer info */
    rt_uint32_t             *data_ptr;
    rt_uint16_t             data_size;

    struct bsp_usart_dma_node *next, *prev;
};

struct bsp_usart_dma_mode
{
    DMA_Channel_TypeDef     *dma_chn;
    struct bsp_usart_dma_node *list_head, *list_tail;

    /* Memory pool */
    struct rt_mempool       dma_mp;
    rt_uint8_t              mem_pool[USART_DMA_QUEUE_SIZE * \
                                (sizeof(struct bsp_usart_dma_node) + 4)];
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_usart_init(void);

#endif /* __DRV_USART_H__ */
