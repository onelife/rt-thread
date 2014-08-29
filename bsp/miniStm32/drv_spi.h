/***************************************************************************//**
 * @file    drv_spi.h
 * @brief   SPI driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-01-24   onelife     Initial creation of SPI module (master mode) driver
 *  for MiniSTM32 (Modified from EFN32branch)
 * 2012-02-06   onelife     Add DMA TX support
 * 2012-02-08   onelife     Apply new driver implementation method
 ******************************************************************************/
#ifndef __DRV_SPI_H__
#define __DRV_SPI_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define SPI_TASK_STACK_SIZE         (512)
#define SPI_RX_MESSAGE_SIZE         (4)
#define SPI_RX_MESSAGE_QUEUE_SIZE   (1)
#define SPI_DMA_QUEUE_SIZE          (5)

#define SPI_WAIT_TIME_TX            (RT_TICK_PER_SECOND / 100 * 3)
#define SPI_RETRY_TIMES_RX          (10)

/* Config options */
#define SPI_CONFIG_DIRECT_EXE       (1 << 0)    /* Directly execute */
#define SPI_CONFIG_MASTER           (1 << 1)    /* Master mode */
#define SPI_CONFIG_AUTOCS           (1 << 2)    /* Auto chip select */
/*
    0: Clock idle low, sample on rising edge
    1: Clock idle low, sample on falling edge
    2: Clock idle high, sample on falling edge
    3: Clock idle high, sample on rising edge.
*/
#define SPI_CONFIG_CLK_MODE_GET(cfg)    ((cfg >> 3) & 0x0003)
#define SPI_CONFIG_REMAP_GET(cfg)   ((cfg >> 5) & 0x03)
#define SPI_CONFIG_DMA_TX           (1 << 7)    /* DMA TX */
#define SPI_CONFIG_DMA_RX           (1 << 8)    /* DMA RX */

/* Status options */
#define SPI_STATUS_MASK             (0x0007)
#define SPI_STATUS_DIRECT_EXE       (1 << 0) 
#define SPI_STATUS_MASTER           (1 << 1)
#define SPI_STATUS_AUTOCS           (1 << 2)
#define SPI_STATUS_START            (1 << 3)
#define SPI_STATUS_READ_ONLY        (1 << 4)
#define SPI_STATUS_WRITE_ONLY       (1 << 5)
#define SPI_STATUS_NONBLOCKING      (1 << 6)
#define SPI_STATUS_TX_BUSY          (1 << 7)
#define SPI_STATUS_RX_BUSY          (1 << 8)

/* SPI command options */
#define SPI_COMMAND_STATUS          (0x000001)
#define SPI_COMMAND_OPEN            (0x000002)
#define SPI_COMMAND_CLOSE           (0x000004)
#define SPI_COMMAND_READ            (0x000008)
#define SPI_COMMAND_WRITE           (0x000010)
#define SPI_COMMAND_CONTROL         (0x000020)

#define SPI_COMMAND_WAIT_TIME       (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
struct miniStm32_spi_device
{
    rt_uint8_t              counter;
    rt_uint8_t              number;
    volatile rt_uint16_t    status;
    SPI_TypeDef             *spi_device;
    void                    *tx_mode;
    void                    *rx_mode;
    struct rt_semaphore     lock;
};

struct miniStm32_spi_cmd_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_uint8_t              *ptr;
};

struct miniStm32_spi_ret_message
{
    rt_uint32_t             cmd;
    rt_uint32_t             other;
    rt_size_t               size;
    rt_err_t                ret;
};

union miniStm32_spi_exec_message
{
    struct miniStm32_spi_cmd_message cmd;
    struct miniStm32_spi_ret_message ret;
};

struct miniStm32_spi_task_struct
{
    struct rt_thread        thread;
    struct rt_messagequeue  rx_msgs;
    struct rt_event         tx_evts;
    rt_uint8_t              stack[SPI_TASK_STACK_SIZE];
    rt_uint8_t              rx_msg_pool[SPI_RX_MESSAGE_QUEUE_SIZE * \
                                (SPI_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_spi_unit_struct
{
    struct miniStm32_spi_task_struct task;
    struct rt_device        device;
    struct miniStm32_spi_device spi;
};

struct miniStm32_spi_unit_init
{
    rt_uint8_t              number;
    rt_uint32_t             config;
    const rt_uint8_t        *name;
    struct miniStm32_spi_unit_struct *unit;
};

struct miniStm32_spi_int_mode
{
    rt_uint8_t              *data_ptr;
    rt_uint8_t              data_size;
    rt_uint32_t             read_index, save_index;
};

struct miniStm32_spi_dma_node
{
    /* buffer info */
    rt_uint32_t             *data_ptr;
    rt_uint16_t             data_size;

    struct miniStm32_spi_dma_node *next, *prev;
};

struct miniStm32_spi_dma_mode
{
    DMA_Channel_TypeDef     *dma_chn;
    struct miniStm32_spi_dma_node *list_head, *list_tail;

    /* Memory pool */
    struct rt_mempool       dma_mp;
    rt_uint8_t              mem_pool[SPI_DMA_QUEUE_SIZE * \
                                (sizeof(struct miniStm32_spi_dma_node) + 4)];
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_spi_init(void);

#endif /* __DRV_SPI_H__ */
