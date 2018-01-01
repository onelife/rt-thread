/***************************************************************************//**
 * @file    drv_spi.c
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

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_spi.h"
#if (defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2) || defined(BSP_USING_SPI3))
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SPI1_DR_Base                        (SPI1_BASE + 0x0C)
#define SPI2_DR_Base                        (SPI2_BASE + 0x0C)
#define SPI3_DR_Base                        (SPI3_BASE + 0x0C)

#define DMA1_Channel_SPI1_TX                (DMA1_Channel3)
#define DMA1_Channel_SPI1_TX_Num            (3)
#define DMA1_Channel_SPI1_TX_IRQn           (DMA1_Channel3_IRQn)
#define DMA1_Channel_SPI1_RX                (DMA1_Channel2)
#define DMA1_Channel_SPI1_RX_Num            (2)
#define DMA1_Channel_SPI1_RX_IRQn           (DMA1_Channel2_IRQn)

#define DMA1_Channel_SPI2_TX                (DMA1_Channel5)
#define DMA1_Channel_SPI2_TX_Num            (5)
#define DMA1_Channel_SPI2_TX_IRQn           (DMA1_Channel5_IRQn)
#define DMA1_Channel_SPI2_RX                (DMA1_Channel4)
#define DMA1_Channel_SPI2_RX_Num            (4)
#define DMA1_Channel_SPI2_RX_IRQn           (DMA1_Channel4_IRQn)

#define DMA2_Channel_SPI3_TX                (DMA2_Channel2)
#define DMA2_Channel_SPI3_TX_Num            (2)
#define DMA2_Channel_SPI3_TX_IRQn           (DMA2_Channel2_IRQn)
#define DMA2_Channel_SPI3_RX                (DMA2_Channel1)
#define DMA2_Channel_SPI3_RX_Num            (1)
#define DMA2_Channel_SPI3_RX_IRQn           (DMA2_Channel1_IRQn)

/* Private macro -------------------------------------------------------------*/
#ifdef BSP_SPI_DEBUG
#define spi_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define spi_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_spi_task_struct *spi_tasks[3];

#if (defined(RCC_APB2ENR_SPI1EN) || defined(BSP_USING_SPI1))
static struct bsp_spi_unit_struct spi1;
 #if (SPI1_SPI_MODE & SPI_CONFIG_DMA_TX)
 static struct bsp_spi_dma_mode spi1_dma_tx;
 #endif
#endif

#if (defined(RCC_APB1ENR_SPI2EN) || defined(BSP_USING_SPI2))
static struct bsp_spi_unit_struct spi2;
 #if (SPI2_SPI_MODE & SPI_CONFIG_DMA_TX)
 static struct bsp_spi_dma_mode spi2_dma_tx;
 #endif
#endif

#if (defined(RCC_APB1ENR_SPI3EN) || defined(BSP_USING_SPI3))
static struct bsp_spi_unit_struct spi3;
 #if (SPI3_SPI_MODE & SPI_CONFIG_DMA_TX)
 static struct bsp_spi_dma_mode spi3_dma_tx;
 #endif
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *  DMA for SPI TX interrupt handler
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_spi_dma_tx_isr(rt_device_t dev)
{
    struct bsp_spi_device *spi;
    struct bsp_spi_dma_mode *dma_tx;
    struct bsp_spi_dma_node *dma_node;
    rt_uint32_t level;

    spi = (struct bsp_spi_device *)dev->user_data;
    dma_tx = (struct bsp_spi_dma_mode *)spi->tx_mode;
    dma_node = dma_tx->list_head;

    RT_ASSERT((dev->flag & RT_DEVICE_FLAG_DMA_TX) && (dma_node != RT_NULL));

    /* Invoke call to notify tx complete */
    if (dev->tx_complete != RT_NULL)
    {
        dev->tx_complete(dev, dma_node->data_ptr);
    }

    /* Remove the node */
    level = rt_hw_interrupt_disable();
    dma_tx->list_head = dma_node->next;
    if (dma_tx->list_head == RT_NULL)  /* data link empty */
    {
        dma_tx->list_tail = RT_NULL;
    }
    rt_hw_interrupt_enable(level);

    /* Release the node */
    rt_mp_free(dma_node);

    SPI_I2S_DMACmd(spi->spi_device, SPI_I2S_DMAReq_Tx, DISABLE);
    DMA_Cmd(dma_tx->dma_chn, DISABLE);

    if (dma_tx->list_head != RT_NULL)
    {
        /* Enable DMA TX */
        dma_tx->dma_chn->CMAR = (rt_uint32_t)dma_tx->list_head->data_ptr;
        dma_tx->dma_chn->CNDTR = (rt_uint32_t)dma_tx->list_head->data_size;
        DMA_Cmd(dma_tx->dma_chn, ENABLE);
        SPI_I2S_DMACmd(spi->spi_device, SPI_I2S_DMAReq_Tx, ENABLE);
    }
    else
    {
        /* Reset status */
        spi->status &= ~(rt_uint16_t)SPI_STATUS_TX_BUSY;
    }
}

static rt_err_t bsp_spi_dma_tx(
    struct bsp_spi_device *spi,
    rt_uint32_t *buffer,
    rt_uint16_t size)
{
    struct bsp_spi_dma_mode *dma_tx;
    struct bsp_spi_dma_node *dma_node;
    rt_uint32_t level;

    dma_tx = (struct bsp_spi_dma_mode *)spi->tx_mode;

    /* Allocate a data node */
    dma_node = (struct bsp_spi_dma_node *)rt_mp_alloc(
            &(dma_tx->dma_mp), RT_WAITING_FOREVER);
    if (dma_node == RT_NULL)
    {
        return -RT_ENOMEM;
    }

    dma_node->data_ptr  = buffer;
    dma_node->data_size = size;
    dma_node->next      = RT_NULL;

    /* Insert the node */
    level = rt_hw_interrupt_disable();

    dma_node->prev = dma_tx->list_tail;
    if (dma_tx->list_tail != RT_NULL)
    {
        dma_tx->list_tail->next = dma_node;
    }
    dma_tx->list_tail = dma_node;

    if (dma_tx->list_head == RT_NULL)
    {
        dma_tx->list_head = dma_node;

        /* Enable DMA TX */
        spi->status |= (rt_uint16_t)SPI_STATUS_TX_BUSY;
        dma_tx->dma_chn->CMAR = (rt_uint32_t)buffer;
        dma_tx->dma_chn->CNDTR = (rt_uint32_t)size;
        DMA_Cmd(dma_tx->dma_chn, ENABLE);

        rt_hw_interrupt_enable(level);

        SPI_I2S_DMACmd(spi->spi_device, SPI_I2S_DMAReq_Tx, ENABLE);
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }

    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *  SPI RX data valid interrupt handler
 *
 * @details
 *
 * @note
 *  9-bit SPI mode has not implemented yet and SPI slave mode is untested
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_spi_rx_isr(rt_device_t dev)
{
    struct bsp_spi_device     *spi;
    struct bsp_spi_int_mode   *int_rx;

    /* interrupt mode receive */
    RT_ASSERT(dev->flag & RT_DEVICE_FLAG_INT_RX);
    spi = (struct bsp_spi_device *)(dev->user_data);
    int_rx = (struct bsp_spi_int_mode *)(spi->rx_mode);

    /* Set status */
    spi->status |= SPI_STATUS_RX_BUSY;

    /* save into rx buffer */
    while (spi->spi_device->SR & SPI_I2S_FLAG_RXNE)
    {
        rt_base_t level;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        /* save character */
        int_rx->data_ptr[int_rx->save_index] = \
            (rt_uint8_t)(spi->spi_device->DR & 0x00ff);
        int_rx->save_index ++;
        if (int_rx->save_index >= SPI_RX_BUFFER_SIZE)
            int_rx->save_index = 0;

        /* if the next position is read index, discard this 'read char' */
        if (int_rx->save_index == int_rx->read_index)
        {
            if (++int_rx->read_index >= SPI_RX_BUFFER_SIZE)
            {
                int_rx->read_index = 0;
            }
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* Reset status */
    spi->status &= ~(rt_uint16_t)SPI_STATUS_RX_BUSY;
    /* invoke callback */
    if (dev->rx_indicate != RT_NULL)
    {
        rt_size_t rx_len;

        /* get rx length */
        rx_len = int_rx->read_index > int_rx->save_index ?
            SPI_RX_BUFFER_SIZE - int_rx->read_index + int_rx->save_index : \
            int_rx->save_index - int_rx->read_index;

        dev->rx_indicate(dev, rx_len);
    }
}

/***************************************************************************//**
 * @brief
 *   Open SPI device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] oflag
 *   Device open flag
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t spi_task_exec(rt_uint8_t number,
    union bsp_spi_exec_message *exec_msg,
    rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &spi_tasks[number - 1]->rx_msgs,
            (void *)&exec_msg,
            SPI_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(SPI_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        spi_debug("SPI%d err: send cmd failed! (%x)\n", number, ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &spi_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &spi_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            spi_debug("SPI%d err: receive event failed! (%x)\n", number, ret);
        }
    }

    return ret;
}

static void spi_task_open(struct bsp_spi_unit_struct *cfg,
   union bsp_spi_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_uint16_t oflag;

    oflag = (rt_uint16_t)exec_msg->cmd.other;
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
    {
        cfg->spi.status |= SPI_STATUS_READ_ONLY;
    }
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
    {
        cfg->spi.status |= SPI_STATUS_WRITE_ONLY;
    }
    if (oflag & (rt_uint16_t)RT_DEVICE_OFLAG_NONBLOCKING)
    {
        cfg->spi.status |= SPI_STATUS_NONBLOCKING;
    }

    cfg->spi.counter++;

    spi_debug("SPI%d: Open with flag %x\n", cfg->spi.number, oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void spi_task_close(struct bsp_spi_unit_struct *cfg,
    union bsp_spi_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    cfg->spi.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void spi_task_read(struct bsp_spi_unit_struct *cfg,
    union bsp_spi_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->spi.status & SPI_STATUS_WRITE_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;

    rt_uint8_t inst_len = *(exec_msg->cmd.ptr);
    rt_uint8_t *inst_ptr = exec_msg->cmd.ptr + 1;
    rt_uint8_t *rx_buf = *((rt_uint8_t **)(exec_msg->cmd.ptr + inst_len + 1));
    rt_off_t i;
    rt_uint8_t retry;

    ptr = inst_ptr;
    len = inst_len;
    spi_debug("SPI%d: RX, TX INS (%d)\n", cfg->spi.number, len);
    /* Write instructions */
    while (len)
    {
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = (rt_uint16_t)*(ptr++);
        len--;
    }

    /* Flushing RX */
    *(rt_uint16_t *)0x00 = cfg->spi.spi_device->DR;

    /* Skip some bytes if necessary */
    spi_debug("SPI%d: RX pos %d\n", cfg->spi.number, pos);
    for (i = 0; i < pos; i++)
    {
        /* dummy write */
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = 0x00ff;
        /* dummy read */
        retry = 0;
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_RXNE))
        {
            if (++retry > SPI_RETRY_TIMES_RX)
            {
                exec_msg->ret.size = 0;
                exec_msg->ret.ret = -RT_ERROR;
                return;
            }
        }
        *((rt_uint16_t *)0x00) = cfg->spi.spi_device->DR;
    }

    ptr = rx_buf;
    len = exec_msg->cmd.size;
    spi_debug("SPI%d: RX data (%d)\n", cfg->spi.number, len);
    /* Read data */
    while (len)
    {
        /* dummy write */
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = 0x00ff;
        /* read a byte of data */
        retry = 0;
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_RXNE))
        {
            if (++retry > SPI_RETRY_TIMES_RX)
            {
                exec_msg->ret.size = exec_msg->cmd.size - len;
                exec_msg->ret.ret = -RT_ERROR;
                return;
            }
        }
        *(ptr++) = (rt_uint8_t)(cfg->spi.spi_device->DR & 0x00ff);
        len--;
    }

    exec_msg->ret.size = exec_msg->cmd.size - len;
    exec_msg->ret.ret = RT_EOK;
}

static void spi_task_write(struct bsp_spi_unit_struct *cfg,
    union bsp_spi_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->spi.status & SPI_STATUS_READ_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;

    rt_uint8_t inst_len = *(exec_msg->cmd.ptr);
    rt_uint8_t *inst_ptr = exec_msg->cmd.ptr + 1;
    rt_uint8_t *tx_buf = *((rt_uint8_t **)(exec_msg->cmd.ptr + inst_len + 1));

    ptr = inst_ptr;
    len = inst_len;
    spi_debug("SPI%d: TX INS (%d)\n", cfg->spi.number, len);
    /* Write instructions */
    while (len)
    {
        while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
        cfg->spi.spi_device->DR = (rt_uint16_t)*(ptr++);
        len--;
    }

    ptr = tx_buf;
    len = exec_msg->cmd.size;
    spi_debug("SPI%d: TX DATA (%d)\n", cfg->spi.number, len);
    /* Write data */
    if (cfg->device.flag & RT_DEVICE_FLAG_STREAM)
    {
        if (*(ptr + len - 1) == '\n')
        {
            *(ptr + len - 1) = '\r';
            *(ptr + len++) = '\n';
            *(ptr + len) = 0;
        }
    }
    if ((cfg->spi.status & SPI_STATUS_START) && \
        (cfg->device.flag & RT_DEVICE_FLAG_DMA_TX) && (len > 8))
    {
        /* DMA mode */
        spi_debug("SPI%d: DMA TX DATA\n", cfg->spi.number);
        exec_msg->ret.ret = bsp_spi_dma_tx(&cfg->spi,
            (rt_uint32_t *)ptr,
            (rt_uint16_t)len);
        if (exec_msg->ret.ret != RT_EOK)
        {
            exec_msg->ret.size = 0;
            return;
        }
        exec_msg->ret.size = exec_msg->cmd.size;
    }
    else
    {
        /* polling mode */
        spi_debug("SPI%d: Polling TX DATA\n", cfg->spi.number);
        while (len)
        {
            while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
            cfg->spi.spi_device->DR = (rt_uint16_t)*(ptr++);
            len--;
        }

        exec_msg->ret.size = exec_msg->cmd.size - len;
        exec_msg->ret.ret = RT_EOK;
    }
}

static void spi_task_control(struct bsp_spi_unit_struct *cfg,
    union bsp_spi_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    switch (exec_msg->cmd.other)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* Suspend device */
        SPI_Cmd(cfg->spi.spi_device, DISABLE);
        cfg->device.flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* Resume device */
        SPI_Cmd(cfg->spi.spi_device, ENABLE);
        cfg->device.flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_MODE_BLOCKING:
        /* Blocking mode operation */
        cfg->spi.status &= ~(rt_uint16_t)SPI_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->spi.status |= (rt_uint16_t)SPI_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_SPI_OUTPUT_CLOCK:
        /* Set RX buffer */
        {
            rt_uint32_t len = (rt_uint32_t)exec_msg->cmd.ptr;

            /* Flushing RX */
            *(rt_uint16_t *)0x00 = cfg->spi.spi_device->DR;

            while (len)
            {
                /* dummy write */
                while (!(cfg->spi.spi_device->SR & SPI_I2S_FLAG_TXE));
                cfg->spi.spi_device->DR = 0x00ff;
                /* Flushing RX */
                *(rt_uint16_t *)0x00 = cfg->spi.spi_device->DR;
                len--;
            }
        }
        break;
    }

    exec_msg->ret.ret = RT_EOK;
}

/***************************************************************************//**
* @brief
*   Register SPI device
*
* @details
*
* @note
*
* @param[in] device
*   Pointer to device descriptor
*
* @param[in] name
*   Device name
*
* @param[in] flag
*   Configuration flags
*
* @param[in] spi
*   Pointer to SPI device descriptor
*
* @return
*   Error code
******************************************************************************/
void spi_task_main_loop(void *parameter)
{
    struct bsp_spi_unit_struct *cfg;
    union bsp_spi_exec_message *p_exec_msg;
    rt_thread_t self;
    rt_bool_t chk_block;

    cfg = (struct bsp_spi_unit_struct *)parameter;
    self = rt_thread_self();

    if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        SPI_RX_MESSAGE_SIZE,
        sizeof(cfg->task.rx_msg_pool),
        RT_IPC_FLAG_FIFO) != RT_EOK)
    {
        rt_kprintf("SPI%d: init mq failed!\n", cfg->spi.number);
        return;
    }

    if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("SPI%d: init event failed!\n", cfg->spi.number);
            return;
        }

    /* Enable SPI1 */
    SPI_Cmd(cfg->spi.spi_device, ENABLE);
    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;
    
    cfg->spi.status |= SPI_STATUS_START;
    spi_debug("SPI%d: enter main loop\n", cfg->spi.number);

SPI_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
        if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&p_exec_msg,
            SPI_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
        {
            continue;
        }

        chk_block = RT_FALSE;
        switch (p_exec_msg->cmd.cmd)
        {
        case SPI_COMMAND_STATUS:
            p_exec_msg->ret.other = (rt_uint32_t)cfg->spi.status;
            break;

        case SPI_COMMAND_OPEN:
            spi_task_open(cfg, p_exec_msg);
            break;

        case SPI_COMMAND_CLOSE:
            spi_task_close(cfg, p_exec_msg);
            break;

        case SPI_COMMAND_READ:
            spi_task_read(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case SPI_COMMAND_WRITE:
            spi_task_write(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case SPI_COMMAND_CONTROL:
            spi_task_control(cfg, p_exec_msg);
            break;

        default:
            break;
        }

        if (chk_block && (cfg->spi.status & SPI_STATUS_NONBLOCKING))
        {
            rt_free(p_exec_msg);
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, p_exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize SPI device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t bsp_spi_init (rt_device_t dev)
{
    struct bsp_spi_device *spi;

    spi = (struct bsp_spi_device *)(dev->user_data);

    return rt_thread_startup(&spi_tasks[spi->number - 1]->thread);
}

/***************************************************************************//**
 * @brief
 *   Open SPI device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] oflag
 *   Device open flag
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t bsp_spi_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct bsp_spi_device *spi;
    union bsp_spi_exec_message exec_msg;

    spi = (struct bsp_spi_device *)(dev->user_data);

    exec_msg.cmd.cmd = SPI_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;

    if (!(spi->status & SPI_STATUS_START) && \
    (spi->status & SPI_STATUS_DIRECT_EXE))
    {
        struct bsp_spi_unit_struct *cfg = RT_NULL;

        switch (spi->number)
        {
        case 1:
            cfg = &spi1;
            break;

#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            cfg = &spi2;
            break;
#endif

#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            cfg = &spi3;
            break;
#endif
        }

        spi_task_open(cfg, &exec_msg);

        return RT_EOK;
    }
    
    return spi_task_exec(spi->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *   Close SPI device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t bsp_spi_close(rt_device_t dev)
{
    struct bsp_spi_device *spi;
    union bsp_spi_exec_message exec_msg;

    spi = (struct bsp_spi_device *)(dev->user_data);

    if (!(spi->status & SPI_STATUS_START) && \
    (spi->status & SPI_STATUS_DIRECT_EXE))
    {
        struct bsp_spi_unit_struct *cfg = RT_NULL;

        switch (spi->number)
        {
        case 1:
            cfg = &spi1;
            break;

#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            cfg = &spi2;
            break;
#endif

#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            cfg = &spi3;
            break;
#endif
        }

        spi_task_close(cfg, &exec_msg);

        return RT_EOK;
    }

    exec_msg.cmd.cmd = SPI_COMMAND_CLOSE;
    return spi_task_exec(spi->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *  Read from SPI device
 *
 * @details
 *
 * @note
 *  Slave mode has not tested yet
 *
 * @param[in] dev
 *  Pointer to device descriptor
 *
 * @param[in] pos
 *  Offset
 *
 * @param[in] buffer
 *  Poniter to the buffer
 *
 * @param[in] size
 *  Buffer size in byte
 *
 * @return
 *  Number of read bytes
 ******************************************************************************/
static rt_size_t bsp_spi_read (
    rt_device_t     dev,
    rt_off_t        pos,
    void            *buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct bsp_spi_device *spi;
    union bsp_spi_exec_message exec_msg;
    rt_bool_t nonblock;

    spi = (struct bsp_spi_device *)(dev->user_data);

    if (!(spi->status & SPI_STATUS_START) && \
        (spi->status & SPI_STATUS_DIRECT_EXE))
    {
        struct bsp_spi_unit_struct *cfg = RT_NULL;

        switch (spi->number)
        {
        case 1:
            cfg = &spi1;
            break;

#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            cfg = &spi2;
            break;
#endif

#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            cfg = &spi3;
            break;
#endif
        }

        while(cfg->spi.status & SPI_STATUS_RX_BUSY);

        exec_msg.cmd.cmd = SPI_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        spi_task_read(cfg, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union bsp_spi_exec_message *p_exec_msg;
        
        exec_msg.cmd.cmd = SPI_COMMAND_STATUS;
        do
        {
            ret = spi_task_exec(spi->number, &exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & SPI_STATUS_RX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(SPI_COMMAND_WAIT_TIME);
                }
            }
            else
            {
                break;
            }
        } while (1);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & SPI_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union bsp_spi_exec_message));
            if (p_exec_msg == RT_NULL)
            {
                ret = RT_ENOMEM;
                break;
            }
        }
        else
        {
            nonblock = RT_FALSE;
            p_exec_msg = &exec_msg;
        }

        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&spi->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&spi->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = SPI_COMMAND_READ;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = spi_task_exec(spi->number, p_exec_msg, nonblock);
        {
            rt_sem_release(&spi->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&spi->lock);
    } while (0);

    if (ret != RT_EOK)
    {
        rt_set_errno(ret);
        return 0;
    }
    if (exec_msg.ret.ret != RT_EOK)
    {
        rt_set_errno(exec_msg.ret.ret);
    }
    return exec_msg.ret.size;
}

/***************************************************************************//**
 * @brief
 *   Write to SPI device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] pos
 *   Offset
 *
 * @param[in] buffer
 *   Poniter to the buffer
 *
 * @param[in] size
 *   Buffer size in byte
 *
 * @return
 *   Number of written bytes
 ******************************************************************************/
static rt_size_t bsp_spi_write (
    rt_device_t     dev,
    rt_off_t        pos,
    const void*     buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct bsp_spi_device *spi;
    union bsp_spi_exec_message exec_msg;
    rt_bool_t nonblock;

    spi = (struct bsp_spi_device *)(dev->user_data);

    if (!(spi->status & SPI_STATUS_START) && \
        (spi->status & SPI_STATUS_DIRECT_EXE))
    {
        struct bsp_spi_unit_struct *cfg = RT_NULL;

        switch (spi->number)
        {
        case 1:
            cfg = &spi1;
            break;

#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            cfg = &spi2;
            break;
#endif

#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            cfg = &spi3;
            break;
#endif
        }

        while(cfg->spi.status & SPI_STATUS_RX_BUSY);

        exec_msg.cmd.cmd = SPI_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        spi_task_write(cfg, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union bsp_spi_exec_message *p_exec_msg;
    
        exec_msg.cmd.cmd = SPI_COMMAND_STATUS;
        do
        {
            ret = spi_task_exec(spi->number, &exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & SPI_STATUS_TX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(SPI_COMMAND_WAIT_TIME);
                }
            }
            else
            {
                break;
            }
        } while (1);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & SPI_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union bsp_spi_exec_message));
            if (p_exec_msg == RT_NULL)
            {
                ret = RT_ENOMEM;
                break;
            }
        }
        else
        {
            nonblock = RT_FALSE;
            p_exec_msg = &exec_msg;
        }

        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&spi->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&spi->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = SPI_COMMAND_WRITE;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = spi_task_exec(spi->number, p_exec_msg, nonblock);
        {
            rt_sem_release(&spi->lock);
            break;
        }

        /* Unlock device */
        rt_sem_release(&spi->lock);
    } while (0);

    if (ret != RT_EOK)
    {
        rt_set_errno(ret);
        return 0;
    }
    if (exec_msg.ret.ret != RT_EOK)
    {
        rt_set_errno(exec_msg.ret.ret);
    }
    return exec_msg.ret.size;
}

/***************************************************************************//**
* @brief
*   Configure SPI device
*
* @details
*
* @note
*
* @param[in] dev
*   Pointer to device descriptor
*
* @param[in] cmd
*   IIC control command
*
* @param[in] args
*   Arguments
*
* @return
*   Error code
******************************************************************************/
static rt_err_t bsp_spi_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    rt_err_t ret;
    struct bsp_spi_device *spi;
    union bsp_spi_exec_message exec_msg;

    spi = (struct bsp_spi_device *)(dev->user_data);

    if (!(spi->status & SPI_STATUS_START) && \
    (spi->status & SPI_STATUS_DIRECT_EXE))
    {
        struct bsp_spi_unit_struct *cfg = RT_NULL;

        switch (spi->number)
        {
        case 1:
            cfg = &spi1;
            break;

#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            cfg = &spi2;
            break;
#endif

#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            cfg = &spi3;
            break;
#endif
        }

        spi_task_control(cfg, &exec_msg);

        return RT_EOK;
    }

    /* Lock device */
    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&spi->lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&spi->lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return 0;
    }

    exec_msg.cmd.cmd = SPI_COMMAND_CONTROL;
    exec_msg.cmd.ptr = (rt_uint8_t *)args;
    exec_msg.cmd.other = (rt_uint32_t)cmd;
    ret = spi_task_exec(spi->number, &exec_msg, RT_FALSE);

    /* Unlock device */
    rt_sem_release(&spi->lock);

    return ret;
}

/***************************************************************************//**
* @brief
*   Initialize the specified SPI unit
*
* @details
*
* @note
*
* @param[in] devicea
*   Pointer to device descriptor
*
* @param[in] unitNumber
*   Unit number
*
* @param[in] location
*   Pin location number
*
* @param[in] flag
*   Configuration flag
*
* @param[in] dmaChannel
*   DMA channel number for TX
*
* @param[in] console
*   Indicate if using as console
*
* @return
*   Pointer to SPI device
******************************************************************************/
static rt_err_t bsp_spi_unit_init(struct bsp_spi_unit_init *init)
{
    rt_device_t         device;
    struct bsp_spi_device *spi;
    struct bsp_spi_dma_mode *dma_tx, *dma_rx;
    rt_uint32_t         flag;
    GPIO_InitTypeDef    gpio_init;
    SPI_InitTypeDef     spi_init;
    DMA_InitTypeDef     dma_init;
    NVIC_InitTypeDef    nvic_init;
    GPIO_TypeDef        *port_spi, *port_cs;
    rt_uint16_t         pin_sck, pin_mosi, pin_miso, pin_cs;
    rt_uint32_t         tx_irq;
    rt_uint8_t          tx_chn;
    miniStm32_irq_hook_init_t hook;

    device = &(init->unit)->device;
    spi = &(init->unit)->spi;
    dma_tx = (struct bsp_spi_dma_mode *)spi->tx_mode;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        spi->counter    = 0;
        spi->number     = init->number;
        spi->status      = init->config & SPI_STATUS_MASK;
        spi->rx_mode    = RT_NULL;  // TODO: INT and DMA RX mode

        /* Init lock */
        if (rt_sem_init(&spi->lock, init->name, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Unit number specified setting */
        switch (init->number)
        {
        case 1:
            {
                spi->spi_device     = SPI1;
                /* Enable clock */
                /* Enable clock and Pre-config GPIO */
                if (SPI_CONFIG_REMAP_GET(init->config))
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | \
                        RCC_APB2Periph_SPI1 | RCC_APB2Periph_AFIO), ENABLE);
                    GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);
                    port_spi        = GPIOB;
                    pin_sck         = GPIO_Pin_3;
                    pin_mosi        = GPIO_Pin_5;
                    pin_miso        = GPIO_Pin_4;
                    port_cs         = GPIOA;
                    pin_cs          = GPIO_Pin_15;
                }
                else
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | \
                        RCC_APB2Periph_SPI1), ENABLE);
                    port_spi        = GPIOA;
                    pin_sck         = GPIO_Pin_5;
                    pin_mosi        = GPIO_Pin_7;
                    pin_miso        = GPIO_Pin_6;
                    port_cs         = GPIOA;
                    pin_cs          = GPIO_Pin_4;
                }
                /* Pre-config DMA */
                if (init->config & SPI_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
                    dma_tx->dma_chn = DMA1_Channel_SPI1_TX;
                    tx_irq          = DMA1_Channel_SPI1_TX_IRQn;
                    tx_chn          = DMA1_Channel_SPI1_TX_Num;
                }
            }
            break;
#if defined(RCC_APB1ENR_SPI2EN)
        case 2:
            {
                spi->spi_device     = SPI2;
                /* Enable clock */
                RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
                /* Pre-config GPIO */
                port_spi            = GPIOB;
                pin_sck             = GPIO_Pin_13;
                pin_mosi            = GPIO_Pin_15;
                pin_miso            = GPIO_Pin_14;
                port_cs             = GPIOB;
                pin_cs              = GPIO_Pin_12;
                /* Pre-config DMA */
                if (init->config & SPI_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
                    dma_tx->dma_chn = DMA1_Channel_SPI2_TX;
                    tx_irq          = DMA1_Channel_SPI2_TX_IRQn;
                    tx_chn          = DMA1_Channel_SPI2_TX_Num;
                }
            }
            break;
#endif
#if defined(RCC_APB1ENR_SPI3EN)
        case 3:
            {
                spi->spi_device     = SPI3;
                /* Enable clock and Pre-config GPIO */
                if (SPI_CONFIG_REMAP_GET(init->config))
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOC | \
                        RCC_APB2Periph_AFIO), ENABLE);
                    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
                    GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
                    port_spi        = GPIOC;
                    pin_sck         = GPIO_Pin_10;
                    pin_mosi        = GPIO_Pin_12;
                    pin_miso        = GPIO_Pin_11;
                    port_cs         = GPIOA;
                    pin_cs          = GPIO_Pin_4;
                }
                else
                {
                    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
                    port_spi        = GPIOB;
                    pin_sck         = GPIO_Pin_3;
                    pin_mosi        = GPIO_Pin_5;
                    pin_miso        = GPIO_Pin_4;
                    port_cs         = GPIOA;
                    pin_cs          = GPIO_Pin_15;
                }
                /* Pre-config DMA */
                if (init->config & SPI_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
                    dma_tx->dma_chn = DMA2_Channel_SPI3_TX;
                    tx_irq          = DMA2_Channel_SPI3_TX_IRQn;
                    tx_chn          = DMA2_Channel_SPI3_TX_Num;
                }
            }
            break;
#endif
        default:
            break;
        }

        /* Config GPIO */
        gpio_init.GPIO_Speed        = GPIO_Speed_50MHz;
        if (init->config & SPI_CONFIG_MASTER)
        {
            /* Master clock */
            gpio_init.GPIO_Pin      = pin_sck;
            gpio_init.GPIO_Mode     = GPIO_Mode_AF_PP;
            GPIO_Init(port_spi, &gpio_init);
            /* Master output */
            gpio_init.GPIO_Pin      = pin_mosi;
            gpio_init.GPIO_Mode     = GPIO_Mode_AF_PP;
            GPIO_Init(port_spi, &gpio_init);
            /* Master input */
            gpio_init.GPIO_Pin      = pin_miso;
            gpio_init.GPIO_Mode     = GPIO_Mode_IPU;    // TODO: Input floating / Input pull-up
            GPIO_Init(port_spi, &gpio_init);
            if (init->config & SPI_CONFIG_AUTOCS)
            {
                gpio_init.GPIO_Pin  = pin_cs;
                gpio_init.GPIO_Mode = GPIO_Mode_AF_PP;
                GPIO_Init(port_cs, &gpio_init);
            }
        }
        else
        {
            /* Slave clock */
            gpio_init.GPIO_Pin      = pin_sck;
            gpio_init.GPIO_Mode     = GPIO_Mode_IN_FLOATING;
            GPIO_Init(port_spi, &gpio_init);
            /* Slave input */
            gpio_init.GPIO_Pin      = pin_mosi;
            gpio_init.GPIO_Mode     = GPIO_Mode_IPU;    // TODO: Input floating / Input pull-up
            GPIO_Init(port_spi, &gpio_init);
            /* Slave output */
            gpio_init.GPIO_Pin      = pin_miso;
            gpio_init.GPIO_Mode     = GPIO_Mode_AF_PP;  // TODO: multi-slave -> open drain
            GPIO_Init(port_spi, &gpio_init);
        }

        /* Init SPI unit */
        spi_init.SPI_Direction      = SPI_Direction_2Lines_FullDuplex;
        if (init->config & SPI_CONFIG_MASTER)
        {
            spi_init.SPI_Mode       = SPI_Mode_Master;
        }
        else
        {
            spi_init.SPI_Mode       = SPI_Mode_Slave;
        }
        spi_init.SPI_DataSize       = SPI_DataSize_8b;
        switch (SPI_CONFIG_CLK_MODE_GET(init->config))
        {
        case 0: // Clock idle low, sample on rising edge
            spi_init.SPI_CPOL       = SPI_CPOL_Low;
            spi_init.SPI_CPHA       = SPI_CPHA_1Edge;
            break;
        case 1: // Clock idle low, sample on falling edge
            spi_init.SPI_CPOL       = SPI_CPOL_Low;
            spi_init.SPI_CPHA       = SPI_CPHA_2Edge;
            break;
        case 2: // Clock idle high, sample on falling edge
            spi_init.SPI_CPOL       = SPI_CPOL_High;
            spi_init.SPI_CPHA       = SPI_CPHA_1Edge;
            break;
        case 3: // Clock idle high, sample on rising edge
            spi_init.SPI_CPOL       = SPI_CPOL_High;
            spi_init.SPI_CPHA       = SPI_CPHA_2Edge;
            break;
        }
        if (init->config & SPI_CONFIG_AUTOCS)
        {
            spi_init.SPI_NSS        = SPI_NSS_Hard;
        }
        else
        {
            spi_init.SPI_NSS        = SPI_NSS_Soft;
        }
        spi_init.SPI_BaudRatePrescaler  = SPI_BaudRatePrescaler_64; // 1.125MHz
        spi_init.SPI_FirstBit       = SPI_FirstBit_MSB;
        spi_init.SPI_CRCPolynomial  = 7;
        SPI_Init(spi->spi_device, &spi_init);

        /* Config DMA TX */
        if (init->config & SPI_CONFIG_DMA_TX)
        {
            /* Init DMA TX queue */
            dma_tx->list_head = dma_tx->list_tail = RT_NULL;

            /* Init memory pool */
            if (rt_mp_init(&dma_tx->dma_mp,
                init->name,
                dma_tx->mem_pool,
                sizeof(dma_tx->mem_pool),
                sizeof(struct bsp_spi_dma_node)) != RT_EOK)
            {
                break;
            }

            /* Config DMA */
            DMA_DeInit(dma_tx->dma_chn);
            dma_init.DMA_PeripheralBaseAddr = SPI1_DR_Base;
            dma_init.DMA_MemoryBaseAddr     = (uint32_t)0x00;
            dma_init.DMA_DIR                = DMA_DIR_PeripheralDST;
            dma_init.DMA_BufferSize         = 1;
            dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
            dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
            dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
            dma_init.DMA_Mode               = DMA_Mode_Normal;
            dma_init.DMA_Priority           = DMA_Priority_Low;
            dma_init.DMA_M2M                = DMA_M2M_Disable;
            DMA_Init(dma_tx->dma_chn, &dma_init);

            /* Config hook */
            hook.type       = miniStm32_irq_type_dma;
            hook.unit       = tx_chn - 1;
            hook.cbFunc     = bsp_spi_dma_tx_isr;
            hook.userPtr    = device;
            miniStm32_irq_hook_register(&hook);

            /* Enable interrupt and NVIC */
            switch (init->number)
            {
            case 1:
                DMA_ClearFlag(DMA1_FLAG_GL3);
                break;
            case 2:
                DMA_ClearFlag(DMA1_FLAG_GL5);
                break;
            case 3:
                DMA_ClearFlag(DMA2_FLAG_GL2);
                break;
            }
            DMA_ITConfig(dma_tx->dma_chn, DMA_IT_TC | DMA_IT_TE, ENABLE);
            NVIC_ClearPendingIRQ(tx_irq);
            nvic_init.NVIC_IRQChannel = tx_irq;
            nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
            nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DMA;
            nvic_init.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&nvic_init);
        }
        else
        {
            dma_tx = RT_NULL;
        }

        /* Config INT RX? */
        if (!(init->config & SPI_CONFIG_MASTER))
        {
            // TODO: slave mode
/*            flag |= RT_DEVICE_FLAG_INT_RX;

            spi->rx_mode = rt_malloc(sizeof(struct bsp_spi_int_mode));
            if (spi->rx_mode == RT_NULL)
            {
                spi_debug("SPI%d err: no mem for INT RX\n", spi->unit);
                break;
            }

            hook.type           = miniStm32_irq_type_spi;
            hook.unit           = unitNumber - 1;
            hook.cbFunc         = bsp_spi_rx_isr;
            hook.userPtr        = device;
            miniStm32_irq_hook_register(&hook);
*/
        }

        /* Config and register device */
        device->type        = RT_Device_Class_SPIBUS;
        device->rx_indicate = RT_NULL;
        device->tx_complete = RT_NULL;
        device->init        = bsp_spi_init;
        device->open        = bsp_spi_open;
        device->close       = bsp_spi_close;
        device->read        = bsp_spi_read;
        device->write       = bsp_spi_write;
        device->control     = bsp_spi_control;
        device->user_data   = (void *)spi;

        if (init->config & SPI_CONFIG_DMA_TX)
        {
            flag |= RT_DEVICE_FLAG_DMA_TX;
        }
        return rt_device_register(device, init->name, flag);
    } while(0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all SPI module related hardware and register SPI device to kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t miniStm32_hw_spi_init(void)
{
    struct bsp_spi_unit_init init;

    do
    {
#if (defined(RCC_APB2ENR_SPI1EN) && defined(BSP_USING_SPI1))
        const rt_uint8_t name[] = SPI1_NAME;

        spi_tasks[0]        = &spi1.task;
 #if (SPI1_SPI_MODE & SPI_CONFIG_DMA_TX)
        spi1.spi.tx_mode    = (void *)&spi1_dma_tx;
 #endif

        init.number         = 1;
        init.config         = SPI1_SPI_MODE;
        init.name           = &name[0];
        init.unit           = &spi1;
        if (bsp_spi_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &spi1.task.thread,
            init.name,
            spi_task_main_loop,
            (void *)&spi1,
            (void *)&spi1.task.stack,
            sizeof(spi1.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

#if (defined(RCC_APB1ENR_SPI2EN) && defined(BSP_USING_SPI2))
        const rt_uint8_t name[] = SPI2_NAME;

        spi_tasks[1]        = &spi2.task;
 #if (SPI2_SPI_MODE & SPI_CONFIG_DMA_TX)
        spi2.spi.tx_mode    = (void *)&spi2_dma_tx;
 #endif

        init.number         = 2;
        init.config         = SPI2_SPI_MODE;
        init.name           = &name[0];
        init.unit           = &spi2;
        if (bsp_spi_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &spi2.task.thread,
            init.name,
            spi_task_main_loop,
            (void *)&spi2,
            (void *)&spi2.task.stack,
            sizeof(spi2.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

#if (defined(RCC_APB1ENR_SPI3EN) && defined(BSP_USING_SPI3))
        const rt_uint8_t name[] = SPI3_NAME;

        spi_tasks[2]        = &spi3.task;
 #if (SPI3_SPI_MODE & SPI_CONFIG_DMA_TX)
        spi3.spi.tx_mode    = (void *)&spi3_dma_tx;
 #endif

        init.number         = 3;
        init.config         = SPI3_SPI_MODE;
        init.name           = &name[0];
        init.unit           = &spi3;
        if (bsp_spi_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &spi3.task.thread,
            init.name,
            spi_task_main_loop,
            (void *)&spi3,
            (void *)&spi3.task.stack,
            sizeof(spi3.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

        spi_debug("SPI%d: H/W init OK!\n", init.number);
        return RT_EOK;
    } while (0);

    rt_kprintf("SPI%d err: H/W init failed!\n", init.number);
    return -RT_ERROR;
}

#endif /* (defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2) || defined(BSP_USING_SPI3)) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
