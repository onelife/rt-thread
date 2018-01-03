/***************************************************************************//**
 * @file    drv_usart.c
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
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_usart.h"
#if (defined(BSP_USING_USART1) || defined(BSP_USING_USART2) || defined(BSP_USING_USART3))
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USART1_DR_Base                      (USART1_BASE + 0x04)
#define USART2_DR_Base                      (USART2_BASE + 0x04)
#define USART3_DR_Base                      (USART3_BASE + 0x04)

#define DMA1_Channel_USART1_TX              (DMA1_Channel4)
#define DMA1_Channel_USART1_TX_Num          (4)
#define DMA1_Channel_USART1_TX_IRQn         (DMA1_Channel4_IRQn)
#define DMA1_Channel_USART1_RX              (DMA1_Channel5)
#define DMA1_Channel_USART1_RX_Num          (5)
#define DMA1_Channel_USART1_RX_IRQn         (DMA1_Channel5_IRQn)

#define DMA1_Channel_USART2_TX              (DMA1_Channel7)
#define DMA1_Channel_USART2_TX_Num          (7)
#define DMA1_Channel_USART2_TX_IRQn         (DMA1_Channel7_IRQn)
#define DMA1_Channel_USART2_RX              (DMA1_Channel6)
#define DMA1_Channel_USART2_RX_Num          (6)
#define DMA1_Channel_USART2_RX_IRQn         (DMA1_Channel6_IRQn)

#define DMA1_Channel_USART3_TX              (DMA1_Channel2)
#define DMA1_Channel_USART3_TX_Num          (2)
#define DMA1_Channel_USART3_TX_IRQn         (DMA1_Channel2_IRQn)
#define DMA1_Channel_USART3_RX              (DMA1_Channel3)
#define DMA1_Channel_USART3_RX_Num          (3)
#define DMA1_Channel_USART3_RX_IRQn         (DMA1_Channel3_IRQn)

/* Private macro -------------------------------------------------------------*/
#ifdef BSP_USART_DEBUG
#define usart_debug(format,args...)         rt_kprintf(format, ##args)
#else
#define usart_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct bsp_usart_task_struct *usart_tasks[3];

#if (defined(USART1) || defined(BSP_USING_USART1))
static struct bsp_usart_unit_struct usart1;
 #if (USART1_USART_MODE & USART_CONFIG_DMA_TX)
 static struct bsp_usart_dma_mode usart1_dma_tx;
 #endif
 #if (USART1_USART_MODE & USART_CONFIG_INT_RX)
 static struct bsp_usart_int_mode usart1_int_rx = {0};
 #endif
#endif

#if (defined(USART2) || defined(BSP_USING_USART2))
static struct bsp_usart_unit_struct usart2;
 #if (USART2_USART_MODE & USART_CONFIG_DMA_TX)
 static struct bsp_usart_dma_mode usart2_dma_tx;
 #endif
 #if (USART2_USART_MODE & USART_CONFIG_INT_RX)
 static struct bsp_usart_int_mode usart2_int_rx = {0};
 #endif
#endif

#if (defined(USART3) || defined(BSP_USING_USART3))
static struct bsp_usart_unit_struct usart3;
 #if (USART3_USART_MODE & USART_CONFIG_DMA_TX)
 static struct bsp_usart_dma_mode usart3_dma_tx;
 #endif
 #if (USART3_USART_MODE & USART_CONFIG_INT_RX)
 static struct bsp_usart_int_mode usart3_int_rx = {0};
 #endif
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *  DMA for USART TX interrupt handler
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_usart_dma_tx_isr(rt_device_t dev)
{
    struct bsp_usart_device *usart;
    struct bsp_usart_dma_mode *dma_tx;
	struct bsp_usart_dma_node *dma_node;
	rt_uint32_t level;

    usart = (struct bsp_usart_device *)dev->user_data;
    dma_tx = (struct bsp_usart_dma_mode *)usart->tx_mode;
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

    USART_DMACmd(usart->usart_device, USART_DMAReq_Tx, DISABLE);
    DMA_Cmd(dma_tx->dma_chn, DISABLE);

	if (dma_tx->list_head != RT_NULL)
	{
        /* Enable DMA TX */
        dma_tx->dma_chn->CMAR = (rt_uint32_t)dma_tx->list_head->data_ptr;
        dma_tx->dma_chn->CNDTR = (rt_uint32_t)dma_tx->list_head->data_size;
        DMA_Cmd(dma_tx->dma_chn, ENABLE);
        USART_DMACmd(usart->usart_device, USART_DMAReq_Tx, ENABLE);
	}
    else
    {
        /* Reset status */
        usart->status &= ~(rt_uint16_t)USART_STATUS_TX_BUSY;
    }
}

static rt_err_t bsp_usart_dma_tx(
    struct bsp_usart_device *usart,
    rt_uint32_t *buffer,
    rt_uint16_t size)
{
    struct bsp_usart_dma_mode *dma_tx;
    struct bsp_usart_dma_node *dma_node;
    rt_uint32_t level;

    dma_tx = (struct bsp_usart_dma_mode *)usart->tx_mode;

    /* Allocate a data node */
    dma_node = (struct bsp_usart_dma_node *)rt_mp_alloc(
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
        usart->status |= (rt_uint16_t)USART_STATUS_TX_BUSY;
        dma_tx->dma_chn->CMAR = (rt_uint32_t)buffer;
        dma_tx->dma_chn->CNDTR = (rt_uint32_t)size;
        DMA_Cmd(dma_tx->dma_chn, ENABLE);

        rt_hw_interrupt_enable(level);

        USART_DMACmd(usart->usart_device, USART_DMAReq_Tx, ENABLE);
    }
    else
    {
        rt_hw_interrupt_enable(level);
    }

    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *  USART RX data valid interrupt handler
 *
 * @details
 *
 * @note
 *  9-bit USART mode has not implemented yet and USART slave mode is untested
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void bsp_usart_int_rx_isr(rt_device_t dev)
{
    struct bsp_usart_device     *usart;
    struct bsp_usart_int_mode   *int_rx;

    /* interrupt mode receive */
    RT_ASSERT(dev->flag & RT_DEVICE_FLAG_INT_RX);

    usart = (struct bsp_usart_device *)(dev->user_data);
    int_rx = (struct bsp_usart_int_mode *)(usart->rx_mode);

    /* Set status */
    usart->status |= USART_STATUS_RX_BUSY;

    /* save into rx buffer */
    while (usart->usart_device->SR & USART_FLAG_RXNE)
    {
        rt_base_t level;

        /* disable interrupt */
        level = rt_hw_interrupt_disable();

        /* save character */
        int_rx->buffer[int_rx->save_index++] = \
            (rt_uint8_t)(usart->usart_device->DR & 0x00ff);
        if (int_rx->save_index >= sizeof(int_rx->buffer))
            int_rx->save_index = 0;

        /* if the next position is read index, overwrite */
        if (int_rx->save_index == int_rx->read_index)
        {
            if (++int_rx->read_index >= sizeof(int_rx->buffer))
            {
                int_rx->read_index = 0;
            }
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* Reset status */
    usart->status &= ~(rt_uint16_t)USART_STATUS_RX_BUSY;

    /* invoke callback */
    if (dev->rx_indicate != RT_NULL)
    {
        rt_size_t rx_len;

        /* get rx length */
        rx_len = int_rx->read_index > int_rx->save_index ?
            int_rx->read_index - int_rx->read_index + int_rx->save_index : \
            int_rx->save_index - int_rx->read_index;

        dev->rx_indicate(dev, rx_len);
    }
}

/***************************************************************************//**
 * @brief
 *   Open USART device
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
static rt_err_t usart_task_exec(rt_uint8_t number,
    union bsp_usart_exec_message *exec_msg,
    rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &usart_tasks[number - 1]->rx_msgs,
            (void *)&exec_msg,
            USART_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(USART_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        usart_debug("USART%d err: send cmd failed! (%x)\n", number, ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &usart_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &usart_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            usart_debug("USART%d err: receive event failed! (%x)\n", number, ret);
        }
    }

    return ret;
}

static void usart_task_open(struct bsp_usart_unit_struct *cfg,
   union bsp_usart_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_uint16_t oflag;

    oflag = (rt_uint16_t)exec_msg->cmd.other;
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
    {
        cfg->usart.status |= USART_STATUS_READ_ONLY;
    }
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
    {
        cfg->usart.status |= USART_STATUS_WRITE_ONLY;
    }
    if (oflag & (rt_uint16_t)RT_DEVICE_OFLAG_NONBLOCKING)
    {
        cfg->usart.status |= USART_STATUS_NONBLOCKING;
    }

    cfg->usart.counter++;

    usart_debug("USART%d: Open with flag %x\n", cfg->usart.number, oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void usart_task_close(struct bsp_usart_unit_struct *cfg,
    union bsp_usart_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    cfg->usart.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void usart_task_read(struct bsp_usart_unit_struct *cfg,
    union bsp_usart_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->usart.status & USART_STATUS_WRITE_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;
    len = exec_msg->cmd.size;
    ptr = exec_msg->cmd.ptr;
    if (cfg->device.flag & RT_DEVICE_FLAG_INT_RX)
    {
        usart_debug("USART%d: INT RX (%d)\n", cfg->usart.number, len);
        /* INT mode */
        while (len)
        {
            rt_base_t level;
            struct bsp_usart_int_mode *int_rx;

            int_rx = (struct bsp_usart_int_mode *)cfg->usart.rx_mode;

            /* disable interrupt */
            level = rt_hw_interrupt_disable();

            if (int_rx->read_index != int_rx->save_index)
            {
                /* read a byte */
                *ptr++ = int_rx->buffer[int_rx->read_index];
                len--;

                /* move to next position */
                int_rx->read_index++;
                if (int_rx->read_index >= sizeof(int_rx->buffer))
                {
                    int_rx->read_index = 0;
                }
            }
            else
            {
                /* enable interrupt */
                rt_hw_interrupt_enable(level);
                exec_msg->ret.size = (rt_uint32_t)ptr - (rt_uint32_t)exec_msg->cmd.ptr;
                exec_msg->ret.ret = -RT_EEMPTY;
                break;
            }

            /* enable interrupt */
            rt_hw_interrupt_enable(level);
            exec_msg->ret.size = (rt_uint32_t)ptr - (rt_uint32_t)exec_msg->cmd.ptr;
            exec_msg->ret.ret = RT_EOK;
        }
    }
    else
    {
        usart_debug("USART%d: Polling RX (%d)\n", cfg->usart.number, len);
        /* polling mode */
        while (len)
        {
			while (cfg->usart.usart_device->SR & USART_FLAG_RXNE)
			{
				*ptr++ = cfg->usart.usart_device->DR & 0x00FF;
			}
            len--;
        }

        exec_msg->ret.size = exec_msg->cmd.size;
        exec_msg->ret.ret = RT_EOK;
    }
}

static void usart_task_write(struct bsp_usart_unit_struct *cfg,
    union bsp_usart_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->usart.status & USART_STATUS_READ_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }

    pos = (rt_off_t)exec_msg->cmd.other;
    len = exec_msg->cmd.size;
    ptr = exec_msg->cmd.ptr;
    if (cfg->device.flag & RT_DEVICE_FLAG_STREAM)
    {
        if (*(ptr + len - 1) == '\n')
        {
            *(ptr + len - 1) = '\r';
            *(ptr + len++) = '\n';
            *(ptr + len) = 0;
        }
    }
    if ((cfg->usart.status & USART_STATUS_START) && \
        (cfg->device.flag & RT_DEVICE_FLAG_DMA_TX) && (len > 2))
    {
        /* DMA mode */
        usart_debug("USART%d: DMA TX (%d)\n", cfg->usart.number, len);
        exec_msg->ret.ret = bsp_usart_dma_tx(&cfg->usart,
            (rt_uint32_t *)ptr,
            (rt_uint16_t)len);
        if (exec_msg->ret.ret != RT_EOK)
        {
            exec_msg->ret.size = 0;
            return;
        }
    }
    else
    {
        /* polling mode */
        usart_debug("USART%d: Polling TX (%d)\n", cfg->usart.number, len);
        while (len)
        {
            while (!(cfg->usart.usart_device->SR & USART_FLAG_TXE));
            cfg->usart.usart_device->DR = (rt_uint16_t)*(ptr++);
            len--;
        }
    }

    exec_msg->ret.size = exec_msg->cmd.size - len;
    exec_msg->ret.ret = RT_EOK;
}

static void usart_task_control(struct bsp_usart_unit_struct *cfg,
    union bsp_usart_exec_message *exec_msg)
{
    RT_ASSERT(cfg != RT_NULL);

    switch (exec_msg->cmd.other)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* Suspend device */
        USART_Cmd(cfg->usart.usart_device, DISABLE);
        cfg->device.flag |= RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* Resume device */
        USART_Cmd(cfg->usart.usart_device, ENABLE);
        cfg->device.flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_MODE_BLOCKING:
        /* Blocking mode operation */
        cfg->usart.status &= ~(rt_uint16_t)USART_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->usart.status |= (rt_uint16_t)USART_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_USART_RX_BUFFER:
        /* Set RX buffer */
        {
            // TODO:
        }
        break;
    }

    exec_msg->ret.ret = RT_EOK;
}

/***************************************************************************//**
* @brief
*   Register USART device
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
* @param[in] usart
*   Pointer to USART device descriptor
*
* @return
*   Error code
******************************************************************************/
void usart_task_main_loop(void *parameter)
{
    struct bsp_usart_unit_struct *cfg;
    union bsp_usart_exec_message *p_exec_msg;
    rt_thread_t self;
    rt_bool_t chk_block;

    cfg = (struct bsp_usart_unit_struct *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        USART_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("USART%d: init mq failed!\n", cfg->usart.number);
        return;
	}

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("USART%d: init event failed!\n", cfg->usart.number);
            return;
        }

    /* Enable USART */
    USART_Cmd(cfg->usart.usart_device, ENABLE);
    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;

    cfg->usart.status |= USART_STATUS_START;
    usart_debug("USART%d: enter main loop\n", cfg->usart.number);

USART_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&p_exec_msg,
            USART_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        chk_block = RT_FALSE;
        switch (p_exec_msg->cmd.cmd)
        {
        case USART_COMMAND_STATUS:
            p_exec_msg->ret.other = (rt_uint32_t)cfg->usart.status;
            break;

        case USART_COMMAND_OPEN:
            usart_task_open(cfg, p_exec_msg);
            break;

        case USART_COMMAND_CLOSE:
            usart_task_close(cfg, p_exec_msg);
            break;

        case USART_COMMAND_READ:
            usart_task_read(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case USART_COMMAND_WRITE:
            usart_task_write(cfg, p_exec_msg);
            chk_block = RT_TRUE;
            break;

        case USART_COMMAND_CONTROL:
            usart_task_control(cfg, p_exec_msg);
            break;

        default:
            break;
        }

        if (chk_block && (cfg->usart.status & USART_STATUS_NONBLOCKING))
        {
            rt_free(p_exec_msg);
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, p_exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize USART device
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
static rt_err_t bsp_usart_init (rt_device_t dev)
{
    struct bsp_usart_device *usart;

    usart = (struct bsp_usart_device *)(dev->user_data);

    return rt_thread_startup(&usart_tasks[usart->number - 1]->thread);
}

/***************************************************************************//**
 * @brief
 *   Open USART device
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
static rt_err_t bsp_usart_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct bsp_usart_device *usart;
    union bsp_usart_exec_message exec_msg;

    usart = (struct bsp_usart_device *)(dev->user_data);

    exec_msg.cmd.cmd = USART_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;

    if (!(usart->status & USART_STATUS_START) && \
        (usart->status & USART_STATUS_DIRECT_EXE))
    {
        struct bsp_usart_unit_struct *cfg = RT_NULL;

        switch (usart->number)
        {
        case 1:
            cfg = &usart1;
            break;

        case 2:
            cfg = &usart2;
            break;

        case 3:
            cfg = &usart3;
            break;
        }

        usart_task_open(cfg, &exec_msg);

        return RT_EOK;
    }

    return usart_task_exec(usart->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *   Close USART device
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
static rt_err_t bsp_usart_close(rt_device_t dev)
{
    struct bsp_usart_device *usart;
    union bsp_usart_exec_message exec_msg;

    usart = (struct bsp_usart_device *)(dev->user_data);

    if (!(usart->status & USART_STATUS_START) && \
        (usart->status & USART_STATUS_DIRECT_EXE))
    {
        struct bsp_usart_unit_struct *cfg = RT_NULL;

        switch (usart->number)
        {
        case 1:
            cfg = &usart1;
            break;

        case 2:
            cfg = &usart2;
            break;

        case 3:
            cfg = &usart3;
            break;
        }

        usart_task_close(cfg, &exec_msg);

        return RT_EOK;
    }

    exec_msg.cmd.cmd = USART_COMMAND_CLOSE;
    return usart_task_exec(usart->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *  Read from USART device
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
static rt_size_t bsp_usart_read (
    rt_device_t     dev,
    rt_off_t        pos,
    void            *buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct bsp_usart_device *usart;
    union bsp_usart_exec_message exec_msg;
    rt_bool_t nonblock;

    usart = (struct bsp_usart_device *)(dev->user_data);

    if (!(usart->status & USART_STATUS_START) && \
        (usart->status & USART_STATUS_DIRECT_EXE))
    {
        struct bsp_usart_unit_struct *cfg = RT_NULL;

        switch (usart->number)
        {
        case 1:
            cfg = &usart1;
            break;

        case 2:
            cfg = &usart2;
            break;

        case 3:
            cfg = &usart3;
            break;
        }

        while(cfg->usart.status & USART_STATUS_RX_BUSY);

        exec_msg.cmd.cmd = USART_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        usart_task_read(cfg, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union bsp_usart_exec_message *p_exec_msg;
        
        exec_msg.cmd.cmd = USART_COMMAND_STATUS;
        do
        {
            ret = usart_task_exec(usart->number, &exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & USART_STATUS_RX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(USART_COMMAND_WAIT_TIME);
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

        if (exec_msg.ret.other & USART_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union bsp_usart_exec_message));
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
            ret = rt_sem_take(&usart->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&usart->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = USART_COMMAND_READ;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = usart_task_exec(usart->number, p_exec_msg, nonblock);
        {
            rt_sem_release(&usart->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&usart->lock);
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
 *   Write to USART device
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
static rt_size_t bsp_usart_write (
    rt_device_t     dev,
    rt_off_t        pos,
    const void*     buffer,
    rt_size_t       size)

{
    rt_err_t    ret;
    struct bsp_usart_device *usart;
    union bsp_usart_exec_message exec_msg;
    rt_bool_t nonblock;

    usart = (struct bsp_usart_device *)(dev->user_data);

    if (!(usart->status & USART_STATUS_START) && \
        (usart->status & USART_STATUS_DIRECT_EXE))
    {
        struct bsp_usart_unit_struct *cfg = RT_NULL;

        switch (usart->number)
        {
        case 1:
            cfg = &usart1;
            break;

        case 2:
            cfg = &usart2;
            break;

        case 3:
            cfg = &usart3;
            break;
        }

        while(cfg->usart.status & USART_STATUS_TX_BUSY);

        exec_msg.cmd.cmd = USART_COMMAND_WRITE;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        usart_task_write(cfg, &exec_msg);

        return RT_EOK;
    }

    do
    {
        union bsp_usart_exec_message *p_exec_msg;

        exec_msg.cmd.cmd = USART_COMMAND_STATUS;
        do
        {
            ret = usart_task_exec(usart->number, &exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & USART_STATUS_TX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(USART_COMMAND_WAIT_TIME);
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

        if (exec_msg.ret.other & USART_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
            p_exec_msg = rt_malloc(sizeof(union bsp_usart_exec_message));
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
            ret = rt_sem_take(&usart->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&usart->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        p_exec_msg->cmd.cmd = USART_COMMAND_WRITE;
        p_exec_msg->cmd.size = size;
        p_exec_msg->cmd.ptr = (rt_uint8_t *)buffer;
        p_exec_msg->cmd.other = (rt_uint32_t)pos;
        ret = usart_task_exec(usart->number, p_exec_msg, nonblock);
        {
            rt_sem_release(&usart->lock);
            break;
        }

        /* Unlock device */
        rt_sem_release(&usart->lock);
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
*   Configure USART device
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
static rt_err_t bsp_usart_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    rt_err_t ret;
    struct bsp_usart_device *usart;
    union bsp_usart_exec_message exec_msg;

    usart = (struct bsp_usart_device *)(dev->user_data);

    if (!(usart->status & USART_STATUS_START) && \
        (usart->status & USART_STATUS_DIRECT_EXE))
    {
        struct bsp_usart_unit_struct *cfg = RT_NULL;

        switch (usart->number)
        {
        case 1:
            cfg = &usart1;
            break;

        case 2:
            cfg = &usart2;
            break;

        case 3:
            cfg = &usart3;
            break;
        }

        usart_task_control(cfg, &exec_msg);

        return RT_EOK;
    }

    /* Lock device */
    if (rt_hw_interrupt_check())
    {
        ret = rt_sem_take(&usart->lock, RT_WAITING_NO);
    }
    else
    {
        ret = rt_sem_take(&usart->lock, RT_WAITING_FOREVER);
    }
    if (ret != RT_EOK)
    {
        return 0;
    }

    exec_msg.cmd.cmd = USART_COMMAND_CONTROL;
    exec_msg.cmd.ptr = (rt_uint8_t *)args;
    exec_msg.cmd.other = (rt_uint32_t)cmd;
    ret = usart_task_exec(usart->number, &exec_msg, RT_FALSE);

    /* Unlock device */
    rt_sem_release(&usart->lock);

    return ret;
}

/***************************************************************************//**
* @brief
*   Initialize the specified USART unit
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
*   Pointer to USART device
******************************************************************************/
static rt_err_t bsp_usart_unit_init(struct bsp_usart_unit_init *init)
{
    rt_device_t             device;
    struct bsp_usart_device *usart;
    struct bsp_usart_dma_mode *dma_tx;
    struct bsp_usart_int_mode *int_rx;
    rt_uint32_t             flag;
    GPIO_InitTypeDef        gpio_init;
    USART_InitTypeDef       usart_init;
    DMA_InitTypeDef         dma_init;
    NVIC_InitTypeDef        nvic_init;
    GPIO_TypeDef            *port_usart;
    rt_uint16_t             pin_tx, pin_rx;
    rt_uint32_t             tx_irq, rx_irq;
    rt_uint8_t              tx_chn;
    miniStm32_irq_hook_init_t hook;

    device = &(init->unit)->device;
    usart = &(init->unit)->usart;
    dma_tx = (struct bsp_usart_dma_mode *)usart->tx_mode;
    int_rx = (struct bsp_usart_int_mode *)usart->rx_mode;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        usart->counter    = 0;
        usart->number     = init->number;
        usart->status     = init->config & USART_STATUS_MASK;

        /* Unit number specified setting */
        switch (init->number)
        {
        case 1:
            {
                usart->usart_device = USART1;
                /* Enable clock and Pre-config GPIO */
                if (USART_CONFIG_REMAP_GET(init->config))
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOB | \
                        RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO), ENABLE);
                    GPIO_PinRemapConfig(GPIO_Remap_USART1, ENABLE);
                    port_usart      = GPIOB;
                    pin_tx          = GPIO_Pin_6;
                    pin_rx          = GPIO_Pin_7;
                }
                else
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOA | \
                        RCC_APB2Periph_USART1), ENABLE);
                    port_usart      = GPIOA;
                    pin_tx          = GPIO_Pin_9;
                    pin_rx          = GPIO_Pin_10;
                }
                /* Pre-config DMA */
                if (init->config & USART_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
                    dma_tx->dma_chn = DMA1_Channel_USART1_TX;
                    tx_irq          = DMA1_Channel_USART1_TX_IRQn;
                    tx_chn          = DMA1_Channel_USART1_TX_Num;
                }
                if (init->config & USART_CONFIG_INT_RX)
                {
                    rx_irq          = USART1_IRQn;
                }
            }
            break;
#if defined(USART2)
        case 2:
            {
                usart->usart_device = USART2;
                /* Enable clock and Pre-config GPIO */
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
                if (USART_CONFIG_REMAP_GET(init->config))
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOD | \
                        RCC_APB2Periph_AFIO), ENABLE);
                    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);
                    port_usart      = GPIOD;
                    pin_tx          = GPIO_Pin_5;
                    pin_rx          = GPIO_Pin_6;
                }
                else
                {
                    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
                    port_usart      = GPIOA;
                    pin_tx          = GPIO_Pin_2;
                    pin_rx          = GPIO_Pin_3;
                }
                /* Pre-config DMA */
                if (init->config & USART_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
                    dma_tx->dma_chn = DMA1_Channel_USART2_TX;
                    tx_irq          = DMA1_Channel_USART2_TX_IRQn;
                    tx_chn          = DMA1_Channel_USART2_TX_Num;
                }
                if (init->config & USART_CONFIG_INT_RX)
                {
                    rx_irq          = USART2_IRQn;
                }
            }
            break;
#endif
#if defined(USART3)
        case 3:
            {
                usart->usart_device = USART3;
                RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
                /* Enable clock and Pre-config GPIO */
                if (USART_CONFIG_REMAP_GET(init->config) == 0x03)
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOD | \
                        RCC_APB2Periph_AFIO), ENABLE);
                    GPIO_PinRemapConfig(GPIO_FullRemap_USART3, ENABLE);
                    port_usart      = GPIOD;
                    pin_tx          = GPIO_Pin_8;
                    pin_rx          = GPIO_Pin_9;
                }
                else if (USART_CONFIG_REMAP_GET(init->config) == 0x01)
                {
                    RCC_APB2PeriphClockCmd((RCC_APB2Periph_GPIOC | \
                        RCC_APB2Periph_AFIO), ENABLE);
                    GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);
                    port_usart      = GPIOC;
                    pin_tx          = GPIO_Pin_10;
                    pin_rx          = GPIO_Pin_11;
                }
                else
                {
                    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
                    port_usart      = GPIOB;
                    pin_tx          = GPIO_Pin_10;
                    pin_rx          = GPIO_Pin_11;
                }
                /* Pre-config DMA */
                if (init->config & USART_CONFIG_DMA_TX)
                {
                    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
                    dma_tx->dma_chn = DMA1_Channel_USART3_TX;
                    tx_irq          = DMA1_Channel_USART3_TX_IRQn;
                    tx_chn          = DMA1_Channel_USART3_TX_Num;
                }
                /* Pre-config interrupt */
                if (init->config & USART_CONFIG_INT_RX)
                {
                    rx_irq          = USART3_IRQn;
                }
            }
            break;
#endif
        default:
            break;
        }

        /* Config GPIO */
        gpio_init.GPIO_Pin      = pin_tx;
        gpio_init.GPIO_Speed    = GPIO_Speed_50MHz;
        gpio_init.GPIO_Mode     = GPIO_Mode_AF_PP;
        GPIO_Init(port_usart, &gpio_init);

        gpio_init.GPIO_Pin      = pin_rx;
        gpio_init.GPIO_Mode     = GPIO_Mode_IN_FLOATING;
        GPIO_Init(port_usart, &gpio_init);

        /* Init USART unit */
        usart_init.USART_BaudRate   = init->frequency;
        if (init->config & USART_CONFIG_9BIT)
        {
            usart_init.USART_WordLength = USART_WordLength_9b;
        }
        else
        {
            usart_init.USART_WordLength = USART_WordLength_8b;
        }
        usart_init.USART_StopBits   = USART_StopBits_1;
        usart_init.USART_Parity     = USART_Parity_No;
        usart_init.USART_Mode       = USART_Mode_Rx | USART_Mode_Tx;
        usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(usart->usart_device, &usart_init);

        /* Config DMA TX */
        if (init->config & USART_CONFIG_DMA_TX)
        {
            /* Init DMA TX queue */
            dma_tx->list_head = dma_tx->list_tail = RT_NULL;

            /* Init memory pool */
            if (rt_mp_init(&dma_tx->dma_mp,
                init->name,
                dma_tx->mem_pool,
                sizeof(dma_tx->mem_pool),
                sizeof(struct bsp_usart_dma_node)) != RT_EOK)
            {
                break;
            }

            /* Config DMA */
            DMA_DeInit(dma_tx->dma_chn);
            dma_init.DMA_PeripheralBaseAddr = USART1_DR_Base;
            dma_init.DMA_MemoryBaseAddr     = (uint32_t)0x00;
            dma_init.DMA_DIR                = DMA_DIR_PeripheralDST;
            dma_init.DMA_BufferSize         = 1;
            dma_init.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
            dma_init.DMA_MemoryInc          = DMA_MemoryInc_Enable;
            dma_init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            dma_init.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
            dma_init.DMA_Mode               = DMA_Mode_Normal;
            dma_init.DMA_Priority           = DMA_Priority_Medium;
            dma_init.DMA_M2M                = DMA_M2M_Disable;
            DMA_Init(dma_tx->dma_chn, &dma_init);

            /* Config hook */
            hook.type       = miniStm32_irq_type_dma;
            hook.unit       = tx_chn - 1;
            hook.cbFunc     = bsp_usart_dma_tx_isr;
            hook.userPtr    = device;
            miniStm32_irq_hook_register(&hook);

            /* Enable interrupt and NVIC */
            switch (init->number)
            {
            case 1:
                DMA_ClearFlag(DMA1_FLAG_GL4);
                break;
            case 2:
                DMA_ClearFlag(DMA1_FLAG_GL7);
                break;
            case 3:
                DMA_ClearFlag(DMA1_FLAG_GL2);
                break;
            }
            DMA_ITConfig(dma_tx->dma_chn, DMA_IT_TC | DMA_IT_TE, ENABLE);
            NVIC_ClearPendingIRQ(tx_irq);
            nvic_init.NVIC_IRQChannel = tx_irq;
            nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
            nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_DMA;
            nvic_init.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&nvic_init);
        }

        /* Config INT RX */
        if (init->config & USART_CONFIG_INT_RX)
        {
            /* Init INT RX structure */
            int_rx->read_index = 0;
            int_rx->save_index = 0;

            /* Config hook */
            hook.type       = miniStm32_irq_type_usart;
            hook.unit       = init->number - 1;
            hook.cbFunc     = bsp_usart_int_rx_isr;
            hook.userPtr    = device;
            miniStm32_irq_hook_register(&hook);

            /* Enable interrupt and NVIC */
            USART_ClearFlag(usart->usart_device, USART_FLAG_RXNE);
            USART_ITConfig(usart->usart_device, USART_IT_RXNE, ENABLE);
            NVIC_ClearPendingIRQ(rx_irq);
            nvic_init.NVIC_IRQChannel = rx_irq;
            nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
            nvic_init.NVIC_IRQChannelSubPriority = BSP_IRQ_PRI_COM;
            nvic_init.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&nvic_init);
        }

        /* Enable USART */
        USART_Cmd(usart->usart_device, ENABLE);

        /* Init lock */
        if (rt_sem_init(&usart->lock, init->name, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Config and register device */
        device->type        = RT_Device_Class_Char;
        device->rx_indicate = RT_NULL;
        device->tx_complete = RT_NULL;
        device->init        = bsp_usart_init;
        device->open        = bsp_usart_open;
        device->close       = bsp_usart_close;
        device->read        = bsp_usart_read;
        device->write       = bsp_usart_write;
        device->control     = bsp_usart_control;
        device->user_data   = (void *)usart;

        if (init->config & USART_CONFIG_CONSOLE)
        {
            flag |= RT_DEVICE_FLAG_STREAM;
        }
        if (init->config & USART_CONFIG_DMA_TX)
        {
            flag |= RT_DEVICE_FLAG_DMA_TX;
        }
        if (init->config & USART_CONFIG_INT_RX)
        {
            flag |= RT_DEVICE_FLAG_INT_RX;
        }
        return rt_device_register(device, init->name, flag);
    } while(0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all USART module related hardware and register USART device to
*   kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t bsp_hw_usart_init(void)
{
    struct bsp_usart_unit_init init;

    do
    {
#if (defined(USART1) && defined(BSP_USING_USART1))
        const rt_uint8_t name1[] = USART1_NAME;

        usart_tasks[0]          = &usart1.task;
 #if (USART1_USART_MODE & USART_CONFIG_DMA_TX)
        usart1.usart.tx_mode    = (void *)&usart1_dma_tx;
 #endif
 #if (USART1_USART_MODE & USART_CONFIG_INT_RX)
        usart1.usart.rx_mode    = (void *)&usart1_int_rx;
 #endif

        init.number             = 1;
        init.config             = USART1_USART_MODE;
        init.frequency          = USART1_FREQUENCY;
        init.name               = &name1[0];
        init.unit               = &usart1;
        if (bsp_usart_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &usart1.task.thread,
            init.name,
            usart_task_main_loop,
            (void *)&usart1,
            (void *)&usart1.task.stack,
            sizeof(usart1.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

#if (defined(USART2) && defined(BSP_USING_USART2))
        const rt_uint8_t name2[] = USART2_NAME;

        usart_tasks[1]          = &usart2.task;
 #if (USART2_USART_MODE & USART_CONFIG_DMA_TX)
        usart2.usart.tx_mode    = (void *)&usart2_dma_tx;
  #endif
#if (USART2_USART_MODE & USART_CONFIG_INT_RX)
        usart2.usart.rx_mode    = (void *)&usart2_int_rx;
 #endif

        init.number             = 2;
        init.config             = USART2_USART_MODE;
        init.frequency          = USART2_FREQUENCY;
        init.name               = &name2[0];
        init.unit               = &usart2;
        if (bsp_usart_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &usart2.task.thread,
            init.name,
            usart_task_main_loop,
            (void *)&usart2,
            (void *)&usart2.task.stack,
            sizeof(usart2.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

#if (defined(USART3) && defined(BSP_USING_USART3))
        const rt_uint8_t name3[] = USART3_NAME;

        usart_tasks[2]          = &usart3.task;
 #if (USART3_USART_MODE & USART_CONFIG_DMA_TX)
        usart3.usart.tx_mode    = (void *)&usart3_dma_tx;
 #endif
 #if (USART3_USART_MODE & USART_CONFIG_INT_RX)
        usart3.usart.rx_mode    = (void *)&usart3_int_rx;
 #endif

        init.number             = 3;
        init.config             = USART3_USART_MODE;
        init.frequency          = USART3_FREQUENCY;
        init.name               = &name3[0];
        init.unit               = &usart3;
        if (bsp_usart_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &usart3.task.thread,
            init.name,
            usart_task_main_loop,
            (void *)&usart3,
            (void *)&usart3.task.stack,
            sizeof(usart3.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

        usart_debug("USART%d: H/W init OK!\n", init.number);
        return RT_EOK;
    } while (0);

    rt_kprintf("USART%d err: H/W init failed!\n", init.number);
    return -RT_ERROR;
}
// INIT_BOARD_EXPORT(bsp_hw_usart_init);

#endif /* (defined(BSP_USING_USART1) || defined(BSP_USING_USART2) || defined(BSP_USING_USART3)) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
