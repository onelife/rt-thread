/***************************************************************************//**
 * @file    drv_gpio_sccb.c
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

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_gpio_sccb.h"
#if defined(MINISTM32_USING_GPIO_SCCB)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_GPIO_SCCB_DEBUG
#define gpio_sccb_debug(format,args...)     rt_kprintf(format, ##args)
#else
#define gpio_sccb_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct miniStm32_gpio_sccb_task_struct *gpio_sccb_tasks[1];

#if GPIO_SCCB_FREQUENCY > 1000000
#error "GPIO SCCB clock frequency must be less than or equal to 1 MHz"
#endif
static struct miniStm32_gpio_sccb_unit_struct gpio_sccb;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
rt_inline void gpio_sccb_delayNs(rt_uint32_t ns)
{
    rt_uint32_t i;

    for(i = 0; i < (SystemCoreClock / 1000000 * ns / 1000); i++)
    {
        __NOP();
    }
}

rt_inline rt_bool_t gpio_sccb_toggleSck(GPIO_TypeDef *port, rt_uint16_t pin, rt_uint32_t us)
{
    rt_bool_t state;

    if ((uint16_t)port->ODR & pin)
    {
        port->BRR = pin;
        state = RT_FALSE;
    }
    else
    {
        port->BSRR = pin;
        state = RT_TRUE;
    }

    gpio_sccb_delayNs(us);
    return state;
}

rt_inline void gpio_sccb_start(
    struct miniStm32_gpio_sccb_unit_struct *cfg)
{
    /* SDA falling */
    cfg->sccb.sccb_device.sda_port->BRR = cfg->sccb.sccb_device.sda_pin;
    /* SCL falling */
    gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
        cfg->sccb.sccb_device.scl_pin,
        cfg->sccb.delay);
}

rt_inline void gpio_sccb_end(
    struct miniStm32_gpio_sccb_unit_struct *cfg)
{
    /* SDA low */
    cfg->sccb.sccb_device.sda_port->BRR = cfg->sccb.sccb_device.sda_pin;
    /* SCL rising */
    gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
        cfg->sccb.sccb_device.scl_pin,
        cfg->sccb.delay);
    /* SDA rising */
    cfg->sccb.sccb_device.sda_port->BSRR = cfg->sccb.sccb_device.sda_pin;

    gpio_sccb_delayNs(cfg->sccb.delay);
}

rt_inline void gpio_sccb_writeByte(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
    rt_uint16_t data)
{
    rt_uint8_t j;

    for (j = 0; j < 9; j++)
    {
        if (data & 0x0100)
        {
            cfg->sccb.sccb_device.sda_port->BSRR = cfg->sccb.sccb_device.sda_pin;
        }
        else
        {
            cfg->sccb.sccb_device.sda_port->BRR = cfg->sccb.sccb_device.sda_pin;
        }
        data <<= 1;

        gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
            cfg->sccb.sccb_device.scl_pin,
            cfg->sccb.delay);
        gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
            cfg->sccb.sccb_device.scl_pin,
            cfg->sccb.delay);
    }
}

rt_inline rt_uint8_t gpio_sccb_readByte(
    struct miniStm32_gpio_sccb_unit_struct *cfg)
{
    rt_uint8_t j, shift, data;
    rt_uint16_t temp2;
    rt_uint32_t temp3;

    shift = 0;
    temp2 = cfg->sccb.sccb_device.sda_pin;
    while(temp2 > 1)
    {
        shift += 4;
        temp2 >>= 1;
    }

    if (shift > 28)
    {
        temp3 = cfg->sccb.sccb_device.sda_port->CRH;
        temp3 &= ~((rt_uint32_t)0x0F << (shift - 32));
        temp3 |= (rt_uint32_t)0x08 << (shift - 32);
        cfg->sccb.sccb_device.sda_port->CRH = temp3;
    }
    else
    {
        temp3 = cfg->sccb.sccb_device.sda_port->CRL;
        temp3 &= ~((rt_uint32_t)0x0F << shift);
        temp3 |= (rt_uint32_t)0x08 << shift;
        cfg->sccb.sccb_device.sda_port->CRL = temp3;
    }
    cfg->sccb.sccb_device.sda_port->BSRR = cfg->sccb.sccb_device.sda_pin;

    data = 0;
    for (j = 0; j < 8; j++)
    {
        data <<= 1;

        gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
            cfg->sccb.sccb_device.scl_pin,
            cfg->sccb.delay);
        if (cfg->sccb.sccb_device.sda_port->IDR & cfg->sccb.sccb_device.sda_pin)
        {
            data |= 0x01;
        }
        gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
            cfg->sccb.sccb_device.scl_pin,
            cfg->sccb.delay);
    }

    if (shift > 28)
    {
        temp3 = cfg->sccb.sccb_device.sda_port->CRH;
        temp3 &= ~((rt_uint32_t)0x0F << (shift - 32));
        temp3 |= (rt_uint32_t)0x03 << (shift - 32);
        cfg->sccb.sccb_device.sda_port->CRH = temp3;
    }
    else
    {
        temp3 = cfg->sccb.sccb_device.sda_port->CRL;
        temp3 &= ~((rt_uint32_t)0x0F << shift);
        temp3 |= (rt_uint32_t)0x01 << shift;
        cfg->sccb.sccb_device.sda_port->CRL = temp3;
    }
    /* 9-bit: 1, master read */
    cfg->sccb.sccb_device.sda_port->BSRR = cfg->sccb.sccb_device.sda_pin;
    gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
        cfg->sccb.sccb_device.scl_pin,
        cfg->sccb.delay);
    gpio_sccb_toggleSck(cfg->sccb.sccb_device.scl_port,
        cfg->sccb.sccb_device.scl_pin,
        cfg->sccb.delay);

    return data;
}

/***************************************************************************//**
 * @brief
 *   Open SCCB device
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
static rt_err_t gpio_sccb_task_exec(rt_uint8_t number,
    union miniStm32_gpio_sccb_exec_message *exec_msg,
    rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &gpio_sccb_tasks[number - 1]->rx_msgs,
            (void *)&exec_msg,
            GPIO_SCCB_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(GPIO_SCCB_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        gpio_sccb_debug("IO_SCCB%d err: send cmd failed! (%x)\n", number, ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &gpio_sccb_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &gpio_sccb_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            gpio_sccb_debug("IO_SCCB%d err: receive event failed!\n", number);
        }
    }

    return ret;
}

static void gpio_sccb_task_open(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
   union miniStm32_gpio_sccb_exec_message *exec_msg)
{
    rt_uint16_t oflag;

    oflag = (rt_uint16_t)exec_msg->cmd.other;
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
    {
        cfg->sccb.status |= GPIO_SCCB_STATUS_READ_ONLY;
    }
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
    {
        cfg->sccb.status |= GPIO_SCCB_STATUS_WRITE_ONLY;
    }
    if (oflag & RT_DEVICE_OFLAG_NONBLOCKING)
    {
        cfg->sccb.status |= GPIO_SCCB_STATUS_NONBLOCKING;
    }

    cfg->sccb.counter++;

    gpio_sccb_debug("IO_SCCB%d: Open with flag %x\n", cfg->sccb.number, oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void gpio_sccb_task_close(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
    union miniStm32_gpio_sccb_exec_message *exec_msg)
{
    cfg->sccb.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void gpio_sccb_task_read(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
    union miniStm32_gpio_sccb_exec_message *exec_msg)
{
    rt_uint8_t *data = *((rt_uint8_t **)(exec_msg->cmd.ptr + 1));
    rt_uint16_t temp;

    /* Start */
    gpio_sccb_start(cfg);
    /* Phase 1: ID */
    /* 8-bit: 0, write; 9-bit: x */
    temp = (exec_msg->cmd.other & 0x0000007F) << 2;
    gpio_sccb_writeByte(cfg, temp);
    /* Phase 2, register address */
    /* 9-bit: x */
    temp = (rt_uint16_t)(*(exec_msg->cmd.ptr)) << 1;
    gpio_sccb_writeByte(cfg, temp);
    /* End */
    gpio_sccb_end(cfg);

    /* Start */
    gpio_sccb_start(cfg);
    /* Phase 1, ID */
    /* 8-bit: 1, read; 9-bit: x */
    temp = ((exec_msg->cmd.other & 0x0000007F) << 2) | 0x0002;
    gpio_sccb_writeByte(cfg, temp);
    /* Phase 2, read data */
    *data = gpio_sccb_readByte(cfg);
    /* End */
    gpio_sccb_end(cfg);

    gpio_sccb_debug("cfg->sccb.delay %x\n", cfg->sccb.delay);

    gpio_sccb_debug("IO_SCCB%d: read %x %x, %x %x\n",
        cfg->sccb.number,
        (exec_msg->cmd.other & 0x0000007F) << 1,
        *(exec_msg->cmd.ptr),
        (((exec_msg->cmd.other & 0x0000007F) << 2) | 0x0002) >> 1,
        *data);
    exec_msg->ret.ret = RT_EOK;
}

static void gpio_sccb_task_write(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
    union miniStm32_gpio_sccb_exec_message *exec_msg)
{
    rt_uint16_t temp;

    /* Start */
    gpio_sccb_start(cfg);
    /* Phase 1: ID */
    /* 8-bit: 0, write; 9-bit: x */
    temp = (exec_msg->cmd.other & 0x0000007F) << 2;
    gpio_sccb_writeByte(cfg, temp);
    /* Phase 2, register address */
    /* 9-bit: x */
    temp = (rt_uint16_t)(*(exec_msg->cmd.ptr)) << 1;
    gpio_sccb_writeByte(cfg, temp);
    /* Phase 3, write data */
    /* 9-bit: x */
    temp = (rt_uint16_t)(*(exec_msg->cmd.ptr + 1)) << 1;
    gpio_sccb_writeByte(cfg, temp);
    /* End */
    gpio_sccb_end(cfg);

    gpio_sccb_debug("IO_SCCB%d: write %x, %x %x\n",
        cfg->sccb.number, (exec_msg->cmd.other & 0x0000007F) << 1,
        *(exec_msg->cmd.ptr), *(exec_msg->cmd.ptr + 1));
    exec_msg->ret.ret = RT_EOK;
}

static void gpio_sccb_task_control(
    struct miniStm32_gpio_sccb_unit_struct *cfg,
    union miniStm32_gpio_sccb_exec_message *exec_msg)
{
    switch (exec_msg->cmd.other)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* Suspend device */
        cfg->device.flag |= (rt_uint16_t)RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* Resume device */
        cfg->device.flag &= ~(rt_uint16_t)RT_DEVICE_FLAG_SUSPENDED;
        break;

    case RT_DEVICE_CTRL_MODE_BLOCKING:
        /* Blocking mode operation */
        cfg->sccb.status &= ~(rt_uint16_t)GPIO_SCCB_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->sccb.status |= (rt_uint16_t)GPIO_SCCB_STATUS_NONBLOCKING;
        break;
    }

    exec_msg->ret.ret = RT_EOK;
}

/***************************************************************************//**
* @brief
*   SCCB task main loop
*
* @details
*
* @note
*
* @param[in] parameter
*   Pointer to SCCB unit structure
******************************************************************************/
void gpio_sccb_task_main_loop(void *parameter)
{
    struct miniStm32_gpio_sccb_unit_struct *cfg;
    union miniStm32_gpio_sccb_exec_message *exec_msg;
    rt_thread_t self;
    rt_bool_t chk_block;

    cfg = (struct miniStm32_gpio_sccb_unit_struct *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        GPIO_SCCB_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("IO_SCCB%d: init mq failed!\n", cfg->sccb.number);
        return;
	}

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("IO_SCCB%d: init event failed!\n", cfg->sccb.number);
            return;
        }

    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;

    gpio_sccb_debug("IO_SCCB%d: enter main loop\n", cfg->sccb.number);

GPIO_SCB_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&exec_msg,
            GPIO_SCCB_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        chk_block = RT_FALSE;
        switch (exec_msg->cmd.cmd)
        {
        case GPIO_SCCB_COMMAND_STATUS:
            exec_msg->ret.other = (rt_uint32_t)cfg->sccb.status;
            break;

        case GPIO_SCCB_COMMAND_OPEN:
            gpio_sccb_task_open(cfg, exec_msg);
            break;

        case GPIO_SCCB_COMMAND_CLOSE:
            gpio_sccb_task_close(cfg, exec_msg);
            break;

        case GPIO_SCCB_COMMAND_READ:
            gpio_sccb_task_read(cfg, exec_msg);
            chk_block = RT_TRUE;
            break;

        case GPIO_SCCB_COMMAND_WRITE:
            gpio_sccb_task_write(cfg, exec_msg);
            chk_block = RT_TRUE;
            break;

        case GPIO_SCCB_COMMAND_CONTROL:
            gpio_sccb_task_control(cfg, exec_msg);
            break;

        default:
            break;
        }

        if (chk_block && (cfg->sccb.status & GPIO_SCCB_STATUS_NONBLOCKING))
        {
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize GPIO SCCB device
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
static rt_err_t miniStm32_gpio_sccb_init (rt_device_t dev)
{
    struct miniStm32_gpio_sccb_device *sccb;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    return rt_thread_startup(&gpio_sccb_tasks[sccb->number - 1]->thread);
}

/***************************************************************************//**
 * @brief
 *   Open GPIO SCCB device
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
static rt_err_t miniStm32_gpio_sccb_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct miniStm32_gpio_sccb_device *sccb;
    union miniStm32_gpio_sccb_exec_message exec_msg;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;
    return gpio_sccb_task_exec(sccb->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *   Close GPIO SCCB device
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
static rt_err_t miniStm32_gpio_sccb_close(rt_device_t dev)
{
    struct miniStm32_gpio_sccb_device *sccb;
    union miniStm32_gpio_sccb_exec_message exec_msg;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_CLOSE;
    return gpio_sccb_task_exec(sccb->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *  Read from GPIO SCCB device
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
static rt_size_t miniStm32_gpio_sccb_read (
    rt_device_t     dev,
    rt_off_t        pos,
    void            *buffer,
    rt_size_t       size)
{
    rt_err_t ret;
    struct miniStm32_gpio_sccb_device *sccb;
    union miniStm32_gpio_sccb_exec_message exec_msg;
    rt_bool_t nonblock;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    do
    {
        exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_STATUS;
        ret = gpio_sccb_task_exec(sccb->number, &exec_msg, RT_FALSE);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & GPIO_SCCB_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
        }
        else
        {
            nonblock = RT_FALSE;
        }

        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        ret = gpio_sccb_task_exec(sccb->number, &exec_msg, nonblock);
        if (ret != RT_EOK)
        {
            rt_sem_release(&sccb->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&sccb->lock);
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
 *   Write to GPIO SCCB device
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
static rt_size_t miniStm32_gpio_sccb_write (
    rt_device_t     dev,
    rt_off_t        pos,
    const void*     buffer,
    rt_size_t       size)
{
    rt_err_t ret;
    struct miniStm32_gpio_sccb_device *sccb;
    union miniStm32_gpio_sccb_exec_message exec_msg;
    rt_bool_t nonblock;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    do
    {
        gpio_sccb_debug("IO_SCCB: get status\n");
        exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_STATUS;
        ret = gpio_sccb_task_exec(sccb->number, &exec_msg, RT_FALSE);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & GPIO_SCCB_STATUS_NONBLOCKING)
        {
            nonblock = RT_TRUE;
        }
        else
        {
            nonblock = RT_FALSE;
        }

        /* Lock device */
        gpio_sccb_debug("IO_SCCB: get lock\n");
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_WRITE;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        gpio_sccb_debug("IO_SCCB: call exec\n");
        ret = gpio_sccb_task_exec(sccb->number, &exec_msg, nonblock);
        if (ret != RT_EOK)
        {
            rt_sem_release(&sccb->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&sccb->lock);
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
*   Configure GPIO SCCB device
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
static rt_err_t miniStm32_gpio_sccb_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    rt_err_t ret;
    struct miniStm32_gpio_sccb_device *sccb;
    union miniStm32_gpio_sccb_exec_message exec_msg;

    sccb = (struct miniStm32_gpio_sccb_device *)(dev->user_data);

    do
    {
        /* Lock device */
        if (rt_hw_interrupt_check())
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_NO);
        }
        else
        {
            ret = rt_sem_take(&sccb->lock, RT_WAITING_FOREVER);
        }
        if (ret != RT_EOK)
        {
            break;
        }

        exec_msg.cmd.cmd = GPIO_SCCB_COMMAND_CONTROL;
        exec_msg.cmd.ptr = (rt_uint8_t *)args;
        exec_msg.cmd.other = (rt_uint32_t)cmd;
        ret = gpio_sccb_task_exec(sccb->number, &exec_msg, RT_FALSE);
        if (ret != RT_EOK)
        {
            rt_sem_release(&sccb->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&sccb->lock);
    } while (0);

    return ret;
}

/***************************************************************************//**
 * @brief
 *  GPIO SCCB RX data interrupt handler
 *
 * @details
 *
 * @note
 *  GPIO SCCB slave mode has not implemented yet
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void miniStm32_gpio_sccb_rx_isr(rt_device_t dev)
{
}

/***************************************************************************//**
* @brief
*   Initialize the specified GPIO SCCB unit
*
* @details
*
* @note
*
* @param[in] device
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
*   Pointer to GPIO SCCB device
******************************************************************************/
static rt_err_t miniStm32_gpio_sccb_unit_init(struct miniStm32_gpio_sccb_unit_init *init)
{
    rt_device_t         device;
    struct miniStm32_gpio_sccb_device *sccb;
    rt_uint32_t         flag;
    GPIO_InitTypeDef    gpio_init;

    device = &(init->unit)->device;
    sccb = &(init->unit)->sccb;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        sccb->counter   = 0;
        sccb->number    = init->number;
        sccb->status    = init->config & GPIO_SCCB_STATUS_MASK;
        sccb->rx_mode   = RT_NULL;  // TODO: INT and DMA RX mode

        /* Init lock */
        if (rt_sem_init(&sccb->lock, init->name, 1, RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            break;
        }

        /* Unit number specified setting */
        switch (init->number)
        {
        case 1:
            {
                sccb->delay                 = 500000000 / GPIO_SCCB_FREQUENCY;

                sccb->sccb_device.scl_port  = GPIO_SCCB_SCL_PORT;
                sccb->sccb_device.sda_port  = GPIO_SCCB_SDA_PORT;
                sccb->sccb_device.scl_pin   = GPIO_SCCB_SCL_PIN;
                sccb->sccb_device.sda_pin   = GPIO_SCCB_SDA_PIN;

                RCC_APB2PeriphClockCmd(
                    (GPIO_SCCB_SCL_CLOCK | GPIO_SCCB_SDA_CLOCK),
                    ENABLE);
            }
            break;

        default:
            break;
        }

        /* Config GPIO */
        gpio_init.GPIO_Speed        = GPIO_Speed_10MHz;
        if (init->config & GPIO_SCCB_CONFIG_MASTER)
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_Out_PP;
        }
        else
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_IPU;
        }
        gpio_init.GPIO_Pin          = sccb->sccb_device.scl_pin;
        GPIO_Init(sccb->sccb_device.scl_port, &gpio_init);
        gpio_init.GPIO_Pin          = sccb->sccb_device.sda_pin;
        GPIO_Init(sccb->sccb_device.sda_port, &gpio_init);

        if (init->config & GPIO_SCCB_CONFIG_MASTER)
        {
            sccb->sccb_device.scl_port->BSRR = sccb->sccb_device.scl_pin;
            sccb->sccb_device.sda_port->BSRR = sccb->sccb_device.sda_pin;
        }

        /* Config INT RX? */
        if (!(init->config & GPIO_SCCB_CONFIG_MASTER))
        {
            // TODO: slave mode
/*            flag |= RT_DEVICE_FLAG_INT_RX;

            sccb->rx_mode = rt_malloc(sizeof(struct miniStm32_sccb_int_mode));
            if (sccb->rx_mode == RT_NULL)
            {
                gpio_sccb_debug("sccb%d err: no mem for INT RX\n", sccb->unit);
                break;
            }

            hook.type           = miniStm32_irq_type_sccb;
            hook.unit           = unitNumber - 1;
            hook.cbFunc         = miniStm32_sccb_rx_isr;
            hook.userPtr        = device;
            miniStm32_irq_hook_register(&hook);
*/
        }

        /* Config and register device */
        device->type        = RT_Device_Class_I2CBUS;
        device->rx_indicate = RT_NULL;
        device->tx_complete = RT_NULL;
        device->init        = miniStm32_gpio_sccb_init;
        device->open        = miniStm32_gpio_sccb_open;
        device->close       = miniStm32_gpio_sccb_close;
        device->read        = miniStm32_gpio_sccb_read;
        device->write       = miniStm32_gpio_sccb_write;
        device->control     = miniStm32_gpio_sccb_control;
        device->user_data   = (void *)sccb;

        return rt_device_register(device, init->name, flag);
    } while(0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all GPIO SCCB module related hardware and register GPIO SCCB
*   device to kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t miniStm32_hw_gpio_sccb_init(void)
{
    struct miniStm32_gpio_sccb_unit_init init;

    do
    {
        const rt_uint8_t name[] = GPIO_SCCB_NAME;

        gpio_sccb_tasks[0]   = &gpio_sccb.task;
        init.number         = 1;
        init.config         = GPIO_SCCB_SCCB_MODE;
        init.name           = &name[0];
        init.unit           = &gpio_sccb;
        if (miniStm32_gpio_sccb_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &gpio_sccb.task.thread,
            init.name,
            gpio_sccb_task_main_loop,
            (void *)&gpio_sccb,
            (void *)&gpio_sccb.task.stack,
            sizeof(gpio_sccb.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }

        gpio_sccb_debug("IO_SCCB: H/W init OK!\n", init.number);
        return RT_EOK;
    } while (0);

    rt_kprintf("IO_SCCB err: H/W init failed!\n", init.number);
    return -RT_ERROR;
}

#endif /* defined(RT_USING_GPIO_SCCB) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
