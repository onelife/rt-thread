/***************************************************************************//**
 * @file    drv_gpio_spi.c
 * @brief   GPIO simulation SPI driver of RT-Thread RTOS for MiniSTM32
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

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_gpio_spi.h"
#if (defined(MINISTM32_USING_GPIO_SPI1) || defined(MINISTM32_USING_GPIO_SPI2))
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_GPIO_SPI_DEBUG
#define gpio_spi_debug(format,args...)      rt_kprintf(format, ##args)
#else
#define gpio_spi_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct miniStm32_gpio_spi_task_struct *gpio_spi_tasks[3];

#if defined(MINISTM32_USING_GPIO_SPI1)
 #if GPIO_SPI1_FREQUENCY > 1000000
 #error "GPIO SPI1 clock frequency must be less than or equal to 1 MHz"
 #endif
static struct miniStm32_gpio_spi_unit_struct gpio_spi1;
#endif

#if defined(MINISTM32_USING_GPIO_SPI2)
 #if GPIO_SPI2_FREQUENCY > 1000000
 #error "GPIO SPI2 clock frequency must be less than or equal to 1 MHz"
 #endif
static struct miniStm32_gpio_spi_unit_struct gpio_spi2;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
rt_inline void gpio_spi_delayNs(rt_uint32_t ns)
{
    rt_uint32_t i;

    for(i = 0; i < (SystemCoreClock / 1000000 * ns / 1000); i++)
    {
        __NOP();
    }
}

rt_inline rt_bool_t gpio_spi_toggleSck(GPIO_TypeDef *port, rt_uint16_t pin, rt_uint32_t us)
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

    gpio_spi_delayNs(us);
    return state;
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
static rt_err_t gpio_spi_task_exec(rt_uint8_t number,
    union miniStm32_gpio_spi_exec_message *exec_msg,
    rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &gpio_spi_tasks[number - 1]->rx_msgs,
            (void *)&exec_msg,
            GPIO_SPI_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(GPIO_SPI_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        gpio_spi_debug("IO_SPI%d err: send cmd failed! (%x)\n", number, ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &gpio_spi_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &gpio_spi_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            gpio_spi_debug("IO_SPI%d err: receive event failed!\n", number);
        }
    }

    return ret;
}

static void gpio_spi_task_open(
    struct miniStm32_gpio_spi_unit_struct *cfg,
   union miniStm32_gpio_spi_exec_message *exec_msg)
{
    rt_uint16_t oflag;

    oflag = (rt_uint16_t)exec_msg->cmd.other;
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
    {
        cfg->spi.status |= GPIO_SPI_STATUS_READ_ONLY;
    }
    if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
    {
        cfg->spi.status |= GPIO_SPI_STATUS_WRITE_ONLY;
    }
    if (oflag & RT_DEVICE_OFLAG_NONBLOCKING)
    {
        cfg->spi.status |= GPIO_SPI_STATUS_NONBLOCKING;
    }

    cfg->spi.counter++;

    gpio_spi_debug("IO_SPI%d: Open with flag %x\n", cfg->spi.number, oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void gpio_spi_task_close(
    struct miniStm32_gpio_spi_unit_struct *cfg,
    union miniStm32_gpio_spi_exec_message *exec_msg)
{
    cfg->spi.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void gpio_spi_task_read(
    struct miniStm32_gpio_spi_unit_struct *cfg,
    union miniStm32_gpio_spi_exec_message *exec_msg)
{
    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;

    if (cfg->spi.status & GPIO_SPI_STATUS_WRITE_ONLY)
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
    rt_uint8_t j;

    cfg->spi.spi_device.cs_port->BRR = cfg->spi.spi_device.cs_pin;
    ptr = inst_ptr;
    len = inst_len;
    gpio_spi_debug("IO_SPI%d: RX, TX INS (%d)\n", cfg->spi.number, len);
    /* Write instructions */
    while (len)
    {
        rt_uint8_t temp;

        temp = *(ptr++);
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
            {
                cfg->spi.spi_device.mosi_port->BSRR = cfg->spi.spi_device.mosi_pin;
            }
            else
            {
                cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;
            }
            temp <<= 1;

            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
        }
        len--;
    }
    cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;

    /* Skip some bytes if necessary */
    gpio_spi_debug("IO_SPI%d: RX pos %d\n", cfg->spi.number, pos);
    for (i = 0; i < pos; i++)
    {
        /* dummy write */
        for (j = 0; j < 8; j++)
        {
            cfg->spi.spi_device.mosi_port->BSRR = cfg->spi.spi_device.mosi_pin;

            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
        }
        cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;

        /* dummy read */
        for (j = 0; j < 8; j++)
        {
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
        }
    }

    ptr = rx_buf;
    len = exec_msg->cmd.size;
    gpio_spi_debug("IO_SPI%d: RX data (%d)\n", cfg->spi.number, len);
    /* Read data */
    while (len)
    {
        rt_uint8_t temp;

        temp = 0;
        for (j = 0; j < 8; j++)
        {
            temp <<= 1;

            if ((cfg->spi.status & GPIO_SPI_STATUS_SAMPLE_EDGE) && \
                gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                    cfg->spi.spi_device.sck_pin,
                    cfg->spi.delay))
            {
                if (cfg->spi.spi_device.miso_port->IDR & \
                    cfg->spi.spi_device.miso_pin)
                {
                    temp |= 0x01;
                }
                gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                    cfg->spi.spi_device.sck_pin,
                    cfg->spi.delay);
            }
            else
            {
                gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                    cfg->spi.spi_device.sck_pin,
                    cfg->spi.delay);
                if (cfg->spi.spi_device.miso_port->IDR & \
                    cfg->spi.spi_device.miso_pin)
                {
                    temp |= 0x01;
                }
            }
        }
        gpio_spi_debug("IO_SPI%d: temp %x\n", cfg->spi.number, temp);
        *(ptr++) = temp;
        len--;
    }
    cfg->spi.spi_device.cs_port->BSRR = cfg->spi.spi_device.cs_pin;
    cfg->spi.spi_device.mosi_port->BSRR = cfg->spi.spi_device.mosi_pin;

    exec_msg->ret.size = exec_msg->cmd.size - len;
    exec_msg->ret.ret = RT_EOK;
}

static void gpio_spi_task_write(
    struct miniStm32_gpio_spi_unit_struct *cfg,
    union miniStm32_gpio_spi_exec_message *exec_msg)
{
    rt_off_t pos;
    rt_size_t len;
    rt_uint8_t *ptr;
    rt_uint8_t j;

    if (cfg->spi.status & GPIO_SPI_STATUS_READ_ONLY)
    {
        exec_msg->ret.size = 0;
        exec_msg->ret.ret = -RT_ERROR;
        return;
    }
    pos = (rt_off_t)exec_msg->cmd.other;

    rt_uint8_t inst_len = *(exec_msg->cmd.ptr);
    rt_uint8_t *inst_ptr = exec_msg->cmd.ptr + 1;
    rt_uint8_t *tx_buf = *((rt_uint8_t **)(exec_msg->cmd.ptr + inst_len + 1));

    cfg->spi.spi_device.cs_port->BRR = cfg->spi.spi_device.cs_pin;
    ptr = inst_ptr;
    len = inst_len;
    gpio_spi_debug("IO_SPI%d: TX INS (%d)\n", cfg->spi.number, len);
    /* Write instructions */
    while (len)
    {
        rt_uint8_t temp;

        temp = *(ptr++);
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
            {
                cfg->spi.spi_device.mosi_port->BSRR = cfg->spi.spi_device.mosi_pin;
            }
            else
            {
                cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;
            }
            temp <<= 1;

            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
        }
        len--;
    }
    cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;

    ptr = tx_buf;
    len = exec_msg->cmd.size;
    gpio_spi_debug("IO_SPI: TX DATA (%d)\n", cfg->spi.number, len);
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
    while (len)
    {
        rt_uint8_t temp;

        temp = *(ptr++);
        for (j = 0; j < 8; j++)
        {
            if (temp & 0x80)
            {
                cfg->spi.spi_device.mosi_port->BSRR = cfg->spi.spi_device.mosi_pin;
            }
            else
            {
                cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;
            }
            temp <<= 1;

            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
            gpio_spi_toggleSck(cfg->spi.spi_device.sck_port,
                cfg->spi.spi_device.sck_pin,
                cfg->spi.delay);
        }
        len--;
    }
    cfg->spi.spi_device.cs_port->BSRR = cfg->spi.spi_device.cs_pin;
    cfg->spi.spi_device.mosi_port->BRR = cfg->spi.spi_device.mosi_pin;
}

static void gpio_spi_task_control(
    struct miniStm32_gpio_spi_unit_struct *cfg,
    union miniStm32_gpio_spi_exec_message *exec_msg)
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
        cfg->spi.status &= ~(rt_uint16_t)GPIO_SPI_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->spi.status |= (rt_uint16_t)GPIO_SPI_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_SPI_RX_BUFFER:
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
void gpio_spi_task_main_loop(void *parameter)
{
    struct miniStm32_gpio_spi_unit_struct *cfg;
    union miniStm32_gpio_spi_exec_message *exec_msg;
    rt_thread_t self;
    rt_bool_t chk_block;

    cfg = (struct miniStm32_gpio_spi_unit_struct *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        GPIO_SPI_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("IO_SPI%d: init mq failed!\n", cfg->spi.number);
        return;
	}

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("IO_SPI%d: init event failed!\n", cfg->spi.number);
            return;
        }

    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;

    gpio_spi_debug("IO_SPI%d: enter main loop\n", cfg->spi.number);

GPIO_SPI_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&exec_msg,
            GPIO_SPI_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        chk_block = RT_FALSE;
        switch (exec_msg->cmd.cmd)
        {
        case GPIO_SPI_COMMAND_STATUS:
            exec_msg->ret.other = (rt_uint32_t)cfg->spi.status;
            break;

        case GPIO_SPI_COMMAND_OPEN:
            gpio_spi_task_open(cfg, exec_msg);
            break;

        case GPIO_SPI_COMMAND_CLOSE:
            gpio_spi_task_close(cfg, exec_msg);
            break;

        case GPIO_SPI_COMMAND_READ:
            gpio_spi_task_read(cfg, exec_msg);
            chk_block = RT_TRUE;
            break;

        case GPIO_SPI_COMMAND_WRITE:
            gpio_spi_task_write(cfg, exec_msg);
            chk_block = RT_TRUE;
            break;

        case GPIO_SPI_COMMAND_CONTROL:
            gpio_spi_task_control(cfg, exec_msg);
            break;

        default:
            break;
        }

        if (chk_block && (cfg->spi.status & GPIO_SPI_STATUS_NONBLOCKING))
        {
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize GPIO SPI device
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
static rt_err_t miniStm32_gpio_spi_init (rt_device_t dev)
{
    struct miniStm32_gpio_spi_device *spi;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    return rt_thread_startup(&gpio_spi_tasks[spi->number - 1]->thread);
}

/***************************************************************************//**
 * @brief
 *   Open GPIO SPI device
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
static rt_err_t miniStm32_gpio_spi_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct miniStm32_gpio_spi_device *spi;
    union miniStm32_gpio_spi_exec_message exec_msg;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    exec_msg.cmd.cmd = GPIO_SPI_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;
    return gpio_spi_task_exec(spi->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *   Close GPIO SPI device
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
static rt_err_t miniStm32_gpio_spi_close(rt_device_t dev)
{
    struct miniStm32_gpio_spi_device *spi;
    union miniStm32_gpio_spi_exec_message exec_msg;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    exec_msg.cmd.cmd = GPIO_SPI_COMMAND_CLOSE;
    return gpio_spi_task_exec(spi->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
 * @brief
 *  Read from GPIO SPI device
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
static rt_size_t miniStm32_gpio_spi_read (
    rt_device_t     dev,
    rt_off_t        pos,
    void            *buffer,
    rt_size_t       size)
{
    rt_err_t ret;
    struct miniStm32_gpio_spi_device *spi;
    union miniStm32_gpio_spi_exec_message exec_msg;
    rt_bool_t nonblock;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    do
    {
        exec_msg.cmd.cmd = GPIO_SPI_COMMAND_STATUS;
        ret = gpio_spi_task_exec(spi->number, &exec_msg, RT_FALSE);
        if (ret != RT_EOK)
        {
            break;
        }

        if (exec_msg.ret.other & GPIO_SPI_STATUS_NONBLOCKING)
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

        exec_msg.cmd.cmd = GPIO_SPI_COMMAND_READ;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        ret = gpio_spi_task_exec(spi->number, &exec_msg, nonblock);
        if (ret != RT_EOK)
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
 *   Write to GPIO SPI device
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
static rt_size_t miniStm32_gpio_spi_write (
    rt_device_t     dev,
    rt_off_t        pos,
    const void*     buffer,
    rt_size_t       size)
{
    rt_err_t ret;
    struct miniStm32_gpio_spi_device *spi;
    union miniStm32_gpio_spi_exec_message exec_msg;
    rt_bool_t nonblock;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    do
    {
        exec_msg.cmd.cmd = GPIO_SPI_COMMAND_STATUS;
        do
        {
            ret = gpio_spi_task_exec(spi->number, &exec_msg, RT_FALSE);
            if (ret != RT_EOK)
            {
                break;
            }

            if (exec_msg.ret.other & GPIO_SPI_STATUS_TX_BUSY)
            {
                if (!rt_hw_interrupt_check())
                {
                    rt_thread_sleep(GPIO_SPI_COMMAND_WAIT_TIME);
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

        if (exec_msg.ret.other & GPIO_SPI_STATUS_NONBLOCKING)
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

        exec_msg.cmd.cmd = GPIO_SPI_COMMAND_WRITE;
        exec_msg.cmd.size = size;
        exec_msg.cmd.ptr = (rt_uint8_t *)buffer;
        exec_msg.cmd.other = (rt_uint32_t)pos;
        ret = gpio_spi_task_exec(spi->number, &exec_msg, nonblock);
        if (ret != RT_EOK)
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
*   Configure GPIO SPI device
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
static rt_err_t miniStm32_gpio_spi_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    rt_err_t ret;
    struct miniStm32_gpio_spi_device *spi;
    union miniStm32_gpio_spi_exec_message exec_msg;

    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);

    do
    {
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

        exec_msg.cmd.cmd = GPIO_SPI_COMMAND_CONTROL;
        exec_msg.cmd.ptr = (rt_uint8_t *)args;
        exec_msg.cmd.other = (rt_uint32_t)cmd;
        ret = gpio_spi_task_exec(spi->number, &exec_msg, RT_FALSE);
        if (ret != RT_EOK)
        {
            rt_sem_release(&spi->lock);
            break;
        }

        /* Unlock device */
        ret = rt_sem_release(&spi->lock);
    } while (0);

    return ret;
}

/***************************************************************************//**
 * @brief
 *  GPIO SPI RX data interrupt handler
 *
 * @details
 *
 * @note
 *  GPIO SPI slave mode has not implemented yet
 *
 * @param[in] dev
 *  Pointer to device descriptor
 ******************************************************************************/
void miniStm32_gpio_spi_rx_isr(rt_device_t dev)
{
#if 0
    struct miniStm32_gpio_spi_device   *spi;
    struct miniStm32_spi_int_mode_t *int_rx;

    /* interrupt mode receive */
    RT_ASSERT(dev->flag & RT_DEVICE_FLAG_INT_RX);
    spi = (struct miniStm32_gpio_spi_device *)(dev->user_data);
    int_rx = (struct miniStm32_spi_int_mode_t *)(spi->rx_mode);

    /* Set status */
    spi->status |= SPI_STATE_RX_BUSY;

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
            int_rx->read_index ++;
            if (int_rx->read_index >= SPI_RX_BUFFER_SIZE)
            {
                int_rx->read_index = 0;
            }
        }

        /* enable interrupt */
        rt_hw_interrupt_enable(level);
    }

    /* invoke callback */
    if (dev->rx_indicate != RT_NULL)
    {
        rt_size_t rx_length;

        /* get rx length */
        rx_length = int_rx->read_index > int_rx->save_index ?
            SPI_RX_BUFFER_SIZE - int_rx->read_index + int_rx->save_index : \
            int_rx->save_index - int_rx->read_index;

        dev->rx_indicate(dev, rx_length);
    }
#endif
}

/***************************************************************************//**
* @brief
*   Initialize the specified GPIO SPI unit
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
*   Pointer to GPIO SPI device
******************************************************************************/
static rt_err_t miniStm32_gpio_spi_unit_init(struct miniStm32_gpio_spi_unit_init *init)
{
    rt_device_t         device;
    struct miniStm32_gpio_spi_device *spi;
    rt_uint32_t         flag;
    GPIO_InitTypeDef    gpio_init;

    device = &(init->unit)->device;
    spi = &(init->unit)->spi;
    flag = RT_DEVICE_FLAG_RDWR;

    do
    {
        spi->counter    = 0;
        spi->number     = init->number;
        spi->status      = init->config & GPIO_SPI_STATUS_MASK;
        /*
            b'00: Clock idle low, sample on rising edge
            b'01: Clock idle low, sample on falling edge
            b'10: Clock idle high, sample on falling edge
            b'11: Clock idle high, sample on rising edge.
            =>
            b'00: Clock idle low, sample on falling edge
            b'01: Clock idle low, sample on rising edge
            b'10: Clock idle high, sample on falling edge
            b'11: Clock idle high, sample on rising edge.
        */
        if (GPIO_SPI_CONFIG_CLK_MODE_GET(spi->status) == 0x00)
        {
            spi->status  = GPIO_SPI_CONFIG_CLK_MODE_SET(spi->status, 0x01);
        }
        else if (GPIO_SPI_CONFIG_CLK_MODE_GET(spi->status) == 0x01)
        {
            spi->status  = GPIO_SPI_CONFIG_CLK_MODE_SET(spi->status, 0x00);
        }
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
                spi->delay                  = 500000000 / GPIO_SPI1_FREQUENCY;

                spi->spi_device.sck_port    = GPIO_SPI1_SCK_PORT;
                spi->spi_device.mosi_port   = GPIO_SPI1_MOSI_PORT;
                spi->spi_device.miso_port   = GPIO_SPI1_MISO_PORT;
                spi->spi_device.cs_port     = GPIO_SPI1_CS_PORT;
                spi->spi_device.sck_pin     = GPIO_SPI1_SCK_PIN;
                spi->spi_device.mosi_pin    = GPIO_SPI1_MOSI_PIN;
                spi->spi_device.miso_pin    = GPIO_SPI1_MISO_PIN;
                spi->spi_device.cs_pin      = GPIO_SPI1_CS_PIN;

                RCC_APB2PeriphClockCmd(
                    (GPIO_SPI1_SCK_CLOCK | GPIO_SPI1_MOSI_CLOCK | \
                    GPIO_SPI1_MISO_CLOCK | GPIO_SPI1_CS_CLOCK),
                    ENABLE);
            }
            break;
#if defined(RT_USING_GPIO_SPI2)
        case 2:
            {
                spi->delay                  = 1000000 / GPIO_SPI2_FREQUENCY;

                spi->spi_device.sck_port    = GPIO_SPI2_SCK_PORT;
                spi->spi_device.mosi_port   = GPIO_SPI2_MOSI_PORT;
                spi->spi_device.miso_port   = GPIO_SPI2_MISO_PORT;
                spi->spi_device.cs_port     = GPIO_SPI2_CS_PORT;
                spi->spi_device.sck_pin     = GPIO_SPI2_SCK_PIN;
                spi->spi_device.mosi_pin    = GPIO_SPI2_MOSI_PIN;
                spi->spi_device.miso_pin    = GPIO_SPI2_MISO_PIN;
                spi->spi_device.cs_pin      = GPIO_SPI2_CS_PIN;

                RCC_APB2PeriphClockCmd(
                    (GPIO_SPI2_SCK_CLOCK | GPIO_SPI2_MOSI_CLOCK | \
                    GPIO_SPI2_MISO_CLOCK | GPIO_SPI2_CS_CLOCK),
                    ENABLE);
            }
            break;
#endif
        default:
            break;
        }

        /* Config GPIO */
        gpio_init.GPIO_Speed        = GPIO_Speed_10MHz;
        if (init->config & GPIO_SPI_CONFIG_MASTER)
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_IPU;
        }
        else
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_Out_PP;
        }
        gpio_init.GPIO_Pin          = spi->spi_device.miso_pin;
        GPIO_Init(spi->spi_device.miso_port, &gpio_init);
        if (init->config & GPIO_SPI_CONFIG_MASTER)
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_Out_PP;
        }
        else
        {
            gpio_init.GPIO_Mode     = GPIO_Mode_IPU;
        }
        gpio_init.GPIO_Pin          = spi->spi_device.mosi_pin;
        GPIO_Init(spi->spi_device.mosi_port, &gpio_init);
        gpio_init.GPIO_Pin          = spi->spi_device.sck_pin;
        GPIO_Init(spi->spi_device.sck_port, &gpio_init);
        gpio_init.GPIO_Pin          = spi->spi_device.cs_pin;
        GPIO_Init(spi->spi_device.cs_port, &gpio_init);

        if (init->config & GPIO_SPI_CONFIG_MASTER)
        {
            spi->spi_device.cs_port->BSRR = spi->spi_device.cs_pin;
            spi->spi_device.mosi_port->BSRR = spi->spi_device.mosi_pin;
            if (init->config & GPIO_SPI_CONFIG_IDLE_LEVEL)
            {
                spi->spi_device.sck_port->BSRR = spi->spi_device.sck_pin;
                gpio_spi_debug("IO_SPI: SCK idle high\n");
            }
            else
            {
                spi->spi_device.sck_port->BRR = spi->spi_device.sck_pin;
                gpio_spi_debug("IO_SPI: SCK idle low\n");
            }
        }
        else
        {
            spi->spi_device.miso_port->BSRR = spi->spi_device.miso_pin;
        }

        /* Config INT RX? */
        if (!(init->config & GPIO_SPI_CONFIG_MASTER))
        {
            // TODO: slave mode
/*            flag |= RT_DEVICE_FLAG_INT_RX;

            spi->rx_mode = rt_malloc(sizeof(struct miniStm32_spi_int_mode));
            if (spi->rx_mode == RT_NULL)
            {
                gpio_spi_debug("SPI%d err: no mem for INT RX\n", spi->unit);
                break;
            }

            hook.type           = miniStm32_irq_type_spi;
            hook.unit           = unitNumber - 1;
            hook.cbFunc         = miniStm32_spi_rx_isr;
            hook.userPtr        = device;
            miniStm32_irq_hook_register(&hook);
*/
        }

        /* Config and register device */
        device->type        = RT_Device_Class_SPIBUS;
        device->rx_indicate = RT_NULL;
        device->tx_complete = RT_NULL;
        device->init        = miniStm32_gpio_spi_init;
        device->open        = miniStm32_gpio_spi_open;
        device->close       = miniStm32_gpio_spi_close;
        device->read        = miniStm32_gpio_spi_read;
        device->write       = miniStm32_gpio_spi_write;
        device->control     = miniStm32_gpio_spi_control;
        device->user_data   = (void *)spi;

        return rt_device_register(device, init->name, flag);
    } while(0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize all GPIO SPI module related hardware and register GPIO SPI device
*   to kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t miniStm32_hw_gpio_spi_init(void)
{
    struct miniStm32_gpio_spi_unit_init init;

    do
    {
#if defined(MINISTM32_USING_GPIO_SPI1)
        const rt_uint8_t name[] = GPIO_SPI1_NAME;

        gpio_spi_tasks[0]   = &gpio_spi1.task;
        init.number         = 1;
        init.config         = GPIO_SPI1_SPI_MODE;
        init.name           = &name[0];
        init.unit           = &gpio_spi1;
        if (miniStm32_gpio_spi_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &gpio_spi1.task.thread,
            init.name,
            gpio_spi_task_main_loop,
            (void *)&gpio_spi1,
            (void *)&gpio_spi1.task.stack,
            sizeof(gpio_spi1.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

#if defined(MINISTM32_USING_GPIO_SPI2)
        const rt_uint8_t name[] = GPIO_SPI2_NAME;

        gpio_spi_tasks[0]   = &gpio_spi2.task;
        init.number         = 2;
        init.config         = GPIO_SPI2_SPI_MODE;
        init.name           = &name[0];
        init.unit           = &gpio_spi2;
        if (miniStm32_gpio_spi_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &gpio_spi2.task.thread,
            init.name,
            gpio_spi_task_main_loop,
            (void *)&gpio_spi2,
            (void *)&gpio_spi2.task.stack,
            sizeof(gpio_spi2.task.stack),
            TASK_PRIORITY_DRIVER,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }
#endif

        gpio_spi_debug("IO_SPI%d: H/W init OK!\n", init.number);
        return RT_EOK;
    } while (0);

    rt_kprintf("IO_SPI%d err: H/W init failed!\n", init.number);
    return -RT_ERROR;
}

#endif /* (defined(MINISTM32_USING_GPIO_SPI1) || defined(MINISTM32_USING_GPIO_SPI2)) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
