/***************************************************************************//**
 * @file    drv_touch.c
 * @brief   Touch screen driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-01-31   onelife     Initial creation of touch screen driver for
 *  MiniSTM32
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_touch.h"
#if defined(MINISTM32_USING_TOUCH)
//#include <rtgui/rtgui.h>
#include <rtgui/event.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_TOUCH_DEBUG
#define touch_debug(format,args...)         rt_kprintf(format, ##args)
#else
#define touch_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct miniStm32_touch_task_struct *touch_task;
static struct miniStm32_touch_unit_struct touch;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *	Touch screen interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
static rt_err_t touch_task_calibration(struct rtgui_touch_device *touch)
{
    rt_device_t lcd;
    rt_bool_t lcd_open;
    rt_uint16_t arg;
    rt_uint16_t x1, y1, x2, y2;
    rt_uint16_t xy[4][2];
    rt_uint32_t temp1, temp2, temp_fac, temp_off;

    lcd_open = RT_FALSE;

    do
    {
        /* Find LCD device */
        lcd = rt_device_find(LCD_DEVICE_NAME);
        if (lcd == RT_NULL)
        {
            touch_debug("Touch err: Can't find device %s!\n", LCD_DEVICE_NAME);
            break;
        }
        touch_debug("Touch: Find device %s\n", LCD_DEVICE_NAME);
        /* Open LCD device */
        if (lcd->open(lcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            break;
        }
        lcd_open = RT_TRUE;

        arg = 0;
        if (lcd->control(lcd, RTGRAPHIC_CTRL_LCD_CALIBRATION, (void *)&arg) != RT_EOK)
        {
            break;
        }

        arg = 1;
        if (lcd->control(lcd, RTGRAPHIC_CTRL_LCD_CALIBRATION, (void *)&arg) != RT_EOK)
        {
            break;
        }
        touch->calibration = 1;
        while(touch->calibration == 1)
        {
            rt_thread_sleep(RT_TICK_PER_SECOND / 2);
        }
        xy[0][0] = touch->x;
        xy[0][1] = touch->y;
        touch_debug("Touch: x1 %x, y1 %x\n", xy[0][0], xy[0][1]);

        arg = 2;
        if (lcd->control(lcd, RTGRAPHIC_CTRL_LCD_CALIBRATION, (void *)&arg) != RT_EOK)
        {
            break;
        }
        touch->calibration = 1;
        while(touch->calibration == 1)
        {
            rt_thread_sleep(RT_TICK_PER_SECOND / 2);
        }
        xy[1][0] = touch->x;
        xy[1][1] = touch->y;
        touch_debug("Touch: x2 %x, y2 %x\n", xy[1][0], xy[1][1]);

        arg = 3;
        if (lcd->control(lcd, RTGRAPHIC_CTRL_LCD_CALIBRATION, (void *)&arg) != RT_EOK)
        {
            break;
        }
        touch->calibration = 1;
        while(touch->calibration == 1)
        {
            rt_thread_sleep(RT_TICK_PER_SECOND / 2);
        }
        xy[2][0] = touch->x;
        xy[2][1] = touch->y;
        touch_debug("Touch: x3 %x, y3 %x\n", xy[2][0], xy[2][1]);

        arg = 4;
        if (lcd->control(lcd, RTGRAPHIC_CTRL_LCD_CALIBRATION, (void *)&arg) != RT_EOK)
        {
            break;
        }
        touch->calibration = 1;
        while(touch->calibration == 1)
        {
            rt_thread_sleep(RT_TICK_PER_SECOND / 2);
        }
        xy[3][0] = touch->x;
        xy[3][1] = touch->y;
        touch_debug("Touch: x4 %x, y4 %x\n", xy[3][0], xy[3][1]);

        temp1 = (xy[0][0] + xy[2][0]) / 2;
        temp2 = (xy[1][0] + xy[3][0]) / 2;
        temp_fac = ((rt_uint32_t)arg << 16) / temp1;
        temp_off = (touch->width - arg) - ((temp2 * temp_fac) >> 16);

        temp1 = touch->width - arg * 2;
        touch->xfac = temp_fac * temp1 / (temp1 - temp_off);
        touch->xoff = 0 - temp_off * arg / (temp1 - temp_off);

        temp1 = (xy[0][1] + xy[1][1]) / 2;
        temp2 = (xy[2][1] + xy[3][1]) / 2;
        temp_fac = ((rt_uint32_t)arg << 16) / temp1;
        temp_off = (touch->height - arg) - ((temp2 * temp_fac) >> 16);

        temp1 = touch->height - arg * 2;
        touch->yfac = temp_fac * temp1 / (temp1 - temp_off);
        touch->yoff = 0 - temp_off * arg / (temp1 - temp_off);

        lcd->close(lcd);
        touch->calibration = 0;
        touch_debug("Touch: xfac %x, yfac %x, xoff %x, yoff %x\n",
            touch->xfac, touch->yfac, touch->xoff, touch->yoff);

        return RT_EOK;
    } while (0);

    if (lcd_open)
    {
        lcd->close(lcd);
    }

    touch_debug("Touch err: Calibration failed!\n");
    return -RT_ERROR;
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
static rt_err_t touch_task_notice(rt_uint32_t cmd)
{
    rt_err_t ret;

    ret = rt_mq_send(
        &touch_task->rx_msgs,
        (void *)&cmd,
        TOUCH_RX_MESSAGE_SIZE);
    if (ret != RT_EOK)
    {
        touch_debug("Touch err: send cmd failed! (%x)\n", ret);
    }

    return ret;
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
static rt_err_t touch_task_exec(
    union miniStm32_touch_exec_message *exec_msg)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &touch_task->rx_msgs,
            (void *)&exec_msg,
            TOUCH_RX_MESSAGE_SIZE);
        if (ret == -RT_EFULL)
        {
            rt_thread_sleep(TOUCH_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        touch_debug("Touch err: send cmd failed! (%x)\n", ret);
        return ret;
    }

    if (rt_hw_interrupt_check())
    {
        ret = rt_event_recv(
            &touch_task->tx_evts,
            exec_msg->cmd.cmd,
            RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
            RT_WAITING_NO,
            &exec_msg->ret.cmd);
    }
    else
    {
        ret = rt_event_recv(
            &touch_task->tx_evts,
            exec_msg->cmd.cmd,
            RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
            RT_WAITING_FOREVER,
            &exec_msg->ret.cmd);
    }
    if (ret != RT_EOK)
    {
        touch_debug("Touch err: receive event failed! (%x)\n", ret);
    }

    return ret;
}

/***************************************************************************//**
 * @brief
 *	Touch screen interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
static touch_task_getPosition(struct rtgui_touch_device *touch)
{
    rt_bool_t err = RT_FALSE;
    rt_size_t rst;
    rt_uint16_t xy[TOUCH_SAMPLE_NUMBER][2], sum[2] = {0};
    rt_uint8_t buf_ins[6], temp[3];
    rt_uint8_t i, j;

    do
    {
        /* Read samples */
        buf_ins[0] = 1;                                 /* Instruction length */
        for (i = 0; i < TOUCH_SAMPLE_NUMBER; i++)
        {
            buf_ins[1] = TOUCH_CMD_READ_X;              /* Command */
            *(rt_uint8_t **)(&buf_ins[2]) = &temp[0];   /* Pointer to RX buffer */
            if ((rst = touch->spi->read(touch->spi, MINISTM32_NO_DATA, buf_ins,
                2)) != 2)
            {
                touch_debug("Touch err: read X failed! (%d, %x)\n", rst, xy[i][0]);
                err = RT_TRUE;
                break;
            }
            temp[2] = temp[0];
            xy[i][0] = *(rt_uint16_t *)&temp[1];

            buf_ins[1] = TOUCH_CMD_READ_Y;              /* Command */
            if ((rst = touch->spi->read(touch->spi, MINISTM32_NO_DATA, buf_ins,
                2)) != 2)
            {
                touch_debug("Touch err: read Y failed! (%d, %x)\n", rst, xy[i][0]);
                err = RT_TRUE;
                break;
            }
            temp[2] = temp[0];
            xy[i][1] = *(rt_uint16_t *)&temp[1];

            //touch_debug("Touch: get (%d, %d)\n", xy[i][0], xy[i][1]);
        }
        if (err)
        {
            touch->x = (rt_uint16_t)(-1);
            touch->y = (rt_uint16_t)(-1);
            break;
        }

        /* Band-pass filtering by select 1/3 samples, and calculate the sum value */
        i = 0; j = 0;
        while (1)
        {
            if (j == 2)
            {
                j = 0;
                i += 3;
            }
            if (i >= TOUCH_SAMPLE_NUMBER)
            {
                break;
            }

            if (xy[i][j] > xy[i + 1][j])
            {
                if (xy[i + 1][j] > xy[i + 2][j])
                {
                    sum[j] += xy[i + 1][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i + 1);
                }
                else if (xy[i][j] > xy[i + 2][j])
                {
                    sum[j] += xy[i + 2][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i + 2);
                }
                else
                {
                    sum[j] += xy[i][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i);
                }
            }
            else
            {
                if (xy[i + 1][j] < xy[i + 2][j])
                {
                    sum[j] += xy[i + 1][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i + 1);
                }
                else if (xy[i][j] > xy[i + 2][j])
                {
                    sum[j] += xy[i][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i);
                }
                else
                {
                    sum[j] += xy[i + 2][j] >> 3;
                    //touch_debug("Touch: %c take %d\n", (j ? 'Y' : 'X'), i + 2);
                }
            }
            j++;
        }

        touch->x = sum[0] / (TOUCH_SAMPLE_NUMBER / 3);
        touch->y = sum[1] / (TOUCH_SAMPLE_NUMBER / 3);
    } while (0);

    touch_debug("Touch: pos (%d, %d)\n", touch->x, touch->y);
    return;
}

static touch_task_getPosition0(struct rtgui_touch_device *touch)
{
    rt_thread_sleep(TOUCH_FIRST_MEASURE_DELAY);
    if (GPIO_ReadInputDataBit(TOUCH_INT_PORT, TOUCH_INT_PIN))
    {
        /* Enable interrupt */
        EXTI_ClearFlag(TOUCH_INT_EXTI_LINE);
        EXTI->IMR |= TOUCH_INT_EXTI_LINE;
    }
    else
    {
        /* Start timer */
        rt_timer_start(&touch->timer);
        touch_task_getPosition(touch);
    }
}

static void touch_task_report(struct rtgui_touch_device *touch)
{
    struct rtgui_event_mouse mouse;

    if (GPIO_ReadInputDataBit(TOUCH_INT_PORT, TOUCH_INT_PIN))
    {
        /* Stop timer */
        rt_timer_stop(&touch->timer);

        touch_debug("Touch: timeout -> up, (%x, %x)\n", touch->x, touch->y);

        if (!((touch->x == (rt_uint16_t)(-1)) && (touch->y == (rt_uint16_t)(-1))))
        {
            if (!touch->calibration)
            {
                rt_int16_t temp_x = touch->xoff + ((touch->xfac * touch->x) >> 16);
                rt_int16_t temp_y = touch->yoff + ((touch->yfac * touch->y) >> 16);

                if (temp_x < 0)
                {
                    temp_x = 0;
                }
                else if (temp_x > touch->width)
                {
                    temp_x = touch->width;
                }
                if (temp_y < 0)
                {
                    temp_y = 0;
                }
                else if (temp_y > touch->height)
                {
                    temp_y = touch->height;
                }

                mouse.parent.type = RTGUI_EVENT_MOUSE_BUTTON;
                mouse.parent.sender = RT_NULL;
                mouse.wid = RT_NULL;
                mouse.x = temp_x;
                mouse.y = temp_y;

                if ((touch->stay == 0) && (touch->move <= 1))
                {   /* Send left click event */
                    mouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_DOWN);
                    rtgui_server_post_event(&mouse.parent, sizeof(struct rtgui_event_mouse));
                    mouse.button = (RTGUI_MOUSE_BUTTON_LEFT | RTGUI_MOUSE_BUTTON_UP);
                    rtgui_server_post_event(&mouse.parent, sizeof(struct rtgui_event_mouse));

                    touch_debug("Touch: left click (%d, %d)\n", mouse.x, mouse.y);
                }
                else if ((touch->stay >= TOUCH_RIGHT_CLICK_COUNT) && (touch->move <= 1))
                {   /* Send right click event */
                    mouse.button = (RTGUI_MOUSE_BUTTON_RIGHT | RTGUI_MOUSE_BUTTON_DOWN);
                    rtgui_server_post_event(&mouse.parent, sizeof(struct rtgui_event_mouse));
                    mouse.button = (RTGUI_MOUSE_BUTTON_RIGHT | RTGUI_MOUSE_BUTTON_UP);
                    rtgui_server_post_event(&mouse.parent, sizeof(struct rtgui_event_mouse));

                    touch_debug("Touch: right click (%d, %d)\n", mouse.x, mouse.y);
                }
            }
            else if ((touch->stay >= TOUCH_RIGHT_CLICK_COUNT) && (touch->move <= 1))
            {
                touch->calibration++;
                touch_debug("Touch: timeout -> calibration\n");
            }

            touch_debug("Touch: stay %d, move%d\n", touch->stay, touch->move);
        }

        touch->stay = touch->move = 0;

        /* Enable interrupt */
        EXTI_ClearFlag(TOUCH_INT_EXTI_LINE);
        EXTI->IMR |= TOUCH_INT_EXTI_LINE;
    }
    else
    {
        rt_uint16_t x = touch->x;
        rt_uint16_t y = touch->y;

        touch_task_getPosition(touch);
        if (!((touch->x == (rt_uint16_t)(-1)) && (touch->y == (rt_uint16_t)(-1))))
        {
            if ((touch->x < (x + TOUCH_POINTER_STAY_LIMIT)) && \
                (touch->x > (x - TOUCH_POINTER_STAY_LIMIT)) && \
                (touch->y < (y + TOUCH_POINTER_STAY_LIMIT)) && \
                (touch->y > (y - TOUCH_POINTER_STAY_LIMIT)))
            {
                if (touch->stay < RT_UINT8_MAX)
                {
                    touch->stay++;
                }
                touch_debug("Touch: timeout -> stay %d\n", touch->stay);
            }
            else
            {
                if (touch->move < RT_UINT8_MAX)
                {
                    touch->move++;
                }
                if (touch->move == 1)
                {
                    touch->x = x;
                    touch->y = y;
                }

                if (!touch->calibration)
                {
                    mouse.parent.type = RTGUI_EVENT_MOUSE_MOTION;
                    mouse.parent.sender = RT_NULL;
                    mouse.x = touch->xoff + ((touch->xfac * touch->x) >> 16);
                    mouse.y = touch->yoff + ((touch->yfac * touch->y) >> 16);
                    mouse.button = 0;
                    rtgui_server_post_event(&mouse.parent, sizeof(struct rtgui_event_mouse));
                }
            }
        }
    }
}

static void touch_task_control(struct miniStm32_touch_unit_struct *cfg,
    union miniStm32_touch_exec_message *exec_msg)
{
    switch (exec_msg->cmd.other)
    {
    case RT_DEVICE_CTRL_SUSPEND:
        /* Suspend device */
        cfg->device.flag |= RT_DEVICE_FLAG_SUSPENDED;
        exec_msg->ret.ret = RT_EOK;
        break;

    case RT_DEVICE_CTRL_RESUME:
        /* Resume device */
        cfg->device.flag &= ~RT_DEVICE_FLAG_SUSPENDED;
        exec_msg->ret.ret = RT_EOK;
        break;

    case RTGRAPHIC_CTRL_TOUCH_CALIBRATION:
        exec_msg->ret.ret = touch_task_calibration(&cfg->touch);
        break;
    }
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
void touch_task_main_loop(void *parameter)
{
    struct miniStm32_touch_unit_struct *cfg;
    union miniStm32_touch_exec_message *exec_msg;
    rt_thread_t self;
    rt_uint32_t cmd;

    cfg = (struct miniStm32_touch_unit_struct *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        TOUCH_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("Touch: init mq failed!\n");
        return;
	}

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("Touch: init event failed!\n");
            return;
        }

    /* Open SPI device */
    if (cfg->touch.spi->open(cfg->touch.spi, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("Touch: open SPI failed!\n");
        return;
    }

    /* Disable interrupt */
    EXTI->IMR &= ~(TOUCH_INT_EXTI_LINE);

    /* Dummy read */
    touch_task_getPosition(&cfg->touch);

    /* Enable interrupt */
    EXTI_ClearFlag(TOUCH_INT_EXTI_LINE);
    EXTI->IMR |= TOUCH_INT_EXTI_LINE;

TOUCH_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&exec_msg,
            TOUCH_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        if (((rt_uint32_t)exec_msg & TOUCH_COMMAND_MASK) == TOUCH_COMMAND_MASK)
        {
            cmd = (rt_uint32_t)exec_msg & ~(rt_uint32_t)TOUCH_COMMAND_MASK;
        }
        else
        {
            cmd = exec_msg->cmd.cmd;
        }

        switch (cmd)
        {
        case TOUCH_COMMAND_INTERRUPT:
            touch_task_getPosition0(&cfg->touch);
            touch_debug("Touch: first position (%d, %d)\n", cfg->touch.x,
                cfg->touch.y);
            break;

        case TOUCH_COMMAND_TIMEOUT:
            touch_debug("Touch: timeout\n");
            touch_task_report(&cfg->touch);
            break;

        case TOUCH_COMMAND_CONTROL:
            touch_task_control(cfg, exec_msg);
            break;

        default:
            break;
        }

        if (cmd != TOUCH_COMMAND_CONTROL)
        {
            continue;
        }

        rt_event_send(&cfg->task.tx_evts, exec_msg->ret.cmd);
    }
}

/***************************************************************************//**
 * @brief
 *	Touch screen interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
static void miniStm32_touch_isr(rt_device_t dev)
{

    /* Disable interrupt */
    EXTI->IMR &= ~(TOUCH_INT_EXTI_LINE);

    /* IRQ 0~31 => ICER[0], IRQ 32~63 => ICER[1], IRQ 64~67 => ICER[2] */
//        NVIC->ICER[TOUCH_INT_EXTI_IRQ >> 0x05] =
//        (rt_uint32_t)0x01 << (TOUCH_INT_EXTI_IRQ & (rt_uint8_t)0x1F);

    touch_task_notice(TOUCH_COMMAND_MASK | TOUCH_COMMAND_INTERRUPT);
}

/***************************************************************************//**
 * @brief
 *	Touch screen interrupt handler
 *
 * @details
 *
 * @note
 ******************************************************************************/
static void miniStm32_touch_timer(void *parameter)
{
    touch_task_notice(TOUCH_COMMAND_MASK | TOUCH_COMMAND_TIMEOUT);
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
static rt_err_t miniStm32_touch_init (rt_device_t dev)
{
    return rt_thread_startup(&touch_task->thread);
}

/***************************************************************************//**
 * @brief
 *   Open touch screen device
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
static rt_err_t miniStm32_touch_open(rt_device_t dev, rt_uint16_t oflag)
{
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Close touch screen device
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
static rt_err_t miniStm32_touch_close(rt_device_t dev)
{
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Configure touch screen device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] cmd
 *   Touch screen control command
 *
 * @param[in] args
 *   Arguments
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t miniStm32_touch_control(rt_device_t dev,
    rt_uint8_t cmd, void *args)
{
    union miniStm32_touch_exec_message exec_msg;

    exec_msg.cmd.cmd = TOUCH_COMMAND_CONTROL;
    exec_msg.cmd.ptr = (rt_uint8_t *)args;
    exec_msg.cmd.other = (rt_uint32_t)cmd;
    return touch_task_exec(&exec_msg);
}

/***************************************************************************//**
* @brief
*   Initialize the touch screen unit
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
static rt_err_t miniStm32_touch_unit_init(
    struct miniStm32_touch_unit_init *init)
{
    rt_device_t         device, lcd;
    struct rtgui_touch_device *touch;
    GPIO_InitTypeDef    gpio_init;
    EXTI_InitTypeDef    exti_init;
    NVIC_InitTypeDef    nvic_init;
    miniStm32_irq_hook_init_t hook;
    struct rt_device_graphic_info lcd_info;

    do
    {
        device = &(init->unit)->device;
        touch = &(init->unit)->touch;

        /* Find LCD device */
        lcd = rt_device_find(LCD_DEVICE_NAME);
        if (lcd == RT_NULL)
        {
            touch_debug("Touch err: Can't find device %s!\n", LCD_DEVICE_NAME);
            break;
        }
        touch_debug("Touch: Find device %s\n", LCD_DEVICE_NAME);
        /* Open LCD device */
        if (lcd->open(lcd, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            break;
        }
        /* Get LCD size */
        if (lcd->control(lcd, RTGRAPHIC_CTRL_GET_INFO, (void *)&lcd_info) \
            != RT_EOK)
        {
            lcd->close(lcd);
            break;
        }
        /* Close LCD device */
        if (lcd->close(lcd) != RT_EOK)
        {
            break;
        }

        /* Find SPI device */
        touch->spi = rt_device_find(TOUCH_USING_DEVICE_NAME);
        if (touch->spi == RT_NULL)
        {
            touch_debug("Touch err: Can't find device %s!\n",
                TOUCH_USING_DEVICE_NAME);
            break;
        }
        touch_debug("Touch: Find device %s\n", TOUCH_USING_DEVICE_NAME);

        /* Init timer */
        rt_timer_init(&touch->timer, init->name, miniStm32_touch_timer,
            RT_NULL, TOUCH_REPORT_PERIOD, RT_TIMER_FLAG_PERIODIC);

        touch->stay             = touch->move = 0;
        touch->calibration      = 0;
        touch->x                = touch->y = 0;
        touch->width            = lcd_info.width;
        touch->height           = lcd_info.height;
        touch->status           = 0;
        // TODO:
        touch->xfac             = 4361;
        touch->yfac             = 5854;
        touch->xoff             = -16;
        touch->yoff             = -15;

        /* Config interrupt pin */
        RCC_APB2PeriphClockCmd((TOUCH_INT_CLOCK | RCC_APB2Periph_AFIO), ENABLE);

        gpio_init.GPIO_Pin      = TOUCH_INT_PIN;
        gpio_init.GPIO_Speed    = GPIO_Speed_50MHz;
        gpio_init.GPIO_Mode     = GPIO_Mode_IPU;
        GPIO_Init(TOUCH_INT_PORT, &gpio_init);

        /* Config interrupt handler */
        hook.type               = miniStm32_irq_type_exti;
        hook.unit               = TOUCH_INT_EXTI_UNIT;
        hook.cbFunc             = miniStm32_touch_isr;
        hook.userPtr            = (void *)device;
        miniStm32_irq_hook_register(&hook);

        /* Config external interrupt */
        GPIO_EXTILineConfig(TOUCH_INT_EXTI_PORT, TOUCH_INT_EXTI_PIN);
        EXTI_ClearFlag(TOUCH_INT_EXTI_LINE);
        exti_init.EXTI_Line     = TOUCH_INT_EXTI_LINE;
        exti_init.EXTI_Mode     = EXTI_Mode_Interrupt;
        exti_init.EXTI_Trigger  = EXTI_Trigger_Falling;
        exti_init.EXTI_LineCmd  = ENABLE;
        EXTI_Init(&exti_init);

        /* Config NVIC */
        NVIC_ClearPendingIRQ(TOUCH_INT_EXTI_IRQ);
        nvic_init.NVIC_IRQChannel = TOUCH_INT_EXTI_IRQ;
        nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
        nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
        nvic_init.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&nvic_init);

        /* Register touch screen device */
        device->type            = RT_Device_Class_MTD;
        device->init            = miniStm32_touch_init;
        device->open            = miniStm32_touch_open;
        device->close           = miniStm32_touch_close;
        device->read            = RT_NULL;
        device->write           = RT_NULL;
        device->control         = miniStm32_touch_control;
        device->user_data       = (void *)touch;

        return rt_device_register(device, TOUCH_DEVICE_NAME,RT_DEVICE_FLAG_RDWR);
    } while (0);

    return -RT_ERROR;
}

/***************************************************************************//**
* @brief
*   Initialize touch screen related hardware and register the device to kernel
*
* @details
*
* @note
*
* @return
*   Error code
******************************************************************************/
rt_err_t miniStm32_hw_touch_init(void)
{
    struct miniStm32_touch_unit_init init;

    const rt_uint8_t name[] = TOUCH_DEVICE_NAME;

    do
    {
        touch_task          = &touch.task;
        init.name           = &name[0];
        init.unit           = &touch;
        if (miniStm32_touch_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &touch.task.thread,
            init.name,
            touch_task_main_loop,
            (void *)&touch,
            (void *)&touch.task.stack,
            sizeof(touch.task.stack),
            TASK_PRIORITY_DRIVER + 1,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }

        touch_debug("Touch: HW init OK\n");
        return RT_EOK;
    } while (0);

    rt_kprintf("Touch: HW init failed!\n");
    return -RT_ERROR;
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>

void adjust_touch(void)
{
    rt_kprintf(" Touch screen calibrating...\n");
    touch_task_calibration(&touch.touch);
    rt_kprintf(" Calibration done. xfac %d, yfac %d, xoff %d, yoff %d\n",
        touch.touch.xfac, touch.touch.yfac, touch.touch.xoff, touch.touch.yoff);
}
FINSH_FUNCTION_EXPORT(adjust_touch, touch screen calibration.)
#endif

#endif /* defined(MINISTM32_USING_TOUCH) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
