/***************************************************************************//**
 * @file    drv_camera.c
 * @brief   Camera (OV7670) driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-05-24   onelife     Initial creation of camera driver for MiniSTM32
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_camera.h"
#include "ov7670.h"
#if defined(MINISTM32_USING_CAMERA)
#include <rtgui/driver.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_CAMERA_DEBUG
#define camera_debug(format,args...)        rt_kprintf(format, ##args)
#else
#define camera_debug(format,args...)
#endif

/* Private constant ----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct miniStm32_camera_task_struct *camera_tasks[1];
static struct miniStm32_camera_unit_struct camera;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void camera_vsync_isr(rt_device_t dev)
{
    struct miniStm32_camera_device *cam;

    cam = (struct miniStm32_camera_device *)(dev->user_data);

    if (!(cam->status & CAMERA_STATUS_CAPTURING))
    {
        CAMERA_WE_SET;
        cam->status |= CAMERA_STATUS_CAPTURING;
    }
    else
    {
        rt_uint32_t temp = CAMERA_COMMAND_MASK | CAMERA_COMMAND_CAPTURE_DONE;

        CAMERA_WE_RESET;
        cam->status &= ~(rt_uint16_t)CAMERA_STATUS_CAPTURING;

        rt_mq_send(
            &camera_tasks[cam->number - 1]->rx_msgs,
            (void *)&temp,
            CAMERA_RX_MESSAGE_SIZE);
    }
}

void camera_init(struct miniStm32_camera_unit_struct *cfg)
{
    GPIO_InitTypeDef gpio_init;
    miniStm32_irq_hook_init_t hook;
    rt_uint8_t buf_ins[6], data;
    extern const struct regval_list ov7670_default_regs[];
    struct regval_list *regval = (struct regval_list *)&ov7670_default_regs;

    /* Enable clock */
    RCC_APB2PeriphClockCmd(CAMERA_DATA_CLOCK | CAMERA_CTRL_CLOCK | \
        CAMERA_SYNC_CLOCK | RCC_APB2Periph_AFIO, ENABLE);

    gpio_init.GPIO_Mode     = GPIO_Mode_IPD;
    gpio_init.GPIO_Speed    = GPIO_Speed_50MHz;
    gpio_init.GPIO_Pin      = CAMERA_DATA_PIN;
    GPIO_Init(CAMERA_DATA_PORT, &gpio_init);

    gpio_init.GPIO_Mode     = GPIO_Mode_IPD;
    gpio_init.GPIO_Pin      = CAMERA_VSYNC_PIN | CAMERA_HREF_PIN;
    GPIO_Init(CAMERA_SYNC_PORT, &gpio_init);

    gpio_init.GPIO_Mode     = GPIO_Mode_Out_PP;
    gpio_init.GPIO_Pin      = \
        CAMERA_WE_PIN | CAMERA_RRST_PIN | CAMERA_OE_PIN | CAMERA_RCK_PIN;
    GPIO_Init(CAMERA_CTRL_PORT, &gpio_init);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    CAMERA_OE_SET;
    CAMERA_WE_RESET;
    CAMERA_RRST_SET;
    CAMERA_RCK_RESET;

    /* Config external interrupt handler */
    hook.type               = miniStm32_irq_type_exti;
    hook.unit               = CAMERA_VSYNC_EXTI_UNIT;
    hook.cbFunc             = camera_vsync_isr;
    hook.userPtr            = (void *)&cfg->device;
    miniStm32_irq_hook_register(&hook);

    /* Set registers through SCCB */
    while(regval->reg_num != 0xff || regval->value != 0xff)
    {
        cfg->camera.sccb->write(cfg->camera.sccb, OV7670_I2C_ADDR, regval++, 1);
    }

    do
    {
        buf_ins[0] = REG_MIDH;
        *(rt_uint8_t **)(&buf_ins[1]) = &data;
        cfg->camera.sccb->read(cfg->camera.sccb, OV7670_I2C_ADDR, buf_ins, 1);
        if (data != MIDH_VAL)
        {
            break;
        }

        buf_ins[0] = REG_MIDL;
        cfg->camera.sccb->read(cfg->camera.sccb, OV7670_I2C_ADDR, buf_ins, 1);
        if (data != MIDL_VAL)
        {
            break;
        }

        buf_ins[0] = REG_PID;
        cfg->camera.sccb->read(cfg->camera.sccb, OV7670_I2C_ADDR, buf_ins, 1);
        if (data != PID_VAL)
        {
            break;
        }

        buf_ins[0] = REG_VER;
        cfg->camera.sccb->read(cfg->camera.sccb, OV7670_I2C_ADDR, buf_ins, 1);
        if (data != VER_VAL)
        {
            break;
        }

        camera_debug("CAM: found OV7670\n");
        return;
    } while (0);

    camera_debug("CAM err: unknown device\n");
    return;
}

static void camera_read_fifo(rt_uint8_t *buffer, rt_uint32_t size)
{
    rt_uint32_t i;

    for (i = 0; i < size; i += 2)
    {
        CAMERA_RCK_SET;
        CAMERA_DATA_IN(buffer + i + 1);
        CAMERA_RCK_RESET;
        CAMERA_RCK_SET;
        CAMERA_DATA_IN(buffer + i);
        CAMERA_RCK_RESET;
    }
}

/***************************************************************************//**
 * @brief
 *   Open camera device
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
static rt_err_t camera_task_exec(rt_uint8_t number,
    union miniStm32_camera_exec_message *exec_msg,
    rt_bool_t nonblock)
{
    rt_err_t ret;

    do
    {
        ret = rt_mq_send(
            &camera_tasks[number - 1]->rx_msgs,
            (void *)&exec_msg,
            CAMERA_RX_MESSAGE_SIZE);
        if ((ret == -RT_EFULL) && !rt_hw_interrupt_check())
        {
            rt_thread_sleep(CAMERA_COMMAND_WAIT_TIME);
            continue;
        }

        break;
    } while (1);
    if (ret != RT_EOK)
    {
        camera_debug("CAM err: send cmd failed! (%x)\n", ret);
        return ret;
    }

    if (!nonblock)
    {
        if (rt_hw_interrupt_check())
        {
            ret = rt_event_recv(
                &camera_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_NO,
                &exec_msg->ret.cmd);
        }
        else
        {
            ret = rt_event_recv(
                &camera_tasks[number - 1]->tx_evts,
                exec_msg->cmd.cmd,
                RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                RT_WAITING_FOREVER,
                &exec_msg->ret.cmd);
        }
        if (ret != RT_EOK)
        {
            camera_debug("CAM err: receive event failed!\n");
        }
    }

    return ret;
}

static void camera_task_open(
    struct miniStm32_camera_unit_struct *cfg,
   union miniStm32_camera_exec_message *exec_msg)
{
    rt_uint16_t oflag;

    if (!cfg->camera.counter)
    {
        /* Init camera */
        camera_init(cfg);

        oflag = (rt_uint16_t)exec_msg->cmd.other;
        if ((oflag & 0x0003) == RT_DEVICE_OFLAG_RDONLY)
        {
            cfg->camera.status |= CAMERA_STATUS_READ_ONLY;
        }
        if ((oflag & 0x0003) == RT_DEVICE_OFLAG_WRONLY)
        {
            exec_msg->ret.ret = RT_ERROR;
            return;
        }
        if (oflag & RT_DEVICE_OFLAG_NONBLOCKING)
        {
            cfg->camera.status |= CAMERA_STATUS_NONBLOCKING;
        }
    }
    cfg->camera.counter++;

    camera_debug("CAM: Open with flag %x\n", cfg->camera.number, oflag);
    exec_msg->ret.ret = RT_EOK;
}

static void camera_task_close(
    struct miniStm32_camera_unit_struct *cfg,
    union miniStm32_camera_exec_message *exec_msg)
{
    cfg->camera.counter--;

    exec_msg->ret.ret = RT_EOK;
}

static void camera_task_capture_start(
    struct miniStm32_camera_unit_struct *cfg,
    union miniStm32_camera_exec_message *exec_msg)
{
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    cfg->camera.status |= CAMERA_STATUS_DELAY_REPLY;
    cfg->camera.reply = (void *)exec_msg;

    /* Enable external interrupt */
    GPIO_EXTILineConfig(CAMERA_VSYNC_EXTI_PORT, CAMERA_VSYNC_EXTI_PIN);
    EXTI_ClearFlag(CAMERA_VSYNC_EXTI_LINE);
    exti_init.EXTI_Line     = CAMERA_VSYNC_EXTI_LINE;
    exti_init.EXTI_Mode     = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger  = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd  = ENABLE;
    EXTI_Init(&exti_init);

    /* Enable NVIC for external interrupt */
    NVIC_ClearPendingIRQ(CAMERA_VSYNC_EXTI_IRQ);
    nvic_init.NVIC_IRQChannel = CAMERA_VSYNC_EXTI_IRQ;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);
}

static void camera_task_capture_done(
    struct miniStm32_camera_unit_struct *cfg,
    union miniStm32_camera_exec_message *exec_msg)
{
    EXTI_InitTypeDef exti_init;
    NVIC_InitTypeDef nvic_init;

    /* Disable NVIC for external interrupt */
    NVIC_ClearPendingIRQ(CAMERA_VSYNC_EXTI_IRQ);
    nvic_init.NVIC_IRQChannel = CAMERA_VSYNC_EXTI_IRQ;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
    nvic_init.NVIC_IRQChannelCmd = DISABLE;
    NVIC_Init(&nvic_init);

    /* Disable external interrupt */
    GPIO_EXTILineConfig(CAMERA_VSYNC_EXTI_PORT, CAMERA_VSYNC_EXTI_PIN);
    EXTI_ClearFlag(CAMERA_VSYNC_EXTI_LINE);
    exti_init.EXTI_Line     = CAMERA_VSYNC_EXTI_LINE;
    exti_init.EXTI_Mode     = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger  = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd  = DISABLE;
    EXTI_Init(&exti_init);

    cfg->camera.status &= ~(rt_uint16_t)CAMERA_STATUS_DELAY_REPLY;
    exec_msg->ret.ret = RT_EOK;
}

static void camera_task_display(
    struct miniStm32_camera_unit_struct *cfg,
    union miniStm32_camera_exec_message *exec_msg)
{
    int *arg;
    rt_device_t lcd;
    struct rt_device_graphic_info lcd_info;
    const struct rtgui_graphic_driver_ops *lcd_ops;
    rt_uint16_t width, height, line_size;
    rt_uint8_t *buf;
    rt_uint32_t i, j, k;

    arg = (int *)exec_msg->cmd.ptr;

    do
    {
        lcd = rt_device_find(CAMERA_USING_DISPLAY_NAME);
        if (lcd == RT_NULL)
        {
            camera_debug("CAM err: Can't find %s\n", CAMERA_USING_DISPLAY_NAME);
            break;
        }

        if (lcd->control(lcd, RTGRAPHIC_CTRL_GET_INFO, (void *)&lcd_info) != \
            RT_EOK)
        {
            camera_debug("CAM err: Control failed!\n");
            break;
        }
        width = (lcd_info.width < CAMERA_CAPTURE_WIDTH) ? \
            lcd_info.width : CAMERA_CAPTURE_WIDTH;
        height = (lcd_info.height < CAMERA_CAPTURE_HEIGHT) ? \
            lcd_info.height : CAMERA_CAPTURE_HEIGHT;
        line_size = CAMERA_CAPTURE_WIDTH * CAMERA_CAPTURE_BYTE_PER_PIXEL;

        lcd_ops = (const struct rtgui_graphic_driver_ops *)lcd->user_data;

        if (CAMERA_BUFFER_SIZE < line_size)
        {
            exec_msg->ret.ret = RT_ENOMEM;
            break;
        }

        buf = rt_malloc(CAMERA_BUFFER_SIZE);
        if (buf == RT_NULL)
        {
            exec_msg->ret.ret = RT_ENOMEM;
            break;
        }

        /* Reset read address */
        CAMERA_OE_RESET;
        CAMERA_RRST_RESET;
        CAMERA_RCK_SET;
        CAMERA_RCK_RESET;
        CAMERA_RRST_SET;

        for (i = 0, j = 0; i < line_size * height;
            i += CAMERA_BUFFER_SIZE)
        {
            camera_read_fifo(buf, CAMERA_BUFFER_SIZE);

            for (k = 0; k < CAMERA_BUFFER_SIZE; k += line_size, j++)
            {
                lcd_ops->draw_raw_hline(buf + k, *arg, *arg + width, *(arg + 1) + j);
            }
        }
        CAMERA_OE_SET;
        rt_free(buf);
    } while (0);
}

static void camera_task_control(
    struct miniStm32_camera_unit_struct *cfg,
    union miniStm32_camera_exec_message *exec_msg)
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
        cfg->camera.status &= ~(rt_uint16_t)CAMERA_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_MODE_NONBLOCKING:
        /* Non-blocking mode operation */
        cfg->camera.status |= (rt_uint16_t)CAMERA_STATUS_NONBLOCKING;
        break;

    case RT_DEVICE_CTRL_CAMERA_CAPTURE:
        camera_task_capture_start(cfg, exec_msg);
        break;

    case RT_DEVICE_CTRL_CAMERA_DISPLAY:
        camera_task_display(cfg, exec_msg);
        break;
    }

    exec_msg->ret.ret = RT_EOK;
}

/***************************************************************************//**
* @brief
*   Camera task main loop
*
* @details
*
* @note
*
* @param[in] parameter
*   Pointer to device descriptor
******************************************************************************/
void camera_task_main_loop(void *parameter)
{
    struct miniStm32_camera_unit_struct *cfg;
    union miniStm32_camera_exec_message *exec_msg;
    rt_thread_t self;
    rt_uint32_t cmd;

    cfg = (struct miniStm32_camera_unit_struct *)parameter;
    self = rt_thread_self();

	if (rt_mq_init(
        &cfg->task.rx_msgs,
        &self->name[0],
        (void *)&cfg->task.rx_msg_pool,
        CAMERA_RX_MESSAGE_SIZE,
		sizeof(cfg->task.rx_msg_pool),
		RT_IPC_FLAG_FIFO) != RT_EOK)
	{
        rt_kprintf("CAM: init mq failed!\n");
        return;
	}

	if (rt_event_init(
        &cfg->task.tx_evts,
        &self->name[0],
        RT_IPC_FLAG_FIFO) != RT_EOK)
        {
            rt_kprintf("CAM: init event failed!\n");
            return;
        }

    /* Open SCCB device */
    if (cfg->camera.sccb->open(cfg->camera.sccb, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
    {
        rt_kprintf("CAM: open SCCB failed!\n");
        return;
    }

    cfg->device.flag |= RT_DEVICE_FLAG_ACTIVATED;

    camera_debug("CAM%d: enter main loop\n", cfg->camera.number);

CAMERA_MAIN_LOOP:
    while ((cfg->device.flag & RT_DEVICE_FLAG_ACTIVATED) && \
        !(cfg->device.flag & RT_DEVICE_FLAG_SUSPENDED))
    {
		if(rt_mq_recv(
            &cfg->task.rx_msgs,
            (void *)&exec_msg,
            CAMERA_RX_MESSAGE_SIZE,
            RT_WAITING_FOREVER) != RT_EOK)
		{
            continue;
		}

        if (((rt_uint32_t)exec_msg & CAMERA_COMMAND_MASK) == CAMERA_COMMAND_MASK)
        {
            cmd = (rt_uint32_t)exec_msg & ~(rt_uint32_t)CAMERA_COMMAND_MASK;
        }
        else
        {
            cmd = exec_msg->cmd.cmd;
        }

        switch (cmd)
        {
        case CAMERA_COMMAND_STATUS:
            exec_msg->ret.other = (rt_uint32_t)cfg->camera.status;
            break;

        case CAMERA_COMMAND_OPEN:
            camera_task_open(cfg, exec_msg);
            break;

        case CAMERA_COMMAND_CLOSE:
            camera_task_close(cfg, exec_msg);
            break;

        case CAMERA_COMMAND_CONTROL:
            camera_task_control(cfg, exec_msg);
            break;

        case CAMERA_COMMAND_CAPTURE_DONE:
            exec_msg = (union miniStm32_camera_exec_message *)cfg->camera.reply;
            cfg->camera.reply = RT_NULL;
            camera_task_capture_done(cfg, exec_msg);
            break;

        default:
            break;
        }

        if (!(cfg->camera.status & CAMERA_STATUS_DELAY_REPLY))
        {
            rt_event_send(&cfg->task.tx_evts, exec_msg->ret.cmd);
        }
    }
}

/***************************************************************************//**
 * @brief
 *   Initialize camera device
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
static rt_err_t miniStm32_camera_init (rt_device_t dev)
{
    struct miniStm32_camera_device *cam;

    cam = (struct miniStm32_camera_device *)(dev->user_data);

    return rt_thread_startup(&camera_tasks[cam->number - 1]->thread);
}

/***************************************************************************//**
 * @brief
 *   Open camera device
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
static rt_err_t miniStm32_camera_open(rt_device_t dev, rt_uint16_t oflag)
{
    struct miniStm32_camera_device *cam;
    union miniStm32_camera_exec_message exec_msg;

    cam = (struct miniStm32_camera_device *)(dev->user_data);

    exec_msg.cmd.cmd = CAMERA_COMMAND_OPEN;
    exec_msg.cmd.other = oflag;
    return camera_task_exec(cam->number, &exec_msg, RT_FALSE);
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
static rt_err_t miniStm32_camera_close(rt_device_t dev)
{
    struct miniStm32_camera_device *cam;
    union miniStm32_camera_exec_message exec_msg;

    cam = (struct miniStm32_camera_device *)(dev->user_data);

    exec_msg.cmd.cmd = CAMERA_COMMAND_CLOSE;
    return camera_task_exec(cam->number, &exec_msg, RT_FALSE);
}

/***************************************************************************//**
* @brief
*   Configure camera device
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
static rt_err_t miniStm32_camera_control (
    rt_device_t     dev,
    rt_uint8_t      cmd,
    void            *args)
{
    struct miniStm32_camera_device *cam;
    union miniStm32_camera_exec_message exec_msg;

    cam = (struct miniStm32_camera_device *)(dev->user_data);

    exec_msg.cmd.cmd = CAMERA_COMMAND_CONTROL;
    exec_msg.cmd.ptr = (rt_uint8_t *)args;
    exec_msg.cmd.other = (rt_uint32_t)cmd;
    return camera_task_exec(cam->number, &exec_msg, RT_FALSE);
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
static rt_err_t miniStm32_camera_unit_init(
    struct miniStm32_camera_unit_init_struct *init)
{
    rt_device_t                 device;
    struct miniStm32_camera_device *cam;

    do
    {
        device = &(init->unit)->device;
        cam = &(init->unit)->camera;

        /* Find SCCB device */
        cam->sccb = rt_device_find(CAMERA_USING_DEVICE_NAME);
        if (cam->sccb == RT_NULL)
        {
            camera_debug("CAM err: Can't find device %s!\n",
                CAMERA_USING_DEVICE_NAME);
            break;
        }
        camera_debug("CAM: Find device %s\n", CAMERA_USING_DEVICE_NAME);

        cam->counter            = 0;
        cam->number             = init->number;
        cam->status             = 0;
        cam->reply              = RT_NULL;

        /* Register camera device */
        device->type            = RT_Device_Class_Graphic;
        device->init            = miniStm32_camera_init;
        device->open            = miniStm32_camera_open;
        device->close           = miniStm32_camera_close;
        device->read            = RT_NULL;
        device->write           = RT_NULL;
        device->control         = miniStm32_camera_control;
        device->user_data       = (void *)cam;

        return rt_device_register(device, CAMERA_DEVICE_NAME,RT_DEVICE_FLAG_RDWR);
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
rt_err_t miniStm32_hw_camera_init(void)
{
    struct miniStm32_camera_unit_init_struct init;

    const rt_uint8_t name[] = CAMERA_DEVICE_NAME;

    do
    {
        camera_tasks[0]     = &camera.task;

        init.number         = 1;
        init.name           = &name[0];
        init.unit           = &camera;
        if (miniStm32_camera_unit_init(&init) != RT_EOK)
        {
            break;
        }

        if (rt_thread_init(
            &camera.task.thread,
            init.name,
            camera_task_main_loop,
            (void *)&camera,
            (void *)&camera.task.stack,
            sizeof(camera.task.stack),
            TASK_PRIORITY_DRIVER + 1,
            RT_TICK_PER_SECOND) != RT_EOK)
        {
            break;
        }

        camera_debug("CAM%d: HW init OK\n", init.number);
        return RT_EOK;
    } while (0);

    rt_kprintf("CAM%d: HW init failed!\n", init.number);
    return -RT_ERROR;
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>

void capture(void)
{
    union miniStm32_camera_exec_message exec_msg;
    struct {
        int x;
        int y;
    } pos = {
        0,
        0,
    };

    rt_kprintf(" Capturing...\n");

    exec_msg.cmd.cmd = CAMERA_COMMAND_OPEN;
    exec_msg.cmd.other = RT_DEVICE_OFLAG_RDONLY;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);

    exec_msg.cmd.cmd = CAMERA_COMMAND_CONTROL;
    exec_msg.cmd.other = RT_DEVICE_CTRL_CAMERA_CAPTURE;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);

    exec_msg.cmd.cmd = CAMERA_COMMAND_CONTROL;
    exec_msg.cmd.other = RT_DEVICE_CTRL_CAMERA_DISPLAY;
    exec_msg.cmd.ptr = (rt_uint8_t *)&pos;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);

    exec_msg.cmd.cmd = CAMERA_COMMAND_CLOSE;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);
    rt_kprintf(" Done.\n");
}
FINSH_FUNCTION_EXPORT(capture, capturing still image.)

rt_uint8_t record_state = 0;
rt_thread_t record_thread = RT_NULL;
void record_entry(void* parameter)
{
    union miniStm32_camera_exec_message exec_msg;
    struct {
        int x;
        int y;
    } pos = {
        0,
        0,
    };
    rt_uint8_t *record = (rt_uint8_t *)parameter;

    rt_kprintf(" REC started.\n");

    exec_msg.cmd.cmd = CAMERA_COMMAND_OPEN;
    exec_msg.cmd.other = RT_DEVICE_OFLAG_RDONLY;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);

    while (*record)
    {
        exec_msg.cmd.cmd = CAMERA_COMMAND_CONTROL;
        exec_msg.cmd.other = RT_DEVICE_CTRL_CAMERA_CAPTURE;
        camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);

        exec_msg.cmd.cmd = CAMERA_COMMAND_CONTROL;
        exec_msg.cmd.other = RT_DEVICE_CTRL_CAMERA_DISPLAY;
        exec_msg.cmd.ptr = (rt_uint8_t *)&pos;
        camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);
        rt_thread_sleep(10);
    }

    exec_msg.cmd.cmd = CAMERA_COMMAND_CLOSE;
    camera_task_exec(camera.camera.number, &exec_msg, RT_FALSE);
    rt_kprintf(" REC finished.\n");
}

void record(rt_uint8_t state)
{
    if (!(record_state ^ state))
    {
        rt_kprintf(" Done.\n");
        return;
    }
    record_state = state;

    if (record_state)
    {
        record_thread = rt_thread_create(
            "rec",
            record_entry, &record_state,
            2048,
            TASK_PRIORITY_DRIVER + 1,
            100);

        if(record_thread == RT_NULL)
        {
            rt_kprintf(" Starting failed!\n");
            return;
        }

        if (rt_thread_startup(record_thread) != RT_EOK)
        {
            rt_thread_delete(record_thread);
            rt_kprintf(" Starting failed!\n");
            return;
        }
    }
    else
    {
        rt_kprintf(" Done.\n");
    }
}
FINSH_FUNCTION_EXPORT(record, continuously capturing.)

#endif

#endif /* defined(MINISTM32_USING_TOUCH) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
