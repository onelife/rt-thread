/***************************************************************************//**
 * @file    drv_touch.h
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
#ifndef __DRV_TOUCH_H__
#define __DRV_TOUCH_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define TOUCH_TASK_STACK_SIZE           (768)
#define TOUCH_RX_MESSAGE_SIZE           (4)
#define TOUCH_RX_MESSAGE_QUEUE_SIZE     (2)
#define TOUCH_DMA_QUEUE_SIZE            (5)

#define TOUCH_INT_CLOCK                 (RCC_APB2Periph_GPIOC)
#define TOUCH_INT_PORT                  (GPIOC)
#define TOUCH_INT_PIN                   (GPIO_Pin_1)

#define TOUCH_INT_EXTI_PORT             (GPIO_PortSourceGPIOC)
#define TOUCH_INT_EXTI_PIN              (GPIO_PinSource1)
#define TOUCH_INT_EXTI_LINE             (EXTI_Line1)
#define TOUCH_INT_EXTI_IRQ              (EXTI1_IRQn)
#define TOUCH_INT_EXTI_UNIT             (1)

#define TOUCH_SAMPLE_NUMBER             (15)    /* In multiple of 3 */

#define TOUCH_CMD_READ_X                (0xD0)
#define TOUCH_CMD_READ_Y                (0x90)

#define TOUCH_REPORT_PERIOD             (RT_TICK_PER_SECOND / 10)
#define TOUCH_FIRST_MEASURE_DELAY       (RT_TICK_PER_SECOND / 10)
#define TOUCH_RIGHT_CLICK_COUNT         (4)

#define TOUCH_POINTER_STAY_LIMIT        (100)

/* Status options */
#define TOUCH_STATUS_NONBLOCKING        (1 << 0)

/* Touch screen command options */
#define TOUCH_COMMAND_MASK              (0xFF0000)
#define TOUCH_COMMAND_INTERRUPT         (0x000001)
#define TOUCH_COMMAND_TIMEOUT           (0x000002)
#define TOUCH_COMMAND_CONTROL           (0x000004)

#define TOUCH_COMMAND_WAIT_TIME         (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
struct rtgui_touch_device
{
    rt_uint8_t          stay, move;
    rt_uint8_t          calibration;
    rt_uint16_t         x, y;
    rt_uint16_t         width, height;
    volatile rt_uint16_t status;
    rt_int16_t          xoff, yoff;
    rt_uint32_t         xfac, yfac;
    rt_device_t         spi;
    struct rt_timer     timer;
};

struct miniStm32_touch_cmd_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_uint8_t          *ptr;
};

struct miniStm32_touch_ret_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_err_t            ret;
};

union miniStm32_touch_exec_message
{
    struct miniStm32_touch_cmd_message cmd;
    struct miniStm32_touch_ret_message ret;
};

struct miniStm32_touch_task_struct
{
    struct rt_thread    thread;
    struct rt_messagequeue rx_msgs;
    struct rt_event     tx_evts;
    rt_uint8_t          stack[TOUCH_TASK_STACK_SIZE];
    rt_uint8_t          rx_msg_pool[TOUCH_RX_MESSAGE_QUEUE_SIZE * \
                            (TOUCH_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_touch_unit_struct
{
    struct miniStm32_touch_task_struct task;
    struct rt_device    device;
    struct rtgui_touch_device touch;
};

struct miniStm32_touch_unit_init
{
    const rt_uint8_t    *name;
    struct miniStm32_touch_unit_struct *unit;
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_touch_init(void);

#endif /* __DRV_TOUCH_H__ */
