/***************************************************************************//**
 * @file    drv_camera.h
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
#ifndef __DRV_CAMERA_H__
#define __DRV_CAMERA_H__

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define CAMERA_TASK_STACK_SIZE          (768)
#define CAMERA_RX_MESSAGE_SIZE          (4)
#define CAMERA_RX_MESSAGE_QUEUE_SIZE    (2)
#define CAMERA_DMA_QUEUE_SIZE           (5)

#define	CAMERA_CAPTURE_WIDTH            QVGA_WIDTH
#define	CAMERA_CAPTURE_HEIGHT           QVGA_HEIGHT
#define	CAMERA_CAPTURE_BYTE_PER_PIXEL   (2)
#define	CAMERA_BUFFER_SIZE              (CAMERA_CAPTURE_WIDTH * CAMERA_CAPTURE_BYTE_PER_PIXEL)

#define	CAMERA_DATA_CLOCK               (RCC_APB2Periph_GPIOA)
#define	CAMERA_DATA_PORT                (GPIOA)
#define	CAMERA_DATA_PIN                 (GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7)
#define	CAMERA_CTRL_CLOCK               (RCC_APB2Periph_GPIOA)
#define	CAMERA_CTRL_PORT                (GPIOA)
#define CAMERA_OE_PIN                   (GPIO_Pin_8)
#define CAMERA_RRST_PIN                 (GPIO_Pin_11)
#define CAMERA_WE_PIN                   (GPIO_Pin_12)
#define CAMERA_RCK_PIN                  (GPIO_Pin_15)
#define	CAMERA_SYNC_CLOCK               (RCC_APB2Periph_GPIOC)
#define	CAMERA_SYNC_PORT                (GPIOC)
#define CAMERA_VSYNC_PIN                (GPIO_Pin_4)
#define CAMERA_HREF_PIN                 (GPIO_Pin_5)

#define CAMERA_VSYNC_EXTI_PORT          (GPIO_PortSourceGPIOC)
#define CAMERA_VSYNC_EXTI_PIN           (GPIO_PinSource4)
#define CAMERA_VSYNC_EXTI_LINE          (EXTI_Line4)
#define CAMERA_VSYNC_EXTI_IRQ           (EXTI4_IRQn)
#define CAMERA_VSYNC_EXTI_UNIT          (4)

#define	CAMERA_DATA_IN(data)            (*(data) = CAMERA_DATA_PORT->IDR & 0x000000FF)
#define	CAMERA_OE_SET                   (CAMERA_CTRL_PORT->BSRR = GPIO_Pin_8)
#define	CAMERA_OE_RESET                 (CAMERA_CTRL_PORT->BRR = GPIO_Pin_8)
#define	CAMERA_RRST_SET                 (CAMERA_CTRL_PORT->BSRR = GPIO_Pin_11)
#define	CAMERA_RRST_RESET               (CAMERA_CTRL_PORT->BRR = GPIO_Pin_11)
#define	CAMERA_WE_SET                   (CAMERA_CTRL_PORT->BSRR = GPIO_Pin_12)
#define	CAMERA_WE_RESET                 (CAMERA_CTRL_PORT->BRR = GPIO_Pin_12)
#define	CAMERA_RCK_SET                  (CAMERA_CTRL_PORT->BSRR = GPIO_Pin_15)
#define	CAMERA_RCK_RESET                (CAMERA_CTRL_PORT->BRR = GPIO_Pin_15)

/* Status options */
#define CAMERA_STATUS_MASK              (0x01)
#define CAMERA_STATUS_MASTER            (1 << 0)
#define CAMERA_STATUS_READ_ONLY         (1 << 4)
#define CAMERA_STATUS_WRITE_ONLY        (1 << 5)
#define CAMERA_STATUS_NONBLOCKING       (1 << 6)
#define CAMERA_STATUS_CAPTURING         (1 << 7)
#define CAMERA_STATUS_DELAY_REPLY       (1 << 8)

/* Camera command options */
#define CAMERA_COMMAND_MASK             (0xFF0000)
#define CAMERA_COMMAND_STATUS           (0x000001)
#define CAMERA_COMMAND_OPEN             (0x000002)
#define CAMERA_COMMAND_CLOSE            (0x000004)
#define CAMERA_COMMAND_READ             (0x000008)
#define CAMERA_COMMAND_WRITE            (0x000010)
#define CAMERA_COMMAND_CONTROL          (0x000020)
#define CAMERA_COMMAND_CAPTURE_DONE     (0x000100)

#define CAMERA_COMMAND_WAIT_TIME        (RT_TICK_PER_SECOND / 10)

/* Exported types ------------------------------------------------------------*/
struct miniStm32_camera_device
{
    rt_uint8_t          counter;
    rt_uint8_t          number;
    volatile rt_uint16_t status;
    rt_device_t         sccb;
    void                *reply;
};

struct miniStm32_camera_cmd_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_uint8_t          *ptr;
};

struct miniStm32_camera_ret_message
{
    rt_uint32_t         cmd;
    rt_uint32_t         other;
    rt_size_t           size;
    rt_err_t            ret;
};

union miniStm32_camera_exec_message
{
    struct miniStm32_camera_cmd_message cmd;
    struct miniStm32_camera_ret_message ret;
};

struct miniStm32_camera_task_struct
{
    struct rt_thread    thread;
    struct rt_messagequeue rx_msgs;
    struct rt_event     tx_evts;
    rt_uint8_t          stack[CAMERA_TASK_STACK_SIZE];
    rt_uint8_t          rx_msg_pool[CAMERA_RX_MESSAGE_QUEUE_SIZE * \
                            (CAMERA_RX_MESSAGE_SIZE + 4)];
};

struct miniStm32_camera_unit_struct
{
    struct miniStm32_camera_task_struct task;
    struct rt_device    device;
    struct miniStm32_camera_device camera;
};

struct miniStm32_camera_unit_init_struct
{
    rt_uint8_t          number;
    const rt_uint8_t    *name;
    struct miniStm32_camera_unit_struct *unit;

};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_camera_init(void);

#endif /* __DRV_CAMERA_H__ */
