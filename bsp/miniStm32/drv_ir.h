/***************************************************************************//**
 * @file    drv_ir.h
 * @brief   IR remote control driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-02-13   onelife     Initial creation of IR remote control driver for
 *  MiniSTM32
 ******************************************************************************/
#ifndef __DRV_IR_H__
#define __DRV_IR_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define IR_DATA_CLOCK                   (RCC_APB2Periph_GPIOA)
#define IR_DATA_PORT                    (GPIOA)
#define IR_DATA_PIN                     (GPIO_Pin_1)

#define IR_DATA_EXTI_PORT               (GPIO_PortSourceGPIOA)
#define IR_DATA_EXTI_PIN                (GPIO_PinSource1)
#define IR_DATA_EXTI_LINE               (EXTI_Line1)
#define IR_DATA_EXTI_IRQ                (EXTI1_IRQn)
#define IR_DATA_EXTI_UNIT               (1)

#define IR_TIMER_CLOCK                  (RCC_APB1Periph_TIM2)
#define IR_TIMER_UNIT                   (TIM2)
#define IR_TIMER_NUMBER                 (2)
#define IR_TIMER_PRESCALER              (SystemCoreClock / 10000 - 1)   /* 10kHz */
#define IR_TIMER_PERIOD                 (1200)                          /* 120ms period */

#define IR_TIMER_IRQ                    (TIM2_IRQn)

/* Exported types ------------------------------------------------------------*/
struct miniStm32_ir_device
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

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t miniStm32_hw_ir_init(void);

#endif /* __DRV_IR_H__ */
