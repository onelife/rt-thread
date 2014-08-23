/***************************************************************************//**
 * @file    drv_ir.c
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

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_ir.h"
#if defined(MINISTM32_USING_IR)

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_IR_DEBUG
#define ir_debug(format,args...)            rt_kprintf(format, ##args)
#else
#define ir_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static rt_uint8_t ir_rx_state, ir_rx_cnt, ir_rx_rpt;
static rt_uint16_t ir_rx_id, ir_rx_data;

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
static void miniStm32_ir_isr(rt_device_t dev)
{
    rt_uint16_t cnt;

    TIM_Cmd(IR_TIMER_UNIT, DISABLE);
    cnt = TIM_GetCounter(IR_TIMER_UNIT);
    TIM_SetCounter(IR_TIMER_UNIT, 0x0000);
    TIM_Cmd(IR_TIMER_UNIT, ENABLE);

    if (ir_rx_state && (ir_rx_state < 5))
    {
        ir_rx_cnt++;
    }

    /* NEC like IR protocol */
    /* Start:   low 9.2ms, high 4.2ms => 13.4ms */
    /* 0 bit:   low 0.8ms, high 0.3ms =>  1.1ms */
    /* 1 bit:   low 0.8ms, high 1.4ms =>  2.2ms */
    /* Repeat:  low 8.9ms, high 2.3ms => 11.2ms */
    /* Delay before repeat:               108ms */
    switch (cnt)
    {
    case 11:
    case 12:
        switch (ir_rx_state)
        {
        case 1:
            ir_rx_id <<= 1;
            break;
        case 2:
            ir_rx_data <<= 1;
            break;
        }
        break;

    case 22:
    case 23:
        switch (ir_rx_state)
        {
        case 1:
            ir_rx_id <<= 1;
            ir_rx_id |= 0x0001;
            break;
        case 2:
            ir_rx_data <<= 1;
            ir_rx_data |= 0x0001;
            break;
        }
        break;

    case 112:
    case 113:
        if (ir_rx_state == 3)
        {
            ir_rx_rpt++;
        }
        break;

    case 134:
    case 135:
        ir_rx_state = 1;
        ir_rx_cnt = 0;
        ir_rx_rpt = 0;
        ir_rx_id = 0;
        ir_rx_data = 0;
        break;

    default:
        break;
    }

    if (ir_rx_cnt >= 16)
    {
        ir_rx_state++;
        ir_rx_cnt = 0;
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
static void miniStm32_timer2_isr(rt_device_t dev)
{
    TIM_Cmd(IR_TIMER_UNIT, DISABLE);
    TIM_SetCounter(IR_TIMER_UNIT, 0x0000);

    ir_rx_state = 0;

    if ((ir_rx_id >> 8) != (~ir_rx_id & 0x00FF))
    {
        ir_debug("IR err: RX id failed!\n");
        return;
    }
    if ((ir_rx_data >> 8) != (~ir_rx_data & 0x00FF))
    {
        ir_debug("IR err: RX data failed!\n");
        return;
    }

    ir_rx_id >>= 8;
    ir_rx_data >>= 8;

    ir_debug("IR: id %x, data %x, rpt %d\n", ir_rx_id, ir_rx_data, ir_rx_rpt);
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
rt_err_t miniStm32_hw_ir_init(void)
{
    GPIO_InitTypeDef    gpio_init;
    EXTI_InitTypeDef    exti_init;
    NVIC_InitTypeDef    nvic_init;
    TIM_TimeBaseInitTypeDef  timer_init;
    miniStm32_irq_hook_init_t hook;

    ir_rx_state = 0;

    /* Config IR data pin */
    RCC_APB2PeriphClockCmd((IR_DATA_CLOCK | RCC_APB2Periph_AFIO), ENABLE);

    gpio_init.GPIO_Pin      = IR_DATA_PIN;
    gpio_init.GPIO_Speed    = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode     = GPIO_Mode_IPU;
    GPIO_Init(IR_DATA_PORT, &gpio_init);

    /* Config external interrupt handler */
    hook.type               = miniStm32_irq_type_exti;
    hook.unit               = IR_DATA_EXTI_UNIT;
    hook.cbFunc             = miniStm32_ir_isr;
    hook.userPtr            = RT_NULL;
    miniStm32_irq_hook_register(&hook);

    /* Config external interrupt */
    GPIO_EXTILineConfig(IR_DATA_EXTI_PORT, IR_DATA_EXTI_PIN);
    EXTI_ClearFlag(IR_DATA_EXTI_LINE);
    exti_init.EXTI_Line     = IR_DATA_EXTI_LINE;
    exti_init.EXTI_Mode     = EXTI_Mode_Interrupt;
    exti_init.EXTI_Trigger  = EXTI_Trigger_Falling;
    exti_init.EXTI_LineCmd  = ENABLE;
    EXTI_Init(&exti_init);

    /* Config NVIC for external interrupt */
    NVIC_ClearPendingIRQ(IR_DATA_EXTI_IRQ);
    nvic_init.NVIC_IRQChannel = IR_DATA_EXTI_IRQ;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

	/* Config Timer2 */
	RCC_APB1PeriphClockCmd(IR_TIMER_CLOCK, ENABLE);

    TIM_TimeBaseStructInit(&timer_init);
	timer_init.TIM_Prescaler    = IR_TIMER_PRESCALER;
	timer_init.TIM_CounterMode  = TIM_CounterMode_Up;
	timer_init.TIM_Period       = IR_TIMER_PERIOD;
	TIM_TimeBaseInit(IR_TIMER_UNIT, &timer_init);
    TIM_SetCounter(IR_TIMER_UNIT, 0x0000);

    /* Config timer2 interrupt handler */
    hook.type               = miniStm32_irq_type_tim2to7;
    hook.unit               = IR_TIMER_NUMBER - 2;
    hook.cbFunc             = miniStm32_timer2_isr;
    hook.userPtr            = RT_NULL;
    miniStm32_irq_hook_register(&hook);

    /* Config timer interrupt */
	TIM_ITConfig(IR_TIMER_UNIT, TIM_IT_Update, ENABLE);

    /* Config NVIC for external interrupt */
    NVIC_ClearPendingIRQ(IR_TIMER_IRQ);
    nvic_init.NVIC_IRQChannel = IR_TIMER_IRQ;
    nvic_init.NVIC_IRQChannelPreemptionPriority = 0;
    nvic_init.NVIC_IRQChannelSubPriority = MINISTM32_IRQ_PRI_DEFAULT;
    nvic_init.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic_init);

    ir_debug("IR: HW init OK\n");
    return RT_EOK;
}

#endif /* defined(MINISTM32_USING_IR) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
