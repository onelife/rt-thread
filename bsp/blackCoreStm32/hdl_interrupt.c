/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <board.h>
#include <rtthread.h>
#include "hdl_interrupt.h"

/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifdef RT_IRQHDL_DEBUG
#define hdl_debug(format,args...) 			rt_kprintf(format, ##args)
#else
#define hdl_debug(format,args...)
#endif

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
miniStm32_irq_hook_t rtcCbTable[1]      = {RT_NULL};
miniStm32_irq_hook_t extiCbTable[19]    = {RT_NULL};
miniStm32_irq_hook_t dmaCbTable[12]     = {RT_NULL};
miniStm32_irq_hook_t usbCbTable[2]      = {RT_NULL};
miniStm32_irq_hook_t tim2to7CbTable[6]  = {RT_NULL};
miniStm32_irq_hook_t spiCbTable[3]      = {RT_NULL};
miniStm32_irq_hook_t usartCbTable[3]    = {RT_NULL};
miniStm32_irq_hook_t sdioCbTable[1]     = {RT_NULL};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
    extern void rt_hw_timer_handler(void);
    rt_hw_timer_handler();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
void RTC_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (rtcCbTable[0].cbFunc != RT_NULL)
    {
        (rtcCbTable[0].cbFunc)(rtcCbTable[0].userPtr);
    }

    /* clear all flags */
    RTC_ClearFlag(RTC_FLAG_OW);

    /* leave interrupt */
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (extiCbTable[1].cbFunc != RT_NULL)
    {
        (extiCbTable[1].cbFunc)(extiCbTable[1].userPtr);
    }

    /* clear all flags */
    EXTI_ClearFlag(EXTI_Line1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (extiCbTable[4].cbFunc != RT_NULL)
    {
        (extiCbTable[4].cbFunc)(extiCbTable[4].userPtr);
    }

    /* clear all flags */
    EXTI_ClearFlag(EXTI_Line4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void EXTI9_5_IRQHandler(void)
{
    rt_uint8_t i;

    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    for (i = 5; i <= 9; i++)
    {
        if ((EXTI->PR & (1 << i)) && (extiCbTable[i].cbFunc != RT_NULL))
        {
            (extiCbTable[i].cbFunc)(extiCbTable[i].userPtr);
        }
    }

    /* clear all flags */
    EXTI_ClearFlag(EXTI_Line5 | EXTI_Line6 | EXTI_Line7 | EXTI_Line8 | \
        EXTI_Line9);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[0].cbFunc != RT_NULL)
    {
        (dmaCbTable[0].cbFunc)(dmaCbTable[0].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL1);

    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : DMA1_Channel2_IRQHandler
* Description    : This function handles DMA1 Channel 2 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void DMA1_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[1].cbFunc != RT_NULL)
    {
        (dmaCbTable[1].cbFunc)(dmaCbTable[1].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[2].cbFunc != RT_NULL)
    {
        (dmaCbTable[2].cbFunc)(dmaCbTable[2].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[3].cbFunc != RT_NULL)
    {
        (dmaCbTable[3].cbFunc)(dmaCbTable[3].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL4);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[4].cbFunc != RT_NULL)
    {
        (dmaCbTable[4].cbFunc)(dmaCbTable[4].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL5);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[5].cbFunc != RT_NULL)
    {
        (dmaCbTable[5].cbFunc)(dmaCbTable[5].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL6);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA1_Channel7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[6].cbFunc != RT_NULL)
    {
        (dmaCbTable[6].cbFunc)(dmaCbTable[6].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA1_FLAG_GL7);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[7].cbFunc != RT_NULL)
    {
        (dmaCbTable[7].cbFunc)(dmaCbTable[7].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA2_FLAG_GL1);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[8].cbFunc != RT_NULL)
    {
        (dmaCbTable[8].cbFunc)(dmaCbTable[8].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA2_FLAG_GL2);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (dmaCbTable[9].cbFunc != RT_NULL)
    {
        (dmaCbTable[9].cbFunc)(dmaCbTable[9].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA2_FLAG_GL3);

    /* leave interrupt */
    rt_interrupt_leave();
}

void DMA2_Channel4_5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (DMA_GetFlagStatus(DMA2_FLAG_TC4) && (dmaCbTable[10].cbFunc != RT_NULL))
    {
        (dmaCbTable[10].cbFunc)(dmaCbTable[10].userPtr);
    }
    if (DMA_GetFlagStatus(DMA2_FLAG_TC5) && (dmaCbTable[11].cbFunc != RT_NULL))
    {
        (dmaCbTable[11].cbFunc)(dmaCbTable[11].userPtr);
    }

    /* clear all flags */
    DMA_ClearFlag(DMA2_FLAG_GL4);
    DMA_ClearFlag(DMA2_FLAG_GL5);

    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : USB_LP_CAN1_RX0_IRQHandler
* Description    : This function handles USB Low Priority or CAN RX0 interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_CAN1_RX0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	/* invoke callback function */
	if (usbCbTable[0].cbFunc != RT_NULL)
	{
		(usbCbTable[0].cbFunc)(usbCbTable[0].userPtr);
	}

    /* clear all flags */
    //USART_ClearFlag(USART1, USART_FLAG_RXNE);
    // TODO: clear other flags

    /* leave interrupt */
    rt_interrupt_leave();
}

void USBWakeUp_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (extiCbTable[18].cbFunc != RT_NULL)
    {
        (extiCbTable[18].cbFunc)(extiCbTable[18].userPtr);
    }

    /* clear all flags */
    EXTI_ClearFlag(EXTI_Line18);

    /* leave interrupt */
    rt_interrupt_leave();
}


void TIM2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* invoke callback function */
    if (tim2to7CbTable[0].cbFunc != RT_NULL)
    {
        (tim2to7CbTable[0].cbFunc)(tim2to7CbTable[0].userPtr);
    }

    /* clear all flags */
    TIM_ClearFlag(TIM2, 0x0000);

    /* leave interrupt */
    rt_interrupt_leave();
}

void SPI1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	/* invoke callback function */
	if (spiCbTable[0].cbFunc != RT_NULL)
	{
		(spiCbTable[0].cbFunc)(spiCbTable[0].userPtr);
	}

    /* clear all flags */
    // TODO: clear error flag

    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : USART1_IRQHandler
* Description    : This function handles USART1 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	/* invoke callback function */
	if (usartCbTable[0].cbFunc != RT_NULL)
	{
		(usartCbTable[0].cbFunc)(usartCbTable[0].userPtr);
	}

    /* clear all flags */
    USART_ClearFlag(USART1, USART_FLAG_RXNE);
    // TODO: clear other flags

    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : USART2_IRQHandler
* Description    : This function handles USART2 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	/* invoke callback function */
	if (usartCbTable[1].cbFunc != RT_NULL)
	{
		(usartCbTable[1].cbFunc)(usartCbTable[1].userPtr);
	}

    /* clear all flags */
    USART_ClearFlag(USART2, USART_FLAG_RXNE);
    // TODO: clear other flags

    /* leave interrupt */
    rt_interrupt_leave();
}

/*******************************************************************************
* Function Name  : USART3_IRQHandler
* Description    : This function handles USART3 global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

	/* invoke callback function */
	if (usartCbTable[2].cbFunc != RT_NULL)
	{
		(usartCbTable[2].cbFunc)(usartCbTable[2].userPtr);
	}

    /* clear all flags */
    USART_ClearFlag(USART3, USART_FLAG_RXNE);
    // TODO: clear other flags

    /* leave interrupt */
    rt_interrupt_leave();
}

#if defined(RT_USING_DFS) && STM32_USE_SDIO
/*******************************************************************************
* Function Name  : SDIO_IRQHandler
* Description    : This function handles SDIO global interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void SDIO_IRQHandler(void)
{
    extern int SD_ProcessIRQSrc(void);

    /* enter interrupt */
    rt_interrupt_enter();

    /* Process All SDIO Interrupt Sources */
    SD_ProcessIRQSrc();

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#ifdef RT_USING_LWIP
#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : ETH_IRQHandler
* Description    : This function handles ETH interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void ETH_IRQHandler(void)
{
	extern void rt_hw_stm32_eth_isr(void);

    /* enter interrupt */
    rt_interrupt_enter();

	rt_hw_stm32_eth_isr();

    /* leave interrupt */
    rt_interrupt_leave();
}
#else
#if (STM32_ETH_IF == 0)
/*******************************************************************************
* Function Name  : EXTI0_IRQHandler
* Description    : This function handles External interrupt Line 0 request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI2_IRQHandler(void)
{
    extern void enc28j60_isr(void);

    /* enter interrupt */
    rt_interrupt_enter();

    enc28j60_isr();

    /* Clear the Key Button EXTI line pending bit */
    EXTI_ClearITPendingBit(EXTI_Line2);

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif

#if (STM32_ETH_IF == 1)
/*******************************************************************************
* Function Name  : EXTI4_IRQHandler
* Description    : This function handles External lines 9 to 5 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void EXTI4_IRQHandler(void)
{
	extern void rt_dm9000_isr(void);

	/* enter interrupt */
	rt_interrupt_enter();

	/* Clear the DM9000A EXTI line pending bit */
	EXTI_ClearITPendingBit(EXTI_Line4);

	rt_dm9000_isr();

	/* leave interrupt */
	rt_interrupt_leave();
}
#endif
#endif
#endif /* end of RT_USING_LWIP */

/**
  * @}
  */

/***************************************************************************//**
 * @brief
 * 	EFM32 common interrupt handlers register function
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void miniStm32_irq_hook_register(miniStm32_irq_hook_init_t *hook)
{
	switch (hook->type)
	{
    case miniStm32_irq_type_rtc:
        rtcCbTable[hook->unit].cbFunc = hook->cbFunc;
        rtcCbTable[hook->unit].userPtr = hook->userPtr;
        break;

    case miniStm32_irq_type_exti:
        extiCbTable[hook->unit].cbFunc = hook->cbFunc;
        extiCbTable[hook->unit].userPtr = hook->userPtr;
        break;

    case miniStm32_irq_type_dma:
        dmaCbTable[hook->unit].cbFunc = hook->cbFunc;
        dmaCbTable[hook->unit].userPtr = hook->userPtr;
        break;

    case miniStm32_irq_type_usb:
        usbCbTable[hook->unit].cbFunc = hook->cbFunc;
        usbCbTable[hook->unit].userPtr = hook->userPtr;
        break;

	case miniStm32_irq_type_tim2to7:
		tim2to7CbTable[hook->unit].cbFunc = hook->cbFunc;
		tim2to7CbTable[hook->unit].userPtr = hook->userPtr;
		break;

	case miniStm32_irq_type_spi:
		spiCbTable[hook->unit].cbFunc = hook->cbFunc;
		spiCbTable[hook->unit].userPtr = hook->userPtr;
		break;

	case miniStm32_irq_type_usart:
		usartCbTable[hook->unit].cbFunc = hook->cbFunc;
		usartCbTable[hook->unit].userPtr = hook->userPtr;
		break;
        
	case board_irq_type_sdio:
		sdioCbTable[hook->unit].cbFunc = hook->cbFunc;
		sdioCbTable[hook->unit].userPtr = hook->userPtr;
		break;

	default:
		break;
	}

	hdl_debug("Hook Registered: type: %s, unit: %x, cbFunc: %x, userPtr: %x\n", \
		hook->type, hook->unit, hook->cbFunc, hook->userPtr);
}


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
