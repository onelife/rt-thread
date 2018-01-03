/***************************************************************************//**
 * @file 	board.c
 * @brief 	Board support of RT-Thread RTOS for EFM32
 *  COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author 	onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 * LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date			Author		Notes
 * 2012-01-24	onelife		Initial creation for EFM32 (Modified from EFN32 branch)
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/***************************************************************************//**
 * @brief
 *   Configure the address of vector table
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
static void NVIC_Configuration(void)
{
#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08003000 */
	NVIC_SetVectorTable(0x08000000, 0x0);
#endif

    /* Set NVIC Preemption Priority Bits: 0 bit for pre-emption, 4 bits for
       subpriority */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_0);

    /* Set Base Priority Mask Register */
    __set_BASEPRI(BSP_BASE_PRI_DEFAULT);
}

/**
 * This is the timer interrupt service routine.
 *
 */
void rt_hw_timer_handler(void)
{
	/* enter interrupt */
	rt_interrupt_enter();

	rt_tick_increase();

	/* leave interrupt */
	rt_interrupt_leave();
}

/***************************************************************************//**
 * @brief
 *   Initialize the board.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void rt_hw_board_init(void)
{
	/* NVIC Configuration */
	NVIC_Configuration();

    /* Configure the SysTick */
    SysTick_Config( SystemCoreClock / RT_TICK_PER_SECOND );
}

/***************************************************************************//**
 * @brief
 *   Initialize the hardware drivers.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void rt_hw_driver_init(void)
{
        /* Initialize USART */
#if (defined(BSP_USING_USART1) || defined(BSP_USING_USART2) || \
    defined(BSP_USING_USART3))
    bsp_hw_usart_init();
#endif

    /* Setup Console */
    rt_console_set_device(CONSOLE_DEVICE);

#if defined(RT_USING_COMPONENTS_INIT)
    rt_components_board_init();
#else
    /* Initialize SPI */
#   if (defined(BSP_USING_SPI1) || defined(BSP_USING_SPI2) || \
    defined(BSP_USING_SPI3))
    bsp_hw_spi_init();
#   endif

    /* Initialize LED */
#   if (defined(BSP_USING_LED1) || defined(BSP_USING_LED2))
	board_hw_led_init();
#   endif

    /* Initialize OLED */
#   if defined(BSP_USING_OLED)
#       if (OLED_DEVICE_INTERFACE == INTERFACE_8BIT_80XX)
    bsp_hw_oled_init();
#       endif
#   endif

    /* Initialize RTC */
#   if defined(BSP_USING_RTC)
    bsp_hw_rtc_init();
#   endif
#endif
}

/***************************************************************************//**
 * @brief
 *   Initialize the second level drivers.
 *
 * @details
 *
 * @note
 *
 ******************************************************************************/
void rt_hw_driver2_init(void)
{
    /* Initialize OLED 2 */
#if defined(BSP_USING_OLED)
#if (OLED_DEVICE_INTERFACE == INTERFACE_4WIRE_SPI)
    bsp_hw_oled_init();
#endif
#endif

    /* Initialize SD card */
#if defined(BSP_USING_SPISD)
//    bsp_hw_spiSd_init();
#endif
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
