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
    __set_BASEPRI(MINISTM32_BASE_PRI_DEFAULT);
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
#if (defined(MINISTM32_USING_USART1) || defined(MINISTM32_USING_USART2) || \
    defined(MINISTM32_USING_USART3))
    miniStm32_hw_usart_init();
#endif

	/* Setup Console */
    rt_console_set_device(CONSOLE_DEVICE);

    /* Initialize SPI */
#if (defined(MINISTM32_USING_SPI1) || defined(MINISTM32_USING_SPI2) || \
    defined(MINISTM32_USING_SPI3))
    miniStm32_hw_spi_init();
#endif

    /* Initialize GPIO SPI */
#if (defined(MINISTM32_USING_GPIO_SPI1) || defined(MINISTM32_USING_GPIO_SPI2))
    miniStm32_hw_gpio_spi_init();
#endif

    /* Initialize GPIO SCCB */
#if defined(MINISTM32_USING_GPIO_SCCB)
    miniStm32_hw_gpio_sccb_init();
#endif

    /* Initialize LCD */
#if defined(MINISTM32_USING_LCD)
    miniStm32_hw_lcd_init();
#endif

    /* Initialize OLED */
#if defined(MINISTM32_USING_OLED)
    miniStm32_hw_oled_init();
#endif

    /* Initialize RTC */
#if defined(MINISTM32_USING_RTC)
    miniStm32_hw_rtc_init();
#endif

    /* Initialize USB */
#if defined(MINISTM32_USING_USB)
    miniStm32_hw_usb_core_init();

 #if defined(MINISTM32_USING_USB_VIRTUAL_COM)
    miniStm32_hw_usb_com_init();
 #endif
 #if defined(MINISTM32_USING_USB_HID_MOUSE)
    miniStm32_hw_usb_mouse_init();
 #endif
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
    /* Initialize SD card */
#if defined(MINISTM32_USING_SPISD)
//    miniStm32_hw_spiSd_init();
#endif

    /* Initialize touch screen */
#if defined(MINISTM32_USING_TOUCH)
    miniStm32_hw_touch_init();
#endif

#if defined(MINISTM32_USING_IR)
    miniStm32_hw_ir_init();
#endif

#if defined(MINISTM32_USING_CAMERA)
    miniStm32_hw_camera_init();
#endif

    /* Initialize DOU */
#if defined(MINISTM32_USING_DOU)
    miniStm32_hw_dou_init();
#endif
}

/***************************************************************************//**
 * @}
 ******************************************************************************/
