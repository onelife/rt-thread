/***************************************************************************//**
 * @file 	board.h
 * @brief 	Board support of RT-Thread RTOS for STM32
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
#ifndef __BOARD_H__
#define __BOARD_H__

/* Includes ------------------------------------------------------------------*/
#include <stm32f10x_conf.h>
#include <stm32f10x.h>
#include <rtthread.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define BSP_NO_DATA		                (0)
#define BSP_NO_POINTER		            (RT_NULL)
#define BSP_NO_OFFSET				    (-1)
#define BSP_NO_DMA				        (-1)

/* SECTION: SYSTEM */
#define BSP_SRAM_SIZE 			        (20 * 1024)
#define BSP_SRAM_END 				    (SRAM_BASE + BSP_SRAM_SIZE)

#define BSP_BASE_PRI_DEFAULT 		    (0x00UL)
#define BSP_IRQ_PRI_DEFAULT 		    (0x08UL)
#define BSP_IRQ_PRI_COM 		        (0x07UL)
#define BSP_IRQ_PRI_DMA 		        (0x06UL)

#define TASK_PRIORITY_DRIVER            (5)
#define TASK_PRIORITY_APPLICATION       (10)

/* SECTION: SPI */
#define SPI_RX_BUFFER_SIZE		        (64)

/* SECTION: OLED */
#if defined(BSP_USING_OLED)
#define OLED_CS_CLOCK                   (RCC_APB2Periph_GPIOB)
#define OLED_CS_PORT                    (GPIOB)
#define OLED_CS_PIN                     (GPIO_Pin_12)
#define OLED_DC_CLOCK                   (RCC_APB2Periph_GPIOB)
#define OLED_DC_PORT                    (GPIOB)
#define OLED_DC_PIN                     (GPIO_Pin_14)
#endif

/* SECTION: Micro SD */
#if defined(BSP_USING_SPISD)
#define SD_CS_CLOCK                     (RCC_APB2Periph_GPIOB)
#define SD_CS_PORT                      (GPIOB)
#define SD_CS_PIN                       (GPIO_Pin_0)
#endif

/*! fixme: The following defines should be moved to Rtdef.h */
#define RT_DEVICE_CTRL_MODE_BLOCKING        (0xF1)      /*!< Blocking mode operatrion */
#define RT_DEVICE_CTRL_MODE_NONBLOCKING     (0xF2)      /*!< Non-blocking mode operatrion */
#define RT_DEVICE_CTRL_SPI_OUTPUT_CLOCK     (0xF3)      /*!< Output SPI clock */
#define RT_DEVICE_CTRL_USART_RX_BUFFER      (0xF4)      /*!< Set USART rx buffer */
#define RTGRAPHIC_CTRL_LCD_CALIBRATION      (0xF5)      /*!< Touch screen calibration support in LCD driver */
#define RTGRAPHIC_CTRL_TOUCH_CALIBRATION    (0xF6)      /*!< Touch screen calibration support in touch screen driver */
#define RT_DEVICE_CTRL_CAMERA_CAPTURE       (0xF7)      /*!< Camera capturing a still image */
#define RT_DEVICE_CTRL_CAMERA_DISPLAY       (0xF8)      /*!< Camera displaying captured image */

#define RT_DEVICE_OFLAG_NONBLOCKING         (0x1000)
/*! fixme: The above defines should be moved to Rtdef.h */

#if (defined(BSP_USING_USART1) && (USART1_USART_MODE & BSP_USART_CONSOLE) )
#define CONSOLE_DEVICE 				    USART1_NAME
#elif (defined(BSP_USING_USART2) && (USART2_USART_MODE & BSP_USART_CONSOLE))
#define CONSOLE_DEVICE 				    USART2_NAME
#elif (defined(BSP_USING_USART3) && (USART3_USART_MODE & BSP_USART_CONSOLE))
#define CONSOLE_DEVICE 				    USART3_NAME
#else
#define CONSOLE_DEVICE 				    "NONE"
#endif

/* Exported functions ------------------------------------------------------- */
void rt_hw_board_init(void);
void rt_hw_driver_init(void);
void rt_hw_driver2_init(void);
rt_uint32_t rt_hw_interrupt_check(void);

#endif /*__BOARD_H__ */
