/***************************************************************************//**
 * @file 	dev_lcd.h
 * @brief 	LCD driver of RT-Thread RTOS for EFM32
 * 	COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author 	onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date			Author		Notes
 * 2012-01-24	onelife		Initial creation for EFM32 (Modified from
 *  "ssd1289.h")
 ******************************************************************************/
#ifndef __DEV_LCD_H__
#define __DEV_LCD_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//#define MINISTM32_LCD_LANDSCAPE
//#define MINISTM32_LCD_BGR
#define MINISTM32_LCD_SPEEDUP

#if defined(MINISTM32_LCD_LANDSCAPE)
#define MINISTM32_LCD_WIDTH             (320)   /* Screen Width (in pixels) */
#define MINISTM32_LCD_HEIGHT            (240)   /* Screen Hight (in pixels) */
#else
#define MINISTM32_LCD_WIDTH             (240)   /* Screen Width (in pixels) */
#define MINISTM32_LCD_HEIGHT            (320)   /* Screen Hight (in pixels) */
#endif

#if defined(MINISTM32_LCD_BGR)
#define MINISTM32_LCD_COLOR_RED         (0x001F)
#define MINISTM32_LCD_COLOR_BLUE        (0xF800)
#else
#define MINISTM32_LCD_COLOR_RED         (0xF800)
#define MINISTM32_LCD_COLOR_BLUE        (0x001F)
#endif
#define MINISTM32_LCD_COLOR_GREENE      (0x07E0)

#if defined(MINISTM32_LCD_SPEEDUP)
#define	MINISTM32_LCD_DATA_IN(data)     (*(data) = GPIOB->IDR)
#define	MINISTM32_LCD_DATA_OUT(data)    (GPIOB->ODR = (data))
#define	MINISTM32_LCD_BL_SET            (GPIOC->BSRR = GPIO_Pin_10)
#define	MINISTM32_LCD_CS_SET            (GPIOC->BSRR = GPIO_Pin_9)
#define	MINISTM32_LCD_RS_SET            (GPIOC->BSRR = GPIO_Pin_8)
#define	MINISTM32_LCD_WR_SET            (GPIOC->BSRR = GPIO_Pin_7)
#define	MINISTM32_LCD_RD_SET            (GPIOC->BSRR = GPIO_Pin_6)
#define	MINISTM32_LCD_BL_RESET          (GPIOC->BRR = GPIO_Pin_10)
#define	MINISTM32_LCD_CS_RESET          (GPIOC->BRR = GPIO_Pin_9)
#define	MINISTM32_LCD_RS_RESET          (GPIOC->BRR = GPIO_Pin_8)
#define	MINISTM32_LCD_WR_RESET          (GPIOC->BRR = GPIO_Pin_7)
#define	MINISTM32_LCD_RD_RESET          (GPIOC->BRR = GPIO_Pin_6)
#endif
#define	MINISTM32_LCD_DATA_CLOCK        (RCC_APB2Periph_GPIOB)
#define	MINISTM32_LCD_CTRL_CLOCK        (RCC_APB2Periph_GPIOC)
#define	MINISTM32_LCD_DATA_PORT         (GPIOB)
#define	MINISTM32_LCD_CTRL_PORT         (GPIOC)
#define	MINISTM32_LCD_BL_PIN            (GPIO_Pin_10)
#define	MINISTM32_LCD_CS_PIN            (GPIO_Pin_9)
#define	MINISTM32_LCD_RS_PIN            (GPIO_Pin_8)
#define	MINISTM32_LCD_WR_PIN            (GPIO_Pin_7)
#define	MINISTM32_LCD_RD_PIN            (GPIO_Pin_6)

/* Exported functions ------------------------------------------------------- */
void miniStm32_hw_lcd_init(void);

#endif /* __DEV_LCD_H__ */
