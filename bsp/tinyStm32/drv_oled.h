/***************************************************************************//**
 * @file 	dev_oled.h
 * @brief 	OLED driver of RT-Thread RTOS for MiniSTM32
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
 * 2012-06-15	onelife		Initial creation for MiniSTM32
 ******************************************************************************/
#ifndef __DEV_OLED_H__
#define __DEV_OLED_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define MINISTM32_OLED_WIDTH            (128)   /* Screen Width (in pixels) */
#define MINISTM32_OLED_HEIGHT           (64)    /* Screen Hight (in pixels) */

#define MINISTM32_OLED_CTRL_SUSPEND     (0x00)
#define MINISTM32_OLED_CTRL_RESUME      (0x01)

#define	MINISTM32_OLED_DATA_IN(data)    (*(data) = GPIOB->IDR & 0x000000FF)
#define	MINISTM32_OLED_DATA_OUT(data)   (GPIOB->ODR = (GPIOB->ODR & 0xFFFFFF00) | data)
#define	MINISTM32_OLED_CS_SET           (GPIOC->BSRR = GPIO_Pin_9)
#define	MINISTM32_OLED_DC_SET           (GPIOC->BSRR = GPIO_Pin_8)
#define	MINISTM32_OLED_WR_SET           (GPIOC->BSRR = GPIO_Pin_7)
#define	MINISTM32_OLED_RD_SET           (GPIOC->BSRR = GPIO_Pin_6)
#define	MINISTM32_OLED_CS_RESET         (GPIOC->BRR = GPIO_Pin_9)
#define	MINISTM32_OLED_DC_RESET         (GPIOC->BRR = GPIO_Pin_8)
#define	MINISTM32_OLED_WR_RESET         (GPIOC->BRR = GPIO_Pin_7)
#define	MINISTM32_OLED_RD_RESET         (GPIOC->BRR = GPIO_Pin_6)

#define	MINISTM32_OLED_DATA_CLOCK       (RCC_APB2Periph_GPIOB)
#define	MINISTM32_OLED_CTRL_CLOCK       (RCC_APB2Periph_GPIOC)
#define	MINISTM32_OLED_DATA_PORT        (GPIOB)
#define	MINISTM32_OLED_CTRL_PORT        (GPIOC)
#define	MINISTM32_OLED_CS_PIN           (GPIO_Pin_9)
#define	MINISTM32_OLED_DC_PIN           (GPIO_Pin_8)
#define	MINISTM32_OLED_WR_PIN           (GPIO_Pin_7)
#define	MINISTM32_OLED_RD_PIN           (GPIO_Pin_6)

/* Exported functions ------------------------------------------------------- */
void miniStm32_hw_oled_init(void);

#endif /* __DEV_OLED_H__ */
