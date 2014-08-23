/***************************************************************************//**
 * @file 	dev_dou.h
 * @brief 	DOU (which is a virtual display with mouse input) driver of
 *  RT-Thread RTOS
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
 * 2012-07-04	onelife		Initial creation for MiniSTM32
 ******************************************************************************/
#ifndef __DEV_DOU_H__
#define __DEV_DOU_H__

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#define DOU_VERSION                     (1)
#define DOU_SUBVERSION                  (0)

//#define DOU_MODE_GRB

#define DOU_SCREEN_WIDTH                (320)   /* Screen Width (in pixels) */
#define DOU_SCREEN_HEIGHT               (240)   /* Screen Hight (in pixels) */
#define DOU_COLOR_DEPTH                 (16)

#if (DOU_COLOR_DEPTH == 24)
 #if defined(DOU_MODE_GRB)
 #define DOU_COLOR_RED                  (0x000000FF)
 #define MDOU_COLOR_BLUE                (0x00FF0000)
 #else
 #define DOU_COLOR_RED                  (0x00FF0000)
 #define DOU_COLOR_BLUE                 (0x000000FF)
 #endif
 #define DOU_COLOR_GREENE               (0x0000FF00)
 typedef unsigned long                  dou_color_t;
#elif (DOU_COLOR_DEPTH == 16)
 #if defined(DOU_MODE_GRB)
 #define DOU_COLOR_RED                  (0x001F)
 #define MDOU_COLOR_BLUE                (0xF800)
 #else
 #define DOU_COLOR_RED                  (0xF800)
 #define DOU_COLOR_BLUE                 (0x001F)
 #endif
 #define DOU_COLOR_GREENE               (0x07E0)
 typedef unsigned short                  dou_color_t;
#endif

#define DOU_COMMAND_START               (0x5AD0)
#define DOU_COMMAND_END                 (0xD0A5)

#define DOU_HEADER_COMMAND              (0xD1)
#define DOU_HEADER_DATA                 (0xD2)

#define DOU_COMMAND_INIT                ('1')
#define DOU_COMMAND_CLEAR               ('2')
#define DOU_COMMAND_READ_POINT          ('3')
#define DOU_COMMAND_DRAW_POINT          ('4')
#define DOU_COMMAND_DRAW_CHLINE         ('5')
#define DOU_COMMAND_DRAW_HLINE          ('6')
#define DOU_COMMAND_DRAW_VLINE          ('7')

/* Exported functions ------------------------------------------------------- */
void miniStm32_hw_dou_init(void);

#endif /* __DEV_DOU_H__ */
