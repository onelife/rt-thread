/***************************************************************************//**
 * @file    drv_rtc.h
 * @brief   RTC driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2012, RT-Thread Development Team
 * @author  onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-02-18   onelife     Initial creation of RTC driver for MiniSTM32
 ******************************************************************************/
#ifndef __DRV_RTC_H__
#define __DRV_RTC_H__

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_rtc_init(void);

#endif /* __DRV_RTC_H__ */
