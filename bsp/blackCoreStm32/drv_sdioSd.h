/***************************************************************************//**
 * @file    drv_sdio.h
 * @brief   SDIO driver of RT-Thread RTOS
 *  COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author  onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2016-03-10   onelife     Initial creation of SDIO module driver
 ******************************************************************************/
#ifndef __DRV_SDIOSD_H__
#define __DRV_SDIOSD_H__

/* Includes ------------------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
struct board_sdioSd_device
{
    struct board_sdio_device    *sdio;
    rt_uint32_t             CSD_Tab[4];
    rt_uint32_t             CID_Tab[4];
    rt_uint32_t             RCA;
};

/* Copy and Modify From STM32 Example Code -----------------------------------*/
/** 
  * @brief SD Card information 
  */
typedef struct
{
  SD_CSD SD_csd;
  SD_CID SD_cid;
  uint32_t CardCapacity;  /*!< Card Capacity */
  uint32_t CardBlockSize; /*!< Card Block Size */
  uint16_t RCA;
  uint8_t CardType;
} SD_CardInfo;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


#endif /* __DRV_SDIOSD_H__ */
