/***************************************************************************//**
 * @file 	drv_sdcard.h
 * @brief   Memory card driver (SPI mode) of RT-Thread RTOS for MiniSTM32
 * 	COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author 	onelife
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date			Author		Notes
 * 2012-01-24   onelife     Initial creation of SD card driver (through SPI) for
 *  MiniSTM32 (Modified from EFN32 branch)
 ******************************************************************************/
#ifndef __DRV_SDCARD_H__
#define __DRV_SDCARD_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define BSP_SDCLK_LOW           (SPI_BaudRatePrescaler_64)
#define BSP_SDCLK_HIGH          (SPI_BaudRatePrescaler_2)

#define SD_INIT_RETRY_TIMES     (5)

#define SD_SPEED_LOW 			(0)
#define SD_SPEED_HIGH 			(1)
#define SD_WAIT_PERIOD 			(RT_TICK_PER_SECOND)

#define SD_SECTOR_SIZE_SHIFT	(9)         /* 512 bytes is always supported */
#define SD_SECTOR_SIZE 			(1 << SD_SECTOR_SIZE_SHIFT)
#define SD_BLOCK_SIZE_CSD 		(16)
#define SD_BLOCK_SIZE_CID 		(16)
#define SD_BLOCK_SIZE_OCR 		(4)
#define SD_BLOCK_SIZE_SDSTAT 	(64)

/* Card type definitions (CardType) */
#define CT_MMC					(0x01)
#define CT_SD1					(0x02)
#define CT_SD2					(0x04)
#define CT_SDC					(CT_SD1|CT_SD2)
#define CT_BLOCK				(0x08)

/* Definitions for MMC/SDC command */
#define CMD0 					(0) 		/* GO_IDLE_STATE */
#define CMD1 					(1) 		/* SEND_OP_COND */
#define ACMD41 					(41|0x80) 	/* SEND_OP_COND (SDC) */
#define CMD8 					(8) 		/* SEND_IF_COND */
#define CMD9 					(9) 		/* SEND_CSD */
#define CMD10 					(10) 		/* SEND_CID */
#define CMD12 					(12) 		/* STOP_TRANSMISSION */
#define ACMD13 					(13|0x80) 	/* SD_STATUS (SDC) */
#define CMD16 					(16) 		/* SET_BLOCKLEN */
#define CMD17 					(17) 		/* READ_SINGLE_BLOCK */
#define CMD18 					(18) 		/* READ_MULTIPLE_BLOCK */
#define CMD23 					(23) 		/* SET_BLOCK_COUNT */
#define ACMD23 					(23|0x80) 	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24 					(24) 		/* WRITE_BLOCK */
#define CMD25 					(25) 		/* WRITE_MULTIPLE_BLOCK */
#define CMD41 					(41) 		/* SEND_OP_COND (ACMD) */
#define CMD55 					(55) 		/* APP_CMD */
#define CMD58 					(58) 		/* READ_OCR */

/* Exported types ------------------------------------------------------------*/
struct sd_register_cid
{
    rt_uint8_t                  man_id;
    rt_uint8_t                  app_id[2];
    rt_uint8_t                  name[5];
    rt_uint8_t                  rev;
    rt_uint8_t                  sn[4];
    rt_uint8_t                  date[2];
    rt_uint8_t                  crc;
};

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
rt_err_t bsp_hw_spiSd_init(void);

#endif /* __DRV_SDCARD_H__ */
