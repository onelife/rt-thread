/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_conf.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Device Firmware Upgrade (DFU) configuration file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF_H
#define __USB_CONF_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* These timings will be returned to the host when it checks the device
   status during a write or erase operation to know how much time the host
   should wait before issuing the next get status request.
   These defines are set in usb_conf.h file.
   The values of this table should be extracted from relative memories
   datasheet (Typical or Maximum timming value for Sector Erase and for
   1024 bytes Write). All timings are expressed in millisecond unit (ms).
   Note that "Sector" refers here to the memory unit used for Erase/Write
   operations. It could be a sector, a page, a block, a word ...
   If the erase operation is not supported, it is advised to set the erase
   timing to 1 (which means 1ms: one USB frame). */
#define SPI_FLASH_SECTOR_ERASE_TIME     3000
#define SPI_FLASH_SECTOR_WRITE_TIME     20

#define INTERN_FLASH_SECTOR_ERASE_TIME 50
#define INTERN_FLASH_SECTOR_WRITE_TIME 50

#define M29W128F_SECTOR_ERASE_TIME      1000
#define M29W128F_SECTOR_WRITE_TIME      25

#define M29W128G_SECTOR_ERASE_TIME      1000
#define M29W128G_SECTOR_WRITE_TIME      25

#define S29GL128_SECTOR_ERASE_TIME      1000
#define S29GL128_SECTOR_WRITE_TIME      45

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/
#define ENDPOINT_NUMBER                 (1)

/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/
/* buffer table base address */
/* buffer table base address */
#define BTABLE_ADDRESS      (0x00)

/* EP0 */
/* rx/tx buffer base address */
#define ENDP0_RXADDR        (0x10)
#define ENDP0_TXADDR        (0x50)


/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM  | \
                 CNTR_WKUPM | \
                 CNTR_SUSPM | \
                 CNTR_ERRM  | \
                 CNTR_SOFM  | \
                 CNTR_ESOFM | \
                 CNTR_RESETM  \
                )

/* CTR service routines */
/* associated to defined endpoints */
#define  EP1_IN_Callback   NOP_Process
#define  EP2_IN_Callback   NOP_Process
#define  EP3_IN_Callback   NOP_Process
#define  EP4_IN_Callback   NOP_Process
#define  EP5_IN_Callback   NOP_Process
#define  EP6_IN_Callback   NOP_Process
#define  EP7_IN_Callback   NOP_Process


#define  EP1_OUT_Callback   NOP_Process
#define  EP2_OUT_Callback   NOP_Process
#define  EP3_OUT_Callback   NOP_Process
#define  EP4_OUT_Callback   NOP_Process
#define  EP5_OUT_Callback   NOP_Process
#define  EP6_OUT_Callback   NOP_Process
#define  EP7_OUT_Callback   NOP_Process

#endif /*__USB_CONF_H*/

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/




