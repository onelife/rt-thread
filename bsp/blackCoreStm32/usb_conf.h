/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_conf.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Virtual COM Port Demo configuration  header
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
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* External variables --------------------------------------------------------*/
#define VER_USB                     (0x0200)    // USB specification revision
#define VID                         (0x0483)    // Vendor ID
#if defined(BSP_USING_USB_VIRTUAL_COM)
#define PID                         (0x5740)    // Product ID
#endif
#if defined(BSP_USING_USB_HID_MOUSE)
#define PID                         (0x5710)    // Product ID
#endif
#define DEV_REV                     (0x0200)    // Device Release number

/*-------------------------------------------------------------*/
/* EP_NUM */
/* defines how many endpoints are used by the device */
/*-------------------------------------------------------------*/

#define ENDPOINT_NUMBER             (8)
#define INTERFACE_NUMBER            (2)
#define DEVICE_NUMBER               (8)


/*-------------------------------------------------------------*/
/* --------------   Buffer Description Table  -----------------*/
/*-------------------------------------------------------------*/
/* buffer table base address */
/* buffer table base address */
#define USB_ENDP0_TX_BUF_SIZE       (0x40)
#define USB_ENDP0_RX_BUF_SIZE       (0x40)
/*#define USB_ENDP1_TX_BUF_SIZE       (0x40)
#define USB_ENDP1_RX_BUF_SIZE       (0x40)
#define USB_ENDP2_TX_BUF_SIZE       (0x10)
#define USB_ENDP2_RX_BUF_SIZE       (0x00)
#define USB_ENDP3_TX_BUF_SIZE       (0x00)
#define USB_ENDP3_RX_BUF_SIZE       (0x00)
#define USB_ENDP4_TX_BUF_SIZE       (0x00)
#define USB_ENDP4_RX_BUF_SIZE       (0x00)
#define USB_ENDP5_TX_BUF_SIZE       (0x00)
#define USB_ENDP5_RX_BUF_SIZE       (0x00)
#define USB_ENDP6_TX_BUF_SIZE       (0x00)
#define USB_ENDP6_RX_BUF_SIZE       (0x00)
#define USB_ENDP7_TX_BUF_SIZE       (0x00)
#define USB_ENDP7_RX_BUF_SIZE       (0x00)*/
#define USB_BUF_TABLE_SIZE          (0x40)

#define USB_BUF_TABLE_ADDR          (0x00)
#define USB_ENDP0_TX_BUF_ADDR       (USB_BUF_TABLE_ADDR + USB_BUF_TABLE_SIZE)
#define USB_ENDP0_RX_BUF_ADDR       (USB_ENDP0_TX_BUF_ADDR + USB_ENDP0_TX_BUF_SIZE)
/*#define USB_ENDP1_TX_BUF_ADDR       (USB_ENDP0_RX_BUF_ADDR + USB_ENDP0_RX_BUF_SIZE)
#define USB_ENDP1_RX_BUF_ADDR       (USB_ENDP1_TX_BUF_ADDR + USB_ENDP1_TX_BUF_SIZE)
#define USB_ENDP2_TX_BUF_ADDR       (USB_ENDP1_RX_BUF_ADDR + USB_ENDP1_RX_BUF_SIZE)
#define USB_ENDP2_RX_BUF_ADDR       (USB_ENDP2_TX_BUF_ADDR + USB_ENDP2_TX_BUF_SIZE)
#define USB_ENDP3_TX_BUF_ADDR       (USB_ENDP2_RX_BUF_ADDR + USB_ENDP2_RX_BUF_SIZE)
#define USB_ENDP3_RX_BUF_ADDR       (USB_ENDP3_TX_BUF_ADDR + USB_ENDP3_TX_BUF_SIZE)
#define USB_ENDP4_TX_BUF_ADDR       (USB_ENDP3_RX_BUF_ADDR + USB_ENDP3_RX_BUF_SIZE)
#define USB_ENDP4_RX_BUF_ADDR       (USB_ENDP4_TX_BUF_ADDR + USB_ENDP4_TX_BUF_SIZE)
#define USB_ENDP5_TX_BUF_ADDR       (USB_ENDP4_RX_BUF_ADDR + USB_ENDP4_RX_BUF_SIZE)
#define USB_ENDP5_RX_BUF_ADDR       (USB_ENDP5_TX_BUF_ADDR + USB_ENDP5_TX_BUF_SIZE)
#define USB_ENDP6_TX_BUF_ADDR       (USB_ENDP5_RX_BUF_ADDR + USB_ENDP5_RX_BUF_SIZE)
#define USB_ENDP6_RX_BUF_ADDR       (USB_ENDP6_TX_BUF_ADDR + USB_ENDP6_TX_BUF_SIZE)
#define USB_ENDP7_TX_BUF_ADDR       (USB_ENDP6_RX_BUF_ADDR + USB_ENDP6_RX_BUF_SIZE)
#define USB_ENDP7_RX_BUF_ADDR       (USB_ENDP7_TX_BUF_ADDR + USB_ENDP7_TX_BUF_SIZE)*/

//#if ((USB_ENDP7_TX_BUF_ADDR + USB_ENDP7_TX_BUF_SIZE) > 0x0200)
//#error "No enough memory for endpoint buffer"
//#endif

/*-------------------------------------------------------------*/
/* -------------------   ISTR events  -------------------------*/
/*-------------------------------------------------------------*/
/* IMR_MSK */
/* mask defining which events has to be handled */
/* by the device application software */
#define IMR_MSK (CNTR_CTRM | CNTR_SOFM | CNTR_RESETM)

/*#define CTR_CALLBACK*/
/*#define DOVR_CALLBACK*/
/*#define ERR_CALLBACK*/
/*#define WKUP_CALLBACK*/
/*#define SUSP_CALLBACK*/
/*#define RESET_CALLBACK*/
#define SOF_CALLBACK
/*#define ESOF_CALLBACK*/

#endif /* __USB_CONF_H */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
