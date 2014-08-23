/***************************************************************************//**
 * @file    drv_usb_mouse.h
 * @brief   USB UART driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2011, RT-Thread Development Team
 * @author  onelife
 * @version 0.4 beta
 *******************************************************************************
 * @section License
 * The license and distribution terms for this file may be found in the file
 *  LICENSE in this distribution or at http://www.rt-thread.org/license/LICENSE
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-03-02   onelife     Initial creation of USB mouse DEMO for MiniSTM32
 ******************************************************************************/
#ifndef __DRV_USB_MOUSE_H__
#define __DRV_USB_MOUSE_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define USB_HID_MOUSE_INT_TX_SIZE               (0x08)
#define USB_HID_MOUSE_REPORT_SIZE               (0x32)

#define HID_DESCRIPTOR_TYPE                     0x21
#define REPORT_DESCRIPTOR                       0x22

/* Exported types ------------------------------------------------------------*/
typedef enum _HID_REQUESTS
{
    GET_REPORT = 1,
    GET_IDLE,
    GET_PROTOCOL,

    SET_REPORT = 9,
    SET_IDLE,
    SET_PROTOCOL
} HID_REQUESTS;

typedef struct usb_hid_mouse_unit
{
    uint8_t             RequestNo;
    uint32_t            ProtocolValue;
} usb_hid_mouse_unit_t;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//
// USB UART Configuration descriptor
//
#define USB_HID_MOUSE_ConfigDescriptor                                  \
    /*** Interface Association Descriptor ***/                          \
    {                                                                   \
        sizeof(interface_association_descriptor_t),  /* bLength */      \
        DSC_TYPE_IAD,					    /* bDescriptorType */       \
        DSC_INTERFACE_HID,			        /* bFirstInterface */       \
        DSC_NUM_INTERFACE,				    /* bInterfaceCount */       \
        0x03,                               /* bInterfaceClass (HID) */ \
        0x01,                               /* bInterfaceSubClass (1=BOOT, 0=no boot) */            \
        0x02,							    /* bFunctionProcotol (0=none, 1=keyboard, 2=mouse) */   \
        0x00,							    /* iInterface */            \
    },                                                                  \
    /*** Interface 0 - HID Class ***/                                   \
    {                                                                   \
        sizeof(interface_descriptor_t),	    /* bLength */               \
        DSC_TYPE_INTERFACE,				    /* bDescriptorType */       \
        DSC_INTERFACE_HID,			        /* bInterfaceNumber */      \
        0x00,							    /* bAlternateSetting */     \
        0x01,							    /* bNumEndpoints */         \
        0x03,							    /* bInterfaceClass (HID) */ \
        0x01,							    /* bInterfaceSubClass (1=BOOT, 0=no boot) */            \
        0x02,							    /* bInterfaceProcotol (0=none, 1=keyboard, 2=mouse) */  \
        0x00,							    /* iInterface */            \
    },                                                                  \
    /*** HID Class Descriptor ***/                                      \
    {                                                                   \
        sizeof(hid_class_descriptor_t),     /* bLength */               \
        DSC_SUBTYPE_CS_HID_CLASS,		    /* bDescriptorType (HID) */ \
        LE(0x0111),						    /* bcdCDC (HID spec release number, 1.11) */            \
        0x00,                               /* bCountryCode */          \
        0x01,                               /* bNumDescriptors */       \
        DSC_SUBTYPE_CS_HID_REPORT,          /* bDescriptorType */       \
        LE(USB_HID_MOUSE_REPORT_SIZE),      /* wItemLength */           \
    },                                                                  \
    /*** Endpoint 3 IN descriptor ***/                                  \
    {                                                                   \
        sizeof(endpoint_descriptor_t),	    /* bLength */               \
        DSC_TYPE_ENDPOINT,				    /* bDescriptorType */       \
        EP3_IN,						        /* bEndpointAddress */      \
        DSC_EP_INTERRUPT,					/* bmAttributes */          \
        LE(USB_HID_MOUSE_INT_TX_SIZE),	    /* MaxPacketSize */         \
        0x20,    						    /* bInterval (Polling Interval 32 ms) */              \
    },                                                                  \
    /* end of Configuration */

/* Exported functions ------------------------------------------------------- */

rt_err_t miniStm32_hw_usb_mouse_init(void);

#endif /* __DRV_USB_MOUSE_H__ */
