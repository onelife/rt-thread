/***************************************************************************//**
 * @file    drv_usb_com.h
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
 * 2012-02-24   onelife     Initial creation of USB UART module driver for
 *  MiniSTM32
s ******************************************************************************/
#ifndef __DRV_USB_COM_H__
#define __DRV_USB_COM_H__

/* Includes ------------------------------------------------------------------*/
/* Exported defines ----------------------------------------------------------*/
#define USB_UART_PORT_BULK_TX_SIZE              (0x40)
#define USB_UART_PORT_BULK_RX_SIZE              (0x40)
#define USB_UART_PORT_INT_TX_SIZE               (0x08)

#define VIRTUAL_COM_PORT_DATA_SIZE              64
#define VIRTUAL_COM_PORT_INT_SIZE               8

#define VIRTUAL_COM_PORT_SIZ_DEVICE_DESC        18
#define VIRTUAL_COM_PORT_SIZ_CONFIG_DESC        75
#define VIRTUAL_COM_PORT_SIZ_STRING_LANGID      4
#define VIRTUAL_COM_PORT_SIZ_STRING_VENDOR      38
#define VIRTUAL_COM_PORT_SIZ_STRING_PRODUCT     50
#define VIRTUAL_COM_PORT_SIZ_STRING_SERIAL      26

#define STANDARD_ENDPOINT_DESC_SIZE             0x09

#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23

/* Exported types ------------------------------------------------------------*/
typedef struct
{
    uint32_t            bitrate;
    uint8_t             format;
    uint8_t             paritytype;
    uint8_t             datatype;
} LINE_CODING;

typedef struct usb_virtual_com_unit
{
    uint8_t             RequestNo;
    LINE_CODING         lineCoding;
} usb_virtual_com_unit_t;

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
//
// USB UART Configuration descriptor
//
#define USB_VIRTUAL_COM_ConfigDescriptor                                \
    /*** Interface Association Descriptor ***/                          \
    {                                                                   \
        sizeof(interface_association_descriptor_t),  /* bLength */      \
        DSC_TYPE_IAD,					    /* bDescriptorType */       \
        DSC_INTERFACE_CDC_comm,			    /* bFirstInterface */       \
        DSC_NUM_INTERFACE,				    /* bInterfaceCount */       \
        0x02,							    /* bFunctionClass (Communication Class) */              \
        0x02,						        /* bFunctionSubClass (Abstract Control Model) */        \
        0x01,							    /* bFunctionProcotol (V.25ter, Common AT commands) */   \
        0x00,							    /* iInterface */            \
    },                                                                  \
    /*** Interface 0 - CDC Communication Class ***/                     \
    {                                                                   \
        sizeof(interface_descriptor_t),	    /* bLength */               \
        DSC_TYPE_INTERFACE,				    /* bDescriptorType */       \
        DSC_INTERFACE_CDC_comm,			    /* bInterfaceNumber */      \
        0x00,							    /* bAlternateSetting */     \
        0x01,							    /* bNumEndpoints */         \
        0x02,							    /* bInterfaceClass (Communication Class) */             \
        0x02,							    /* bInterfaceSubClass (Abstract Control Model) */       \
        0x01,							    /* bInterfaceProcotol (V.25ter, Common AT commands) */  \
        0x00,							    /* iInterface */            \
    },                                                                  \
    /*** Header Functional Descriptor ***/                              \
    {                                                                   \
        sizeof(header_func_descriptor_t),   /* bLength */               \
        DSC_TYPE_CS_INTERFACE,			    /* bDescriptorType (CS_INTERFACE) */                    \
        DSC_SUBTYPE_CS_CDC_HEADER_FUNC,	    /* bDescriptorSubtype (Header Functional) */            \
        LE(0x0110),						    /* bcdCDC (CDC spec release number, 1.1) */             \
    },                                                                  \
    /*** Call Management Functional Descriptor ***/                     \
    {                                                                   \
        sizeof(call_man_func_descriptor_t), /* bLength */               \
        DSC_TYPE_CS_INTERFACE,			    /* bDescriptorType (CS_INTERFACE) */                    \
        DSC_SUBTYPE_CS_CDC_CALL_MAN,	    /* bDescriptorSubtype (Call Management) */              \
        0x00,							    /* TODO: 0x01 bmCapabilities (only over Communication Class IF / handles itself) */ \
        DSC_INTERFACE_CDC_data,			    /* bDataInterface (Interface number of Data Class interface) */                     \
    },                                                                  \
    /*** Abstract Control Management Functional Descriptor ***/         \
    {                                                                   \
        sizeof(abst_control_mana_descriptor_t), /* bLength */           \
        DSC_TYPE_CS_INTERFACE,			    /* bDescriptorType (CS_INTERFACE) */                    \
        DSC_SUBTYPE_CS_CDC_ABST_CNTRL,	    /* bDescriptorSubtype (Abstract Control Management) */  \
        0x02,							    /* TODO: 0x06 bmCapabilities (Supports Send_Break, Set_Line_Coding,
        								    Set_Control_Line_State, Get_Line_Coding, and the notification Serial_State) */      \
    },                                                                  \
    /*** Union Functional Descriptor ***/                               \
    {                                                                   \
        sizeof(union_func_descriptor_t),    /* bLength */               \
        DSC_TYPE_CS_INTERFACE,		        /* bDescriptorType (CS_INTERFACE) */                    \
        DSC_SUBTYPE_CS_CDC_UNION_FUNC,	    /* bDescriptorSubtype (Union Functional) */             \
        DSC_INTERFACE_CDC_comm,			    /* bMasterInterface (Interface number master interface in the union) */             \
        DSC_INTERFACE_CDC_data,			    /* bSlaveInterface0 (Interface number slave interface in the union) */              \
    },                                                                  \
    /*** Endpoint 2 IN descriptor ***/                                  \
    {                                                                   \
        sizeof(endpoint_descriptor_t),	    /* bLength */               \
        DSC_TYPE_ENDPOINT,				    /* bDescriptorType */       \
        EP2_IN,							    /* bEndpointAddress */      \
        DSC_EP_INTERRUPT,				    /* bmAttributes */          \
        LE(USB_UART_PORT_INT_TX_SIZE),	    /* MaxPacketSize */         \
        0xFF							    /* TODO: ? bInterval */     \
    },                                                                  \
    /*** Interface 1 - CDC Data Interface Class ***/                    \
    {                                                                   \
        sizeof(interface_descriptor_t),	    /* bLength */               \
        DSC_TYPE_INTERFACE, 			    /* bDescriptorType */       \
        DSC_INTERFACE_CDC_data,			    /* bInterfaceNumber */      \
        0x00,							    /* bAlternateSetting */     \
        0x02,							    /* bNumEndpoints */         \
        0x0A,							    /* bInterfaceClass (Data Interface Class) */            \
        0x00,							    /* bInterfaceSubClass */    \
        0x00,							    /* bInterfaceProcotol (No class specific protocol required) */  \
        0x00							    /* iInterface */            \
    },                                                                  \
    /*** Endpoint 1 OUT descriptor ***/                                 \
    {                                                                   \
        sizeof(endpoint_descriptor_t),	    /* bLength */               \
        DSC_TYPE_ENDPOINT,				    /* bDescriptorType */       \
        EP1_OUT,						    /* bEndpointAddress */      \
        DSC_EP_BULK,					    /* bmAttributes */          \
        LE(USB_UART_PORT_BULK_RX_SIZE),	    /* MaxPacketSize */         \
        0x00    						    /* bInterval (ignore for Bulk transfer) */              \
    },                                                                  \
    /*** Endpoint 1 IN descriptor ***/                                  \
    {                                                                   \
        sizeof(endpoint_descriptor_t),      /* bLength */               \
        DSC_TYPE_ENDPOINT,                  /* bDescriptorType */       \
        EP1_IN,                             /* bEndpointAddress */      \
        DSC_EP_BULK,                        /* bmAttributes */          \
        LE(USB_UART_PORT_BULK_TX_SIZE),     /* MaxPacketSize */         \
        0x00                                /* bInterval (ignore for Bulk transfer)  */             \
    },
    /* end of Configuration */

/* Exported functions ------------------------------------------------------- */

rt_err_t board_hw_usb_com_init(void);

#endif /* __DRV_USB_COM_H__ */
