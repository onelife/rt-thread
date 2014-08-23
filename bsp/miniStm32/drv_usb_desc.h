/***************************************************************************//**
 * @file    drv_usb_desc.h
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
 * 2012-02-28   onelife     Copy and modify from Tsueno's example code
 ******************************************************************************/
#ifndef __DRV_USB_DESCRIPTOR_H__
#define __DRV_USB_DESCRIPTOR_H__

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"

/* Exported typedef ----------------------------------------------------------*/
//------------------------------------------
// Standard Descriptors
//------------------------------------------
//------------------------------------------
// Device Descriptor
//------------------------------------------
typedef struct {
    uint8_t     bLength;                // Size of this Descriptor in Bytes
    uint8_t     bDescriptorType;        // Descriptor Type (=1)
    uint8_t     bcdUSB[2];              // USB Spec Release Number in BCD
    uint8_t     bDeviceClass;           // Device Class Code
    uint8_t     bDeviceSubClass;        // Device Subclass Code
    uint8_t     bDeviceProtocol;        // Device Protocol Code
    uint8_t     bMaxPacketSize0;        // Maximum Packet Size for EP0
    uint8_t     idVendor[2];            // Vendor ID
    uint8_t     idProduct[2];           // Product ID
    uint8_t     bcdDevice[2];           // Device Release Number in BCD
    uint8_t     iManufacturer;          // Index of String Desc for Manufacturer
    uint8_t     iProduct;               // Index of String Desc for Product
    uint8_t     iSerialNumber;          // Index of String Desc for SerNo
    uint8_t     bNumConfigurations;     // Number of possible Configurations
} device_descriptor_t;

//--------------------------------------------------
// Configuration Descriptor
//--------------------------------------------------
typedef struct {
    uint8_t     bLength;                // Size of this Descriptor in Bytes
    uint8_t     bDescriptorType;        // Descriptor Type (=2)
    uint8_t     wTotalLength[2];        // Total Length of Data for this Conf
    uint8_t     bNumInterfaces;         // number of Interfaces supported by Conf
    uint8_t     bConfigurationValue;    // Designator Value for *this* Conf
    uint8_t     iConfiguration;         // Index of String Desc for this Conf
    uint8_t     bmAttributes;           // Configuration Characteristics
    uint8_t     bMaxPower;              // Max. Power Consumption in Conf (*2mA)
} configuration_descriptor_t;

//----------------------------------------------
// Interface Descriptor
//----------------------------------------------
typedef struct {
    uint8_t     bLength;                // Size of this Descriptor in Bytes
    uint8_t     bDescriptorType;        // Descriptor Type (=4)
    uint8_t     bInterfaceNumber;       // Number of *this* Interface (0..)
    uint8_t     bAlternateSetting;      // Alternative for this Interface
    uint8_t     bNumEndpoints;          // No of EPs used by this IF (excl. EP0)
    uint8_t     bInterfaceClass;        // Interface Class Code
    uint8_t     bInterfaceSubClass;     // Interface Subclass Code
    uint8_t     bInterfaceProtocol;     // Interface Protocol Code
    uint8_t     iInterface;             // Index of String Desc for Interface
} interface_descriptor_t;

//---------------------------------------------
// Endpoint Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;                // Size of this Descriptor in Bytes
    uint8_t     bDescriptorType;        // Descriptor Type (=5)
    uint8_t     bEndpointAddress;       // Endpoint Address (Number + Direction)
    uint8_t     bmAttributes;           // Endpoint Attributes (Transfer Type)
    uint8_t     wMaxPacketSize[2];      // Max. Endpoint Packet Size
    uint8_t     bInterval;              // Polling Interval (Interrupt) ms
} endpoint_descriptor_t;

//---------------------------------------------
// Interface Association Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;                // Size of this Descriptor in Bytes
    uint8_t     bDescriptorType;        // Descriptor Type (=11)
    uint8_t     bFirstInterface;        // Interface number of the first one associated with this function
    uint8_t     bInterfaceCount;        // Numver of contiguous interface associated with this function
    uint8_t     bFunctionClass;         // The class triad of this interface,
    uint8_t     bFunctionSubClass;      //   usually same as the triad of the first interface
    uint8_t     bFunctionProcotol;
    uint8_t     iInterface;             // Index of String Desc for this function
} interface_association_descriptor_t;

//---------------------------------------------
// Class specific descriptors for CDC
//---------------------------------------------
//---------------------------------------------
// Header Functional Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bcdCDC[2];
} header_func_descriptor_t;

//---------------------------------------------
// Call Management Functional Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bmCapabilities;
    uint8_t     bDataInterface;
} call_man_func_descriptor_t;

//---------------------------------------------
// Abstract Control Management Functional Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bmCapabilities;
} abst_control_mana_descriptor_t;

//---------------------------------------------
// Union Functional Descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bDescriptorSubtype;
    uint8_t     bMasterInterface;
    uint8_t     bSlaveInterface0;
} union_func_descriptor_t;

//---------------------------------------------
// Class specific descriptors for HID
//---------------------------------------------
//---------------------------------------------
// HID class descriptor
//---------------------------------------------
typedef struct {
    uint8_t     bLength;
    uint8_t     bDescriptorType;
    uint8_t     bcdHID[2];
    uint8_t     bCountryCode;
    uint8_t     bNumDescriptors;
    uint8_t     bReportDescriptorType;
    uint8_t     wReportDescriptorLength[2];
} hid_class_descriptor_t;

//---------------------------------------------
// Configuration Descriptor Set
//---------------------------------------------
typedef struct {
    configuration_descriptor_t              config_desc;
#if defined(USING_USB_VIRTUAL_COM)
    interface_association_descriptor_t      IAD1;
    interface_descriptor_t                  interface_desc_CDC_comm;
    header_func_descriptor_t                header_func_desc;
    call_man_func_descriptor_t              call_man_desc;
    abst_control_mana_descriptor_t          abst_control_desc;
    union_func_descriptor_t                 union_func_desc;
    endpoint_descriptor_t                   endpoint_desc_CDC_INT_IN;
    interface_descriptor_t                  interface_desc_CDC_data;
    endpoint_descriptor_t                   endpoint_desc_CDC_BULK_IN;
    endpoint_descriptor_t                   endpoint_desc_CDC_BULK_OUT;
#endif
#if defined(MINISTM32_USING_USB_HID_MOUSE)
    interface_association_descriptor_t      IAD2;
    interface_descriptor_t                  interface_desc_HID;
    hid_class_descriptor_t                  HID_class_descriptor;
    endpoint_descriptor_t                   endpoint_desc_HID_IN;
#endif
} configuration_descriptor_set_t;

/* interface number */
enum
{
#if defined(USING_USB_VIRTUAL_COM)
    DSC_INTERFACE_CDC_comm,
    DSC_INTERFACE_CDC_data,
#endif
#if defined(MINISTM32_USING_USB_HID_MOUSE)
    DSC_INTERFACE_HID,
#endif

    DSC_NUM_INTERFACE                   // Number of Interfaces (for config desc)
};

/* Exported defines ----------------------------------------------------------*/
// Descriptor type
#define DSC_TYPE_DEVICE                 0x01
#define DSC_TYPE_CONFIG                 0x02
#define DSC_TYPE_STRING                 0x03
#define DSC_TYPE_INTERFACE              0x04
#define DSC_TYPE_ENDPOINT               0x05
#define DSC_TYPE_IAD                    0x0B

// Configuration descriptor: bmAttributes
#define DSC_CNFG_ATR_BASE               0x80    // D7: base attribute, always 1
#define DSC_CNFG_ATR_SELF_POWERED       0x40    // D6: bus-powered: 0, self-powered: 1, both: 1
#define DSC_CNFG_ATR_BUS_POWERED        0x00
#define DSC_CNFG_ATR_REMOTEWAKEUP       0x20    // D5: remote-wakeup disabled: 0, enabled: 1

// Endpoint transfer type
#define DSC_EP_CONTROL                  0x00
#define DSC_EP_ISOC                     0x01
#define DSC_EP_BULK                     0x02
#define DSC_EP_INTERRUPT                0x03

// Descriptor type (Class specific)
#define DSC_TYPE_CS_INTERFACE           0x24

// Descriptor type (Class specific) - CDC
#define DSC_SUBTYPE_CS_CDC_HEADER_FUNC  0x00
#define DSC_SUBTYPE_CS_CDC_CALL_MAN     0x01
#define DSC_SUBTYPE_CS_CDC_ABST_CNTRL   0x02
#define DSC_SUBTYPE_CS_CDC_UNION_FUNC   0x06

// Descriptor type (Class specific) - HID
#define DSC_SUBTYPE_CS_HID_CLASS        0x21
#define DSC_SUBTYPE_CS_HID_REPORT       0x22
#define DSC_SUBTYPE_CS_HID_PHYSICAL     0x23

/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
#if defined BIG_ENDIAN
#define LE(x)   {(((x) & 0xFF00) >> 8), ((x) & 0x00FF)}     // convert to little endian
#else
#define LE(x)   {((x) & 0x00FF), (((x) & 0xFF00) >> 8)}
#endif

// Configuration descriptor: bMaxPower
#define DSC_CNFG_MAXPOWER(x)            (((x) + 1) / 2)     // 1 unit = 2mA

/* Exported functions ------------------------------------------------------- */
#endif /* __DRV_USB_DESCRIPTOR_H__ */
