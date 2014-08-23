/***************************************************************************//**
 * @file    drv_usb_com.c
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
 * 2012-02-29   onelife     Copy and modify USB structures from Tsueno's
 *  example code
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_usb_core.h"
#include "drv_usb_mouse.h"

#if defined(USING_USB_HID_MOUSE)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef MINISTM32_USB_DEBUG
#define usb_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define usb_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
RESULT Mouse_Data_Setup(uint8_t);
RESULT Mouse_NoData_Setup(uint8_t);

/* Private variables ---------------------------------------------------------*/
static const usb_device_property_t preperty = {
    RT_NULL,
    RT_NULL,
    Mouse_Data_Setup,
    Mouse_NoData_Setup
};

static usb_hid_mouse_unit_t hid_mouse =
{
    0,
    0
};

//---------------------------------------------
// HID report descriptor
//---------------------------------------------
uint8_t HID_report_desc0[USB_HID_MOUSE_REPORT_SIZE] =
{
	0x05, 0x01,						        // Usage Page (Generic Desktop)
	0x09, 0x02,								// Usage (Mouse)
	0xA1, 0x01,								// Collection (Application)
	0x09, 0x01,								// Usage (Pointer)
	0xA1, 0x00,						        // Collection (Physical)
    0x05, 0x09,                             // Usage Page (Buttons)
	0x19, 0x01,								// Usage Minimum (01)
    0x29, 0x03,                             // Usage Maximun (03)
	0x15, 0x00,				                // Logical Minimum (0)
	0x25, 0x01,								// Logical Maximum (1)
	0x95, 0x03,								// Report Count (3)
	0x75, 0x01,				                // Report Size (1)
	0x81, 0x02,								// Input (Data, Variable, Absolute)
	0x95, 0x01,								// Report Count (1)
    0x75, 0x05,                             // Report Size (5)
    0x81, 0x01,                             // Input (Constant)
    0x05, 0x01,                             // Usage Page (Generic Desktop)
    0x09, 0x30,                             // Usage (X)
    0x09, 0x31,                             // Usage (Y)
    0x15, 0x81,                             // Logical Minimum (-127)
    0x25, 0x7F,                             // Logical Maximum (127)
    0x75, 0x08,                             // Report Size (8)
    0x95, 0x02,                             // Report Count (2)
    0x81, 0x06,                             // Input (Data, Variable, Relative)
    0xC0,                                   // End Collection,
	0xC0									// End Collection,
};

ONE_DESCRIPTOR HID_Report_Descriptor =
{
    (uint8_t *)HID_report_desc0,
    sizeof(HID_report_desc0)
};

ONE_DESCRIPTOR Mouse_Hid_Descriptor =
{
    (uint8_t *)&_ConfigDescriptor.HID_class_descriptor,
    sizeof(hid_class_descriptor_t)
};

__IO uint8_t PrevXferComplete = 1;

/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name : Joystick_Send.
* Description   : prepares buffer to be sent containing Joystick event infos.
* Input         : Keys: keys received from terminal.
* Output        : None.
* Return value  : None.
*******************************************************************************/
void Joystick_Send(uint8_t Keys)
{
  uint8_t Mouse_Buffer[4] = {0, 0, 0, 0};
  int8_t X = 0, Y = 0;

  switch (Keys)
  {
/*    case JOY_LEFT:
      X -= CURSOR_STEP;
      break;
    case JOY_RIGHT:

      X += CURSOR_STEP;
      break;
    case JOY_UP:
      Y -= CURSOR_STEP;
      break;
    case JOY_DOWN:
      Y += CURSOR_STEP;
      break;*/
    default:
      return;
  }

  /* prepare buffer to send */
  Mouse_Buffer[1] = X;
  Mouse_Buffer[2] = Y;

  /* Reset the control token to inform upper layer that a transfer is ongoing */
  PrevXferComplete = 0;

  /* Copy mouse position info in ENDP1 Tx Packet Memory Area*/
  USB_SIL_Write(EP3_IN, Mouse_Buffer, 4);

  /* Enable endpoint for transmission */
  SetEPTxValid(ENDP3);
}

/*main2(void)
{
    while (1)
    {

      if (bDeviceState == CONFIGURED)
      {
        if ((JoyState() != 0) && (PrevXferComplete))
        {
          Joystick_Send(JoyState());
        }
      }
    }
}*/

/*******************************************************************************
* Function Name  : Joystick_GetReportDescriptor.
* Description    : Gets the HID report descriptor.
* Input          : Length
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *Mouse_GetReportDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &HID_Report_Descriptor);
}

/*******************************************************************************
* Function Name  : Joystick_GetHIDDescriptor.
* Description    : Gets the HID descriptor.
* Input          : Length
* Output         : None.
* Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *Mouse_GetHIDDescriptor(uint16_t Length)
{
    return Standard_GetDescriptorData(Length, &Mouse_Hid_Descriptor);
}

/*******************************************************************************
* Function Name  : Joystick_SetProtocol
* Description    : Joystick Set Protocol request routine.
* Input          : None.
* Output         : None.
* Return         : USB SUCCESS.
*******************************************************************************/
RESULT Mouse_SetProtocol(void)
{
    uint8_t wValue0 = pInformation->USBwValue0;

    hid_mouse.ProtocolValue = wValue0;

    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Joystick_GetProtocolValue
* Description    : get the protocol value
* Input          : Length.
* Output         : None.
* Return         : address of the protocol value.
*******************************************************************************/
uint8_t *Mouse_GetProtocolValue(uint16_t Length)
{
    if (Length == 0)
    {
        pInformation->Ctrl_Info.Usb_wLength = 1;
        return NULL;
    }
    else
    {
        return (uint8_t *)(&hid_mouse.ProtocolValue);
    }
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Mouse_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)(uint16_t);

    CopyRoutine = NULL;
    if ((RequestNo == GET_DESCRIPTOR) && \
        (Type_Recipient == (STANDARD_REQUEST | INTERFACE_RECIPIENT)) && \
        (pInformation->USBwIndex0 == 0))
    {
        if (pInformation->USBwValue1 == REPORT_DESCRIPTOR)
        {
            CopyRoutine = Mouse_GetReportDescriptor;
        }
        else if (pInformation->USBwValue1 == HID_DESCRIPTOR_TYPE)
        {
            CopyRoutine = Mouse_GetHIDDescriptor;
        }
    } /* End of GET_DESCRIPTOR */

    /*** GET_PROTOCOL ***/
    else if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) && \
        RequestNo == GET_PROTOCOL)
    {
        CopyRoutine = Mouse_GetProtocolValue;
    }


    if (CopyRoutine == NULL)
    {
        return USB_UNSUPPORT;
    }

    pInformation->Ctrl_Info.CopyData = CopyRoutine;
    pInformation->Ctrl_Info.Usb_wOffset = 0;
    (*CopyRoutine)(0);

    return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_NoData_Setup.
* Description    : handle the no data class specific requests.
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Mouse_NoData_Setup(uint8_t RequestNo)
{
    if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) && \
        (RequestNo == SET_PROTOCOL))
    {
        return Mouse_SetProtocol();
    }

    else
    {
        return USB_UNSUPPORT;
    }
}

/*******************************************************************************
* Function Name  : EP1_OUT_Callback.
* Description    : EP1 OUT Callback Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_IN_Callback(void)
{
    /* Set the transfer complete token to inform upper layer that the current
    transfer has been complete */
    PrevXferComplete = 1;
}

/***************************************************************************//**
* @brief
*   Initialize all USART module related hardware and register USART device to
*   kernel
*
* @details
*
* @note
******************************************************************************/
rt_err_t miniStm32_hw_usb_mouse_init(void)
{
    usb_endpoint_register_t ep;

    ep.num = 3;
    ep.type = EP_INTERRUPT;
    ep.inBufSize = USB_HID_MOUSE_INT_TX_SIZE;
    ep.outBufSize = 0x00;
    ep.inFunc = RT_NULL;
    ep.outFunc = RT_NULL;
    usb_endpoint_register(&usb, &ep);

    usb_device_register(&usb, &preperty);

    usb_debug("USB_mouse: H/W init OK\n");
    return RT_EOK;
}

#endif /* defined(USING_USB_HID_MOUSE) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
