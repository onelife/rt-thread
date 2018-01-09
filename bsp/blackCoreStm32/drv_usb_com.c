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
#include "drv_usb_com.h"

#if defined(BSP_USING_USB_VIRTUAL_COM)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Interval between sending IN packets in frame number (1 frame = 1ms) */
#define VCOMPORT_IN_FRAME_INTERVAL          5

/* Private macro -------------------------------------------------------------*/
#ifdef BSP_USB_DEBUG
#define usb_debug(format,args...)           rt_kprintf(format, ##args)
#else
#define usb_debug(format,args...)
#endif

/* Private function prototypes -----------------------------------------------*/
rt_bool_t Virtual_Com_Port_Status_In (void);
rt_bool_t Virtual_Com_Port_Status_Out (void);
RESULT Virtual_Com_Port_Data_Setup(uint8_t);
RESULT Virtual_Com_Port_NoData_Setup(uint8_t);

/* Private variables ---------------------------------------------------------*/
static const usb_device_property_t preperty = {
    Virtual_Com_Port_Status_In,
    RT_NULL,
    Virtual_Com_Port_Data_Setup,
    Virtual_Com_Port_NoData_Setup
};

static usb_virtual_com_unit_t virtual_com =
{
    0,
    {
        115200, /* baud rate*/
        0x00,   /* stop bits-1*/
        0x00,   /* parity - none*/
        0x08    /* no. of bits 8*/
    }
};

uint16_t USB_Rx_Cnt, USB_Rx_Cnt2;
uint8_t USB_Rx_Buffer[VIRTUAL_COM_PORT_DATA_SIZE];
uint8_t USB_Rx_Buffer2[VIRTUAL_COM_PORT_DATA_SIZE];
uint8_t *USB_Tx_Buffer = USB_Rx_Buffer;
uint8_t *USB_Tx_Buffer2 = USB_Rx_Buffer2;
uint8_t USB_Tx_State, USB_Tx_State2;

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : Virtual_Com_Port_GetLineCoding.
* Description    : send the linecoding structure to the PC host.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Virtual_Com_Port_GetLineCoding(uint16_t Length)
{
    if (Length == 0)
    {
        Device_Info.Ctrl_Info.Usb_wLength = sizeof(virtual_com.lineCoding);
        return NULL;
    }
    return(uint8_t *)&virtual_com.lineCoding;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_SetLineCoding.
* Description    : Set the linecoding structure fields.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *Virtual_Com_Port_SetLineCoding(uint16_t Length)
{
    if (Length == 0)
    {
        Device_Info.Ctrl_Info.Usb_wLength = sizeof(virtual_com.lineCoding);
        return NULL;
    }
    return(uint8_t *)&virtual_com.lineCoding;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_In.
* Description    : Virtual COM Port Status In Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
rt_bool_t Virtual_Com_Port_Status_In(void)
{
    if (virtual_com.RequestNo == SET_LINE_CODING)
    {
        virtual_com.RequestNo = 0;

        return RT_TRUE;
    }

    return RT_FALSE;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Status_Out
* Description    : Virtual COM Port Status OUT Routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
rt_bool_t Virtual_Com_Port_Status_Out(void)
{
    return RT_FALSE;
}

/*******************************************************************************
* Function Name  : Virtual_Com_Port_Data_Setup
* Description    : handle the data class specific requests
* Input          : Request Nb.
* Output         : None.
* Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT Virtual_Com_Port_Data_Setup(uint8_t RequestNo)
{
    uint8_t *(*CopyRoutine)(uint16_t) = RT_NULL;

    if (RequestNo == GET_LINE_CODING)
    {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
        {
            CopyRoutine = Virtual_Com_Port_GetLineCoding;
        }
    }
    else if (RequestNo == SET_LINE_CODING)
    {
        if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
        {
            CopyRoutine = Virtual_Com_Port_SetLineCoding;
        }
        virtual_com.RequestNo = SET_LINE_CODING;
    }

    if (CopyRoutine == NULL)
    {
        return USB_UNSUPPORT;
    }

    Device_Info.Ctrl_Info.CopyData = CopyRoutine;
    Device_Info.Ctrl_Info.Usb_wOffset = 0;
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
RESULT Virtual_Com_Port_NoData_Setup(uint8_t RequestNo)
{
    if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
    {
        if (RequestNo == SET_COMM_FEATURE)
        {
            return USB_SUCCESS;
        }
        else if (RequestNo == SET_CONTROL_LINE_STATE)
        {
            return USB_SUCCESS;
        }
    }

    return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : EP1_IN_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void)
{
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;

    if (USB_Tx_State == 1)
    {
        if (USB_Rx_Cnt == 0)
        {
            USB_Tx_State = 0;
        }
        else
        {
            if (USB_Rx_Cnt > VIRTUAL_COM_PORT_DATA_SIZE){
                USB_Tx_ptr = 0;
                USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;

                USB_Rx_Cnt -= VIRTUAL_COM_PORT_DATA_SIZE;
            }
            else
            {
                USB_Tx_ptr = 0;
                USB_Tx_length = USB_Rx_Cnt;

                USB_Rx_Cnt = 0;
            }

            USB_SIL_Write(EP1_IN, &USB_Rx_Buffer[0], USB_Tx_length);
            SetEPTxValid(ENDP1);
        }
    }
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void)
{
    /* Get the received data buffer and update the counter */
    USB_Rx_Cnt = USB_SIL_Read(EP1_OUT, USB_Rx_Buffer);

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the USART Xfer */

    //USB_To_USART_Send_Data(USB_Rx_Buffer, USB_Rx_Cnt);

    SetEPRxValid(ENDP1);
}

/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void)
{
  static uint32_t FrameCount = 0;

  if(usb.usb.bDeviceState == CONFIGURED)
  {
    if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL)
    {
      /* Reset the frame counter */
      FrameCount = 0;

      /* Check the data to be sent through IN pipe */
      if (USB_Rx_Cnt)
      {
          Handle_USBAsynchXfer(USB_Rx_Cnt);
      }
      if (USB_Rx_Cnt2)
      {
          Handle_USBAsynchXfer2(USB_Rx_Cnt2);
      }
    }
  }
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
rt_err_t board_hw_usb_com_init(void)
{
    usb_endpoint_register_t ep;

    ep.num = 1;
    ep.type = EP_BULK;
    ep.inBufSize = USB_UART_PORT_BULK_TX_SIZE;
    ep.outBufSize = USB_UART_PORT_BULK_RX_SIZE;
    ep.inFunc = EP1_IN_Callback;
    ep.outFunc = EP1_OUT_Callback;
    usb_endpoint_register(&usb, &ep);

    ep.num = 2;
    ep.type = EP_INTERRUPT;
    ep.inBufSize = USB_UART_PORT_INT_TX_SIZE;
    ep.outBufSize = 0x00;
    ep.inFunc = RT_NULL;
    ep.outFunc = RT_NULL;
    usb_endpoint_register(&usb, &ep);

    usb_device_register(&usb, &preperty);

    usb_debug("USB_COM: H/W init OK\n");
    return RT_EOK;
}

#endif /* defined(BSP_USING_USB_VIRTUAL_COM) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
