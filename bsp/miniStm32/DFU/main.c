/***************************************************************************//**
 * @file    usb_dfu.c
 * @brief   USB DFU driver of RT-Thread RTOS for MiniSTM32
 *  COPYRIGHT (C) 2012, onelife
 * @author  onelife
 * @version 1.0
 *******************************************************************************
 * @section License
 * GPL
 *******************************************************************************
 * @section Change Logs
 * Date         Author      Notes
 * 2012-03-06   onelife     Initial creation of USB DFU driver for MiniSTM32
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup STM32_DFU
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "usb_dfu.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Public variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/


/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : usb_dfu.c
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Device Firmware Upgrade(DFU) demo main file
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
typedef  void (*pFunction)(void);

/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
uint8_t DeviceState;
uint8_t DeviceStatus[6];
pFunction Jump_To_Application;
uint32_t JumpAddress;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void board_init(void)
{
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);

    /* Set NVIC Preemption Priority Bits: 0 bit for pre-emption, 4 bits for
       subpriority */
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_0);

    /* Set Base Priority Mask Register */
    __set_BASEPRI(0x00UL);
}

/*******************************************************************************
* Function Name  : main.
* Description    : main routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int main(void)
{
    board_init();

    DFU_Button_Config();

    usb.bDeviceState = UNCONNECTED;
    usb.fSuspendEnabled = TRUE;

  /* Check if the Key push-button on STM3210x-EVAL Board is pressed */
  if (DFU_Button_Read() != 0x00)
  { /* Test if user code is programmed starting from address 0x8003000 */
    if (((*(__IO uint32_t*)AppAddress) & 0x2FFE0000 ) == 0x20000000)
    { /* Jump to user application */

      JumpAddress = *(__IO uint32_t*) (AppAddress + 4);
      Jump_To_Application = (pFunction) JumpAddress;
      /* Initialize user application's Stack Pointer */
      __set_MSP(*(__IO uint32_t*) AppAddress);
      Jump_To_Application();
    }
  } /* Otherwise enters DFU mode to allow user to program his application */


  /* Config and enable clock */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  /* Enter DFU mode */
  dfu.bStatus = STATUS_ERRFIRMWARE;
  dfu.bState = STATE_dfuERROR;

  /* Unlock the internal flash */
  FLASH_Unlock();

  /* Init the media interface */
  MAL_Init();

  /* Select USBCLK source */
  RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);

  /* Enable the USB clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);

  USB_Init();

  /* Main loop */
  while (1)
  {}
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
