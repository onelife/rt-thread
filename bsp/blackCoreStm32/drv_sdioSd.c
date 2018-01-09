/***************************************************************************//**
 * @file    drv_sdio.c
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
 * 2016-03-27   onelife     Initial creation of SDIO SD card driver
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup OneBoard
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "hdl_interrupt.h"
#include "drv_sdioSd.h"
#if defined(BSP_USING_SDIOSD)
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
 
/* Private macro -------------------------------------------------------------*/
#ifdef BSP_SDIOSD_DEBUG
#define sdioSd_debug(format,args...)            rt_kprintf(format, ##args)
#else
#define sdioSd_debug(format,args...)
#endif

/* Private variables ---------------------------------------------------------*/
static struct rt_device     sd_dev;
static struct board_sdioSd_device sdioSd;
static rt_device_t          sdio_dev = RT_NULL;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Copy and Modify From STM32 Example Code -----------------------------------*/
/**
  * @brief  Intialises all cards or single card as the case may be Card(s) come 
  *         into standby state.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
static SD_Error sdioSd_InitializeCards(struct board_sdioSd_device *sdioSd)
{
    SD_Error            ret = SD_OK;
    rt_uint16_t         rca = 0x01;
    SDIO_CmdInitTypeDef cmd_init;

    do
    {
        if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
        {
            ret = SD_REQUEST_NOT_APPLICABLE;
            break;
        }

        if (SDIO_SECURE_DIGITAL_IO_CARD != sdioSd->sdio->card_type)
        {
            /* Send CMD2 ALL_SEND_CID */
            cmd_init.SDIO_Argument = 0x0;
            cmd_init.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
            cmd_init.SDIO_Response = SDIO_Response_Long;
            cmd_init.SDIO_Wait = SDIO_Wait_No;
            cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand(&cmd_init);
            ret = CmdResp2Error();
            if (SD_OK != ret)
            {
                break;
            }

            sdioSd->CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
            sdioSd->CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
            sdioSd->CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
            sdioSd->CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
        }
        
        if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == sdioSd->sdio->card_type) || \
            (SDIO_STD_CAPACITY_SD_CARD_V2_0 == sdioSd->sdio->card_type) || \
            (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == sdioSd->sdio->card_type) || \
            (SDIO_HIGH_CAPACITY_SD_CARD == sdioSd->sdio->card_type))
        {
            /* Send CMD3 SET_REL_ADDR with argument 0 */
            /* SD Card publishes its RCA. */
            cmd_init.SDIO_Argument = 0x00;
            cmd_init.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;
            cmd_init.SDIO_Response = SDIO_Response_Short;
            cmd_init.SDIO_Wait = SDIO_Wait_No;
            cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand(&cmd_init);
            ret = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);
            if (SD_OK != ret)
            {
                break;
            }
        }

        if (SDIO_SECURE_DIGITAL_IO_CARD != sdioSd->sdio->card_type)
        {
            sdioSd->RCA = rca;
            /* Send CMD9 SEND_CSD with argument as card's RCA */
            cmd_init.SDIO_Argument = (uint32_t)(rca << 16);
            cmd_init.SDIO_CmdIndex = SD_CMD_SEND_CSD;
            cmd_init.SDIO_Response = SDIO_Response_Long;
            cmd_init.SDIO_Wait = SDIO_Wait_No;
            cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
            SDIO_SendCommand(&cmd_init);
            ret = CmdResp2Error();
            if (SD_OK != ret)
            {
                break;
            }

            sdioSd->CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
            sdioSd->CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
            sdioSd->CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
            sdioSd->CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
        }
    } while (0);
    
    if (SD_OK != ret) {
        rt_kprintf("SDIOSD err: Unit init failed (%d)!\n", ret);
    }
    return ret;
}

/**
  * @brief  Returns information about specific card.
  * @param  cardinfo: pointer to a SD_CardInfo structure that contains all SD card 
  *         information.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error sdioSd_GetCardInfo(SD_CardInfo *cardinfo, struct board_sdioSd_device *sdioSd)
{
    SD_Error ret = SD_OK;
    rt_uint8_t tmp = 0;

    cardinfo->CardType = (uint8_t)sdioSd->sdio->card_type;
    cardinfo->RCA = (uint16_t)sdioSd->RCA;

    /*!< Byte 0 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    cardinfo->SD_csd.Reserved1 = tmp & 0x03;
    /*!< Byte 1 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.TAAC = tmp;
    /*!< Byte 2 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[0] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.NSAC = tmp;
    /*!< Byte 3 */
    tmp = (uint8_t)(sdioSd->CSD_Tab[0] & 0x000000FF);
    cardinfo->SD_csd.MaxBusClkFrec = tmp;
    /*!< Byte 4 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CardComdClasses = tmp << 4;
    /*!< Byte 5 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;
    /*!< Byte 6 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.Reserved2 = 0; /*!< Reserved */

    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == sdioSd->sdio->card_type) || \
        (SDIO_STD_CAPACITY_SD_CARD_V2_0 == sdioSd->sdio->card_type))
    {
        cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

        /*!< Byte 7 */
        tmp = (uint8_t)(sdioSd->CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize |= (tmp) << 2;
        /*!< Byte 8 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0xFF000000) >> 24);
        cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;
        cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
        cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);
        /*!< Byte 9 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0x00FF0000) >> 16);
        cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
        cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
        cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
        /*!< Byte 10 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0x0000FF00) >> 8);
        cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
        cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
        cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
        cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if (SDIO_HIGH_CAPACITY_SD_CARD == sdioSd->sdio->card_type)
    {
        /*!< Byte 7 */
        tmp = (uint8_t)(sdioSd->CSD_Tab[1] & 0x000000FF);
        cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;
        /*!< Byte 8 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0xFF000000) >> 24);
        cardinfo->SD_csd.DeviceSize |= (tmp << 8);
        /*!< Byte 9 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0x00FF0000) >> 16);
        cardinfo->SD_csd.DeviceSize |= (tmp);
        /*!< Byte 10 */
        tmp = (uint8_t)((sdioSd->CSD_Tab[2] & 0x0000FF00) >> 8);
        cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
        cardinfo->CardBlockSize = 512;    
    }

    cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;
    
    /*!< Byte 11 */
    tmp = (uint8_t)(sdioSd->CSD_Tab[2] & 0x000000FF);
    cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);
    /*!< Byte 12 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
    cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;
    /*!< Byte 13 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.Reserved3 = 0;
    cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);
    /*!< Byte 14 */
    tmp = (uint8_t)((sdioSd->CSD_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
    cardinfo->SD_csd.ECC = (tmp & 0x03);
    /*!< Byte 15 */
    tmp = (uint8_t)(sdioSd->CSD_Tab[3] & 0x000000FF);
    cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;

    /*!< Byte 0 */
    tmp = (uint8_t)((sdioSd->CID_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ManufacturerID = tmp;
    /*!< Byte 1 */
    tmp = (uint8_t)((sdioSd->CID_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;
    /*!< Byte 2 */
    tmp = (uint8_t)((sdioSd->CID_Tab[0] & 0x000000FF00) >> 8);
    cardinfo->SD_cid.OEM_AppliID |= tmp;
    /*!< Byte 3 */
    tmp = (uint8_t)(sdioSd->CID_Tab[0] & 0x000000FF);
    cardinfo->SD_cid.ProdName1 = tmp << 24;
    /*!< Byte 4 */
    tmp = (uint8_t)((sdioSd->CID_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdName1 |= tmp << 16;
    /*!< Byte 5 */
    tmp = (uint8_t)((sdioSd->CID_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdName1 |= tmp << 8;
    /*!< Byte 6 */
    tmp = (uint8_t)((sdioSd->CID_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdName1 |= tmp;
    /*!< Byte 7 */
    tmp = (uint8_t)(sdioSd->CID_Tab[1] & 0x000000FF);
    cardinfo->SD_cid.ProdName2 = tmp;
    /*!< Byte 8 */
    tmp = (uint8_t)((sdioSd->CID_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdRev = tmp;
    /*!< Byte 9 */
    tmp = (uint8_t)((sdioSd->CID_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdSN = tmp << 24;
    /*!< Byte 10 */
    tmp = (uint8_t)((sdioSd->CID_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdSN |= tmp << 16;
    /*!< Byte 11 */
    tmp = (uint8_t)(sdioSd->CID_Tab[2] & 0x000000FF);
    cardinfo->SD_cid.ProdSN |= tmp << 8;
    /*!< Byte 12 */
    tmp = (uint8_t)((sdioSd->CID_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdSN |= tmp;
    /*!< Byte 13 */
    tmp = (uint8_t)((sdioSd->CID_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;
    /*!< Byte 14 */
    tmp = (uint8_t)((sdioSd->CID_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ManufactDate |= tmp;
    /*!< Byte 15 */
    tmp = (uint8_t)(sdioSd->CID_Tab[3] & 0x000000FF);
    cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;

    return(ret);
}

/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by 
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus)
{
  SD_Error errorstatus = SD_OK;
  uint8_t tmp = 0;

  errorstatus = SD_SendSDStatus((uint32_t *)SDSTATUS_Tab);

  if (errorstatus  != SD_OK)
  {
    return(errorstatus);
  }

  /*!< Byte 0 */
  tmp = (uint8_t)((SDSTATUS_Tab[0] & 0xC0) >> 6);
  cardstatus->DAT_BUS_WIDTH = tmp;

  /*!< Byte 0 */
  tmp = (uint8_t)((SDSTATUS_Tab[0] & 0x20) >> 5);
  cardstatus->SECURED_MODE = tmp;

  /*!< Byte 2 */
  tmp = (uint8_t)((SDSTATUS_Tab[2] & 0xFF));
  cardstatus->SD_CARD_TYPE = tmp << 8;

  /*!< Byte 3 */
  tmp = (uint8_t)((SDSTATUS_Tab[3] & 0xFF));
  cardstatus->SD_CARD_TYPE |= tmp;

  /*!< Byte 4 */
  tmp = (uint8_t)(SDSTATUS_Tab[4] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

  /*!< Byte 5 */
  tmp = (uint8_t)(SDSTATUS_Tab[5] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

  /*!< Byte 6 */
  tmp = (uint8_t)(SDSTATUS_Tab[6] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

  /*!< Byte 7 */
  tmp = (uint8_t)(SDSTATUS_Tab[7] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

  /*!< Byte 8 */
  tmp = (uint8_t)((SDSTATUS_Tab[8] & 0xFF));
  cardstatus->SPEED_CLASS = tmp;

  /*!< Byte 9 */
  tmp = (uint8_t)((SDSTATUS_Tab[9] & 0xFF));
  cardstatus->PERFORMANCE_MOVE = tmp;

  /*!< Byte 10 */
  tmp = (uint8_t)((SDSTATUS_Tab[10] & 0xF0) >> 4);
  cardstatus->AU_SIZE = tmp;

  /*!< Byte 11 */
  tmp = (uint8_t)(SDSTATUS_Tab[11] & 0xFF);
  cardstatus->ERASE_SIZE = tmp << 8;

  /*!< Byte 12 */
  tmp = (uint8_t)(SDSTATUS_Tab[12] & 0xFF);
  cardstatus->ERASE_SIZE |= tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((SDSTATUS_Tab[13] & 0xFC) >> 2);
  cardstatus->ERASE_TIMEOUT = tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((SDSTATUS_Tab[13] & 0x3));
  cardstatus->ERASE_OFFSET = tmp;
 
  return(errorstatus);
}

/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by 
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval SD_Error: SD Card Error code.
  */
SD_Error sdioSd_EnableWideBusOperation(uint32_t WideMode, struct board_sdioSd_device *sdioSd)
{
    SD_Error ret = SD_OK;
    SDIO_InitTypeDef sdio_init;

    do {
        /*!< MMC Card doesn't support this feature */
        if (SDIO_MULTIMEDIA_CARD == sdioSd->sdio->card_type)
        {
            ret = SD_UNSUPPORTED_FEATURE;
            break;
        }
        else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == sdioSd->sdio->card_type) || \
                 (SDIO_STD_CAPACITY_SD_CARD_V2_0 == sdioSd->sdio->card_type) || \
                 (SDIO_HIGH_CAPACITY_SD_CARD == sdioSd->sdio->card_type))
        {
            if (SDIO_BusWide_8b == WideMode)
            {
                ret = SD_UNSUPPORTED_FEATURE;
                break;
            }
            else if (SDIO_BusWide_4b == WideMode)
            {
                ret = SDEnWideBus(ENABLE);

                if (SD_OK == ret)
                {
                    /*!< Configure the SDIO peripheral */
                    sdio_init.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
                    sdio_init.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
                    sdio_init.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
                    sdio_init.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
                    sdio_init.SDIO_BusWide = SDIO_BusWide_4b;
                    sdio_init.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
                    SDIO_Init(&sdio_init);
                }
            }
            else
            {
                ret = SDEnWideBus(DISABLE);

                if (SD_OK == ret)
                {
                    /*!< Configure the SDIO peripheral */
                    sdio_init.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
                    sdio_init.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
                    sdio_init.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
                    sdio_init.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
                    sdio_init.SDIO_BusWide = SDIO_BusWide_1b;
                    sdio_init.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
                    SDIO_Init(&sdio_init);
                }
            }
        }
    } while (0);

    if (SD_OK != ret) {
        rt_kprintf("SDIOSD err: Unit init failed (%d)!\n", ret);
    }
    return ret;
    }

/**
  * @brief  Selects od Deselects the corresponding card.
  * @param  addr: Address of the Card to be selected.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error sdioSd_SelectDeselect(uint32_t addr)
{
    SD_Error ret = SD_OK;
    SDIO_CmdInitTypeDef cmd_init;

    /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
    cmd_init.SDIO_Argument =  addr;
    cmd_init.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
    cmd_init.SDIO_Response = SDIO_Response_Short;
    cmd_init.SDIO_Wait = SDIO_Wait_No;
    cmd_init.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&cmd_init);

    ret = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

    return(ret);
}


/***************************************************************************//**
* @brief
*   Initialize all SD card (by SDIO) related hardware and register the device to
*  kernel
*
* @details
*
* @note
*
* @return
*   Error code
******************************************************************************/
rt_err_t board_hw_sdioSd_init(void)
{
    do
    {
        /* Find SDIO device */
        sdio_dev = rt_device_find(SDIOSD_USING_DEVICE_NAME);
        if (sdio_dev == RT_NULL)
        {
            sdioSd_debug("SDIOSD: Can't find device %s!\n",
                SDIOSD_USING_DEVICE_NAME);
            break;
        }
        sdioSd_debug("SDIOSD: Find device %s\n", SDIOSD_USING_DEVICE_NAME);

        /* Register SDIO SD device */
        sd_dev.type      = RT_Device_Class_MTD;
        sd_dev.init      = rt_sdioSd_init;
        sd_dev.open      = rt_sdioSd_open;
        sd_dev.close     = rt_sdioSd_close;
        sd_dev.read      = rt_sdioSd_read;
        sd_dev.write     = rt_sdioSd_write;
        sd_dev.control   = rt_sdioSd_control;
        sd_dev.user_data = (void*)&sdioSd;
        if (rt_device_register(
            &sd_dev,
            SDIOSD_DEVICE_NAME,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE
            ) != RT_EOK)
        {
            break;
        }
        sdioSd_debug("SDIOSD: HW init OK\n");
        return RT_EOK;
    } while (0);

    rt_kprintf("SDIOSD: HW init failed!\n");
    return -RT_ERROR;
}
