/***************************************************************************//**
 * @file    drv_sdcard.c
 * @brief   Memory card driver (SPI mode) of RT-Thread RTOS for MiniSTM32
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
 * 2012-01-24   onelife     Initial creation of SD card driver (through SPI) for
 *  MiniSTM32 (Modified from EFN32 branch)
 ******************************************************************************/

/***************************************************************************//**
 * @addtogroup MiniSTM32
 * @{
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "board.h"
#include "drv_spi.h"
#include "drv_sdcard.h"

#if defined(BSP_USING_SPISD)
#include <dfs_fs.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#ifdef BSP_SDCARD_DEBUG
#define sdcard_debug(format,args...)        rt_kprintf(format, ##args)
#else
#define sdcard_debug(format,args...)
#endif

/* Private constants ---------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static struct rt_device     sd_dev;
static struct dfs_partition sdPart;
static rt_device_t          spi_dev         = RT_NULL;
static rt_uint16_t          sdType;
static rt_timer_t           sdTimer         = RT_NULL;
static volatile rt_bool_t   sdInTime        = RT_TRUE;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/***************************************************************************//**
 * @brief
 *   Memory device timeout interrupt handler
 *
 * @details
 *
 * @note
 *
 * @param[in] parameter
 *  Parameter
 ******************************************************************************/
static void bsp_spiSd_timer(void* parameter)
{
    sdInTime = RT_FALSE;
}

/***************************************************************************//**
 * @brief
 *   Set/Clear chip select
 *
 * @details
 *
 * @note
 *
 * @param[in] enable
 *  Chip select pin setting
 ******************************************************************************/
#if (defined(SD_CS_PORT) && defined(SD_CS_PIN))
#   define bsp_spiSd_cs(enable)  if (enable) SD_CS_PORT->BRR = SD_CS_PIN; else SD_CS_PORT->BSRR = SD_CS_PIN;
#endif


/***************************************************************************//**
 * @brief
 *   Set operation speed level
 *
 * @details
 *
 * @note
 *
 * @param[in] level
 *  Set SD speed level
 ******************************************************************************/
static void bsp_spiSd_speed(rt_uint8_t level)
{
    RT_ASSERT(spi_dev != RT_NULL);

    struct bsp_spi_device *spi;
    rt_uint16_t prescaler = (level == SD_SPEED_HIGH)? BSP_SDCLK_HIGH : BSP_SDCLK_LOW;

    spi = (struct bsp_spi_device *)(spi_dev->user_data);
    spi->spi_device->CR1 &= ~0x0038;
    spi->spi_device->CR1 |= prescaler;
}

/***************************************************************************//**
 * @brief
 *   Read raw data from memory device
 *
 * @details
 *
 * @note
 *
 * @param[in] buffer
 *   Poniter to the buffer
 *
 * @param[in] size
 *   Buffer size in byte
 *
 * @return
 *   Number of read bytes
 ******************************************************************************/
static rt_size_t bsp_spiSd_read(void *buffer, rt_size_t size)
{
    rt_uint8_t buf_read[5], ret;

    /* Build instruction buffer */
    buf_read[0] = 0x00;
    *(rt_uint8_t **)(&buf_read[1]) = buffer;
    /* Read data */
    bsp_spiSd_cs(1);
    if ((ret = rt_device_read(spi_dev, BSP_NO_DATA, buf_read, size)) == 0)
    {
        rt_uint8_t *temp = (rt_uint8_t *)buffer;
        sdcard_debug("SPISD: Read failed! (%d, %x %x %x %x %x)\n", ret,
            *temp, *(temp + 1), *(temp + 2), *(temp + 3), *(temp + 4));
    }
    bsp_spiSd_cs(0);

    return ret;
}

/***************************************************************************//**
 * @brief
 *   Send command to memory device
 *
 * @details
 *
 * @note
 *
 * @param[in] cmd
 *   Command index
 *
 * @param[in] arg
 *   Argument
 *
 * @param[in] trail
 *   Pointer to the buffer to store trailing data
 *
 * @return
 *   Command response
 ******************************************************************************/
static rt_uint16_t bsp_spiSd_cmd(
    rt_uint8_t cmd,
    rt_uint32_t arg,
    rt_uint8_t *trail)
{
    rt_uint8_t buf_ins[11];
    rt_uint8_t buf_res[32];     /* Expect (x+1+4) bytes for CRC, (x+1+19) bytes for CSD/CID */
    rt_uint8_t len_trl, i, j;
    rt_uint16_t ret;
    rt_bool_t skip;

    ret = 0xffff;
    rt_memset(buf_res, 0xff, sizeof(buf_res));

    sdcard_debug("SPISD: Send command %d(%x)\n", cmd, arg);
    do
    {
        /* Build instruction buffer */
        buf_ins[0] = 6;                             /* Instruction length */
        buf_ins[1] = 0x40 | cmd;                    /* Command index */
        buf_ins[2] = (arg >> 24) & 0x000000ff;      /* Argument: MSB first */
        buf_ins[3] = (arg >> 16) & 0x000000ff;
        buf_ins[4] = (arg >> 8) & 0x000000ff;
        buf_ins[5] = arg & 0x000000ff;
        switch (cmd) {
        case CMD0:  buf_ins[6] = 0x95; break;       /* Valid CRC for CMD0(0) */
        case CMD8:  buf_ins[6] = 0x87; break;       /* Valid CRC for CMD8(0x1AA) */
        case CMD58: buf_ins[6] = 0x01; break;       /* Dummy CRC + Stop */
        default:    buf_ins[6] = 0x01;              /* Dummy CRC + Stop */
        }
        *(rt_uint8_t **)(&buf_ins[7]) = buf_res;    /* Pointer to RX buffer */

        /* Set trail length */
        switch (cmd) {
        case CMD8:  len_trl = 4; break;             /* R7 response */
        case CMD9:  len_trl = SD_BLOCK_SIZE_CSD; break; 
        case CMD10: len_trl = SD_BLOCK_SIZE_CID; break;
        case CMD58: len_trl = SD_BLOCK_SIZE_OCR; break; /* R3 response */
        default:    len_trl = 0;
        }

        /* Send command and get response */
        bsp_spiSd_cs(1);
        rt_err_t rst;
        if ((rst=rt_device_read(spi_dev, BSP_NO_DATA, buf_ins, sizeof(buf_res))) == 0)
        {
            sdcard_debug("SPISD: Send command failed! (%d, %x %x %x %x %x)\n", rst,
                buf_res[0], buf_res[1], buf_res[2], buf_res[3], buf_res[4]);
            break;
        }
        bsp_spiSd_cs(0);

        /* Skip a stuff byte when stop reading */
        skip = (cmd == CMD12) ? RT_TRUE : RT_FALSE;

        /* Find valid response: The response is sent back within command response time
            (NCR), 0 to 8 bytes for SDC, 1 to 8 bytes for MMC */
        for (i = 0; i < sizeof(buf_res); i++)
        {
            if (buf_res[i] != 0xff)
            {
                if (skip)
                {
                    skip = RT_FALSE;
                    sdcard_debug("SPISD: Skip %x (at %d)\n", buf_res[i], i);
                    continue;
                }

                if (cmd == ACMD13 & 0x7f)
                {
                    ret = (rt_uint16_t)buf_res[i];  /* R2 response */
                }
                else
                {
                    ret = (rt_uint8_t)buf_res[i];
                }
                break;
            }
        }
        sdcard_debug("SPISD: Response %x (at %d)\n", ret, i);
        i++;
        
        /* Copy the trailing data */
        if ((ret != 0xffff) && len_trl && trail)
        {
            /* Read CSD/CID */
            if (cmd == CMD9 || cmd == CMD10)
            {
                /* Find data block */
                for (; i < sizeof(buf_res); i++)
                {
                    if (buf_res[i] == 0xfe) break;
                }
                /* Check if valid */
                if (i >= sizeof(buf_res))
                {
                    sdcard_debug("SPISD: No CSD/CID found!\n");
                    ret = 0xffff;
                    break;
                }
                i++;
                sdcard_debug("SPISD: CSD/CID %x (at %d)\n", buf_res[i], i);
                sdcard_debug("SPISD: Read CRC %x %x\n", buf_res[i+len_trl], 
                    buf_res[i+len_trl+1]);
            }

            /* Copy the data */
            for (j = 0; j < len_trl; j++)
            {
                trail[j] = buf_res[i + j];
            }
        }
    } while(0);

    return ret;
}

/***************************************************************************//**
 * @brief
 *   Read a block of data from memory device. This function is used to handle
 *  the responses of specified commands (e.g. ACMD13, CMD17 and CMD18)
 *
 * @details
 *
 * @note
 *
 * @param[in] buffer
 *   Poniter to the buffer
 *
 * @param[in] size
 *   Buffer size in byte
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t bsp_spiSd_readBlock(void *buffer, rt_size_t size)
{
    rt_uint8_t buf_ins[5];
    rt_uint8_t buf_res[8];      /* Expect 2 bytes for CRC */
    rt_uint8_t i, len_copy;
    rt_bool_t start;

    start = RT_FALSE;
    do
    {
        /* Build instruction buffer */
        buf_ins[0] = 0;                             /* Instruction length */
        *(rt_uint8_t **)(&buf_ins[1]) = buf_res;    /* Pointer to RX buffer */

        while(1)
        {
            /* Send read command */
            bsp_spiSd_cs(1);
            if (rt_device_read(spi_dev, BSP_NO_DATA, buf_ins, \
                sizeof(buf_res)) == 0)
            {
                sdcard_debug("SPISD: Get read command response failed!\n");
                break;
            }
            bsp_spiSd_cs(0);
            /* Wait for data */
            for (i = 0; i < sizeof(buf_res); i++)
            {
                if (buf_res[i] != 0xff)
                {
                    start = RT_TRUE;
                    break;
                }
            }
            if (start)
            {
                break;
            }
        };

        /* Ckeck if valid */
        if (!start || (buf_res[i] != 0xfe))
        {
            sdcard_debug("SPISD: Invalid token %x (at %d)!\n", buf_res[i], i);
            break;
        }
        /* Copy data to buffer and read the rest */
        len_copy = sizeof(buf_res) - i - 1;
        rt_memcpy(buffer, &buf_res[i + 1], len_copy);
        sdcard_debug("SPISD: Read block start at %d, copy %d bytes\n", i, \
            len_copy);

        /* Build instruction buffer */
        buf_ins[0] = 0;                             /* Instruction length */
        *(rt_uint8_t **)(&buf_ins[1]) = (rt_uint8_t *)buffer + len_copy;    /* Pointer to RX buffer */

        /* Send read command */
        bsp_spiSd_cs(1);
        if (rt_device_read(spi_dev, BSP_NO_DATA, buf_ins, size - len_copy) == 0)
        {
            sdcard_debug("SPISD: Read data block failed!\n");
            break;
        }
        *(rt_uint8_t **)(&buf_ins[1]) = buf_res;    /* Pointer to RX buffer */
        if (rt_device_read(spi_dev, BSP_NO_DATA, buf_ins, sizeof(buf_res)) == 0)
        {
            sdcard_debug("SPISD: Read CRC failed!\n");
            break;
        }
        sdcard_debug("SPISD: Read CRC %x %x\n", buf_res[0], buf_res[1]);
        bsp_spiSd_cs(0);

        return RT_EOK;
    } while(0);

    sdcard_debug("SPISD: Read block failed!\n");
    return -RT_ERROR;
}

/***************************************************************************//**
 * @brief
 *   Write a block of data to memory device. This function is used to send data
 *  and control tokens for block write commands (e.g. CMD24 and CMD25)
 *
 * @details
 *
 * @note
 *
 * @param[in] buffer
 *   Poniter to the buffer
 *
 * @param[in] token
 *   Control token
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t bsp_spiSd_writeBlock(void *buffer, rt_uint8_t token)
{
    rt_err_t ret;
    rt_uint8_t buf_ins[11];
    rt_uint8_t buf_res[8];      /* Expect a byte for data response */
    rt_uint8_t i;

    ret = RT_ERROR;
    sdcard_debug("SPISD: Write block\n");
    do
    {
        /* Initialize timer */
        sdInTime = RT_TRUE;
        rt_timer_start(sdTimer);
        /* Wait for card ready */
        do
        {
            bsp_spiSd_read(buf_res, sizeof(buf_res));
        } while (sdInTime && (buf_res[sizeof(buf_res) - 1] != 0xff));
        if (buf_res[sizeof(buf_res) - 1] != 0xff)
        {
            sdcard_debug("SPISD: Card is busy before writing! (%x)\n", \
                buf_res[sizeof(buf_res) - 1]);
            ret = -RT_EBUSY;
            break;
        }
        rt_timer_stop(sdTimer);

        /* Send data */
        sdcard_debug("SPISD: Send data, token %x\n", token);
        if (token != 0xfd)
        {
            /* Send token and data */
            buf_ins[0] = 1;                             /* Instruction length */
            buf_ins[1] = token;
            *(rt_uint8_t **)(&buf_ins[2]) = (rt_uint8_t *)buffer;   /* Pointer to TX buffer */
            bsp_spiSd_cs(1);
            if (rt_device_write(spi_dev, BSP_NO_DATA, buf_ins, SD_SECTOR_SIZE) == 0)
            {
                sdcard_debug("SPISD: Write data failed!\n");
                break;
            }

            /* Build instruction buffer */
            buf_ins[0] = 2;                             /* Instruction length */
            buf_ins[1] = 0xff;                          /* CRC (Dummy) */
            buf_ins[2] = 0xff;
            *(rt_uint8_t **)(&buf_ins[3]) = buf_res;    /* Pointer to RX buffer */
            /* Send CRC and read a byte */
            if (rt_device_read(spi_dev, BSP_NO_DATA, buf_ins, sizeof(buf_res)) == 0)
            {
                sdcard_debug("SPISD: Write CRC failed!\n");
                break;
            }
            bsp_spiSd_cs(0);

            /* Check if accepted */
            for (i = 0; i < sizeof(buf_res); i++)
            {
                if (buf_res[i] != 0xff)
                {
                    buf_res[i] &= 0x1f;
                    break;
                }
            }
            if (buf_res[i] != 0x05)
            {
                sdcard_debug("SPISD: Writing is not accepted! (%x at %d)\n", \
                    buf_res[i], i);
                break;
            }
        }
        else
        {
            /* Send token */
            buf_ins[0] = 1;                             /* Instruction length */
            buf_ins[1] = token;
            *(rt_uint8_t **)(&buf_ins[2]) = RT_NULL;    /* Pointer to TX buffer */
            bsp_spiSd_cs(1);
            if (rt_device_write(spi_dev, BSP_NO_DATA, buf_ins, 0) != 0)
            {
                sdcard_debug("SPISD: Write token failed!\n");
                break;
            }

            /* Initialize timer */
            sdInTime = RT_TRUE;
            rt_timer_start(sdTimer);
            /* Wait for card ready */
            do
            {
                bsp_spiSd_read(buf_res, sizeof(buf_res));
            } while (sdInTime && (buf_res[sizeof(buf_res) - 1] != 0xff));
            if (buf_res[sizeof(buf_res) - 1] != 0xff)
            {
                sdcard_debug("SPISD: Card is busy after writing! (%x)\n", \
                    buf_res[sizeof(buf_res) - 1] );
                ret = -RT_EBUSY;
                break;
            }
            rt_timer_stop(sdTimer);
        }

        return RT_EOK;
    } while(0);

    sdcard_debug("SPISD: Write block failed!\n");
    return ret;
}

/***************************************************************************//**
 * @brief
 *   Wrapper function of send command to memory device
 *
 * @details
 *
 * @note
 *
 * @param[in] cmd
 *   Command index
 *
 * @param[in] arg
 *   Argument
 *
 * @param[in] trail
 *   Pointer to the buffer to store trailing data
 *
 * @return
 *   Command response
 ******************************************************************************/
rt_uint16_t bsp_spiSd_sendCmd(
    rt_uint8_t cmd,
    rt_uint32_t arg,
    rt_uint8_t *trail)
{
    rt_uint16_t ret;

    /* ACMD<n> is the command sequense of CMD55-CMD<n> */
    if (cmd & 0x80)
    {
        cmd &= 0x7f;
        ret = bsp_spiSd_cmd(CMD55, 0x00000000, BSP_NO_POINTER);
        if (ret > 0x01)
        {
            return ret;
        }
    }

    return bsp_spiSd_cmd(cmd, arg, trail);
}

/***************************************************************************//**
 * @brief
 *   De-initialize memory card device
 *
 * @details
 *
 * @note
 ******************************************************************************/
void bsp_spiSd_deinit(void)
{
    /* Close SPI device */
    rt_device_close(spi_dev);
    spi_dev = RT_NULL;
    sdcard_debug("SPISD: Close device %s\n", SPISD_USING_DEVICE_NAME);

    /* Delete timer */
    if (sdTimer != RT_NULL)
    {
        rt_timer_delete(sdTimer);
        sdTimer = RT_NULL;
        sdcard_debug("SPISD: Delete timer\n");
    }

    sdcard_debug("SPISD: Deinit OK\n");
}

/***************************************************************************//**
 * @brief
 *   Initialize memory card device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t rt_spiSd_init(rt_device_t dev)
{
    /* Ref: http://elm-chan.org/docs/mmc/mmc_e.html */
    rt_uint8_t type, tril[4];
    rt_uint8_t *buf_res;
    rt_uint8_t retry;

    type = 0;
    buf_res = RT_NULL;
    retry = SD_INIT_RETRY_TIMES;

    do
    {
        /* Create and setup timer */
        if ((sdTimer = rt_timer_create(
            "sd_tmr",
            bsp_spiSd_timer,
            RT_NULL,
            SD_WAIT_PERIOD,
            RT_TIMER_FLAG_ONE_SHOT)) == RT_NULL)
        {
            sdcard_debug("SPISD: Create timer failed!\n");
            break;
        }

        /* Open SPI device */
        if (rt_device_open(spi_dev, RT_DEVICE_OFLAG_RDWR) != RT_EOK)
        {
            break;
        }

        /* Switch to low speed */
        bsp_spiSd_speed(SD_SPEED_LOW);

        /* >74 dummy clocks */
        rt_device_control(spi_dev, RT_DEVICE_CTRL_SPI_OUTPUT_CLOCK, (void *)10);
        /* Enter Idle state */
        while (retry && (bsp_spiSd_sendCmd(CMD0, 0x00000000, BSP_NO_POINTER) != 0x01))
        {
			retry--;
        }
        if (!retry)
        {
            break;
        }
        /* Check if SDv2 */
        if (bsp_spiSd_sendCmd(CMD8, 0x000001AA, tril) == 0x01)
        {
            /* SDv2, Vdd: 2.7-3.6V */
            if ((tril[2] & 0x0F) == 0x01 && tril[3] == 0xAA)
            {
                /* Initialize timer */
                sdInTime = RT_TRUE;
                rt_timer_start(sdTimer);
                /* SD initialization (ACMD41 with HCS bit) */
                while (sdInTime && bsp_spiSd_sendCmd(ACMD41, 0x40000000, BSP_NO_POINTER));
                /* Check CCS bit (bit 30) in the OCR */
                if (sdInTime && (bsp_spiSd_sendCmd(CMD58, 0x00000000, tril) == 0x00))
                {
                    type = (tril[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        }
        else
        {
            /* Initialize timer */
            sdInTime = RT_TRUE;
            rt_timer_start(sdTimer);
            /* SD initialization */
            while (sdInTime && bsp_spiSd_sendCmd(ACMD41, 0x00000000, BSP_NO_POINTER));
            if (sdInTime)
            {
                /* SDv1 */
                type = CT_SD1;
            }
            else
            {
                /* Initialize timer */
                sdInTime = RT_TRUE;
                rt_timer_start(sdTimer);
                /* MMC initialization */
                while (sdInTime && bsp_spiSd_sendCmd(CMD1, 0x00000000, BSP_NO_POINTER));
                if (sdInTime)
                {
                    /* MMCv3 */
                    type = CT_MMC;
                }
            }
        }
        rt_timer_stop(sdTimer);

        /* Set read/write block length to 512 bytes */
        if ((type > 0) && !(type | CT_BLOCK))
        {
            if (bsp_spiSd_sendCmd(CMD16, 0x00000200, BSP_NO_POINTER) != 0x00)
            {
                type = 0;
            }
        }

        /* Check type */
        sdType = type;
        if (sdType)
        {
            /* Initialization succeded */
            bsp_spiSd_speed(SD_SPEED_HIGH);
        }
        else
        {
            break;
        }

        /* Allocate buffer */
        if ((buf_res = rt_malloc(SD_SECTOR_SIZE)) == RT_NULL)
        {
            sdcard_debug("SPISD: No memory for sector buffer\n");
            break;
        }
        /* Read the first sector for partition table */
        if (dev->read(dev, 0, buf_res, 1) != 1)
        {
            sdcard_debug("SPISD: Read first sector failed!\n");
            break;
        }
        /* Fetch the partition table */
        if (dfs_filesystem_get_partition(&sdPart, buf_res, 0) != RT_EOK)
        {
            sdPart.offset = 0;
            sdPart.size = 0;
            sdcard_debug("SPISD: No partition table\n");
        }
        /* Release buffer */
        rt_free(buf_res);
        sdcard_debug("SPISD: Init OK, card type %x\n", sdType);
        return RT_EOK;
    } while (0);

    /* Release buffer */
    if (buf_res)
    {
        rt_free(buf_res);
    }
    bsp_spiSd_deinit();
    rt_kprintf("SPISD err: Init failed!\n");
    return -RT_ERROR;
}

/***************************************************************************//**
 * @brief
 *   Open memory card device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] oflag
 *   Device open flag
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t rt_spiSd_open(rt_device_t dev, rt_uint16_t oflag)
{
    sdcard_debug("SPISD: Open, flag %x\n", sd_dev.flag);
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Close memory card device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @return
 *   Error code
 ******************************************************************************/
static rt_err_t rt_spiSd_close(rt_device_t dev)
{
    sdcard_debug("SPISD: Close, flag %x\n", sd_dev.flag);
    return RT_EOK;
}

/***************************************************************************//**
 * @brief
 *   Read from memory card device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] sector
 *   Start sector number (LBA)
 *
 * @param[in] buffer
 *   Pointer to the buffer
 *
 * @param[in] count
 *   Sector count (1..255)
 *
 * @return
 *   Number of read sectors
 ******************************************************************************/
static rt_size_t rt_spiSd_read(
    rt_device_t     dev,
    rt_off_t        sector,
    void            *buffer,
    rt_size_t       count)
{
    rt_uint8_t buf_ins[11], buf_res[12];
    rt_uint8_t *ptr;
    rt_uint8_t cmd, i;
    rt_size_t cnt;

    ptr = (rt_uint8_t *)buffer;
    cnt = count;

    sdcard_debug("SPISD: ****** Read Data ******\n");
    if (!(sdType & CT_BLOCK))
    {
        /* Convert to byte address if needed */
        sector *= SD_SECTOR_SIZE;
    }

    do
    {
        if (cnt == 1)
        {
            /* Single block read */
            cmd = CMD17;
            sdcard_debug("SPISD: Read single block\n");
        }
        else
        {
            /* Multiple block read */
            cmd = CMD18;
            sdcard_debug("SPISD: Read multiple blocks\n");
        }

        if (bsp_spiSd_sendCmd(cmd, sector, BSP_NO_POINTER))
        {
            sdcard_debug("SPISD: Read command error!\n");
            break;
        }

        /* Read data */
        do
        {
            if (bsp_spiSd_readBlock(ptr, SD_SECTOR_SIZE))
            {
                break;
            }
            ptr += SD_SECTOR_SIZE;
        } while(--cnt);

        /* Stop transmission */
        if (cmd == CMD18)
        {
            if (bsp_spiSd_sendCmd(CMD12, 0x00000000, BSP_NO_POINTER))
            {
                break;
            }
        }

        return (count);
    } while(0);

    return (0);
}

/***************************************************************************//**
 * @brief
 *   Write to memory card device
 *
 * @details
 *
 * @note
 *
 * @param[in] dev
 *   Pointer to device descriptor
 *
 * @param[in] sector
 *   Start sector number (LBA)
 *
 * @param[in] buffer
 *   Pointer to the buffer
 *
 * @param[in] count
 *   Sector count (1..255)
 *
 * @return
 *   Number of written sectors
 ******************************************************************************/
static rt_size_t rt_spiSd_write (
    rt_device_t     dev,
    rt_off_t        sector,
    const void      *buffer,
    rt_size_t       count)
{
    rt_uint8_t buf_ins[11], buf_res[12];
    rt_uint8_t *ptr;
    rt_uint8_t cmd, token, i;
    rt_size_t cnt;

    ptr = (rt_uint8_t *)buffer;
    cnt = count;

    sdcard_debug("SPISD: ****** Write Data ******\n");
    if (!(sdType & CT_BLOCK))
    {
        /* Convert to byte address if needed */
        sector *= SD_SECTOR_SIZE;
    }

    do
    {
        if (cnt == 1)
        {
            /* Single block write */
            cmd = CMD24;
            token = 0xfe;
            sdcard_debug("SPISD: Write single block\n");
        }
        else
        {
            /* Multiple block write */
            cmd = CMD25;
            token = 0xfc;
            sdcard_debug("SPISD: Write multiple blocks\n");
            if (sdType & CT_SDC)
            {
                if (bsp_spiSd_sendCmd(ACMD23, count, BSP_NO_POINTER))
                {
                    break;
                }
            }
        }

        if (bsp_spiSd_sendCmd(cmd, sector, BSP_NO_POINTER))
        {
            sdcard_debug("SPISD: Write command error!\n");
            break;
        }

        /* Write data */
        do
        {
            if (bsp_spiSd_writeBlock(ptr, token))
            {
                break;
            }
            ptr += SD_SECTOR_SIZE;
        } while(--cnt);

        /* Stop transmission token */
        if (bsp_spiSd_writeBlock(BSP_NO_POINTER, 0xfd))
        {
            break;
        }

        return (count);
    } while(0);

    return (0);
}

/***************************************************************************//**
* @brief
*   Configure memory card device
*
* @details
*
* @note
*
* @param[in] dev
*   Pointer to device descriptor
*
* @param[in] ctrl
*   Memory card control command
*
* @param[in] buffer
*   Pointer to the buffer of in/out data
*
* @return
*   Error code
******************************************************************************/
static rt_err_t rt_spiSd_control (
    rt_device_t     dev,
    rt_uint8_t      ctrl,
    void            *buffer)
{
    rt_err_t ret;
    rt_uint32_t c_size;
    rt_uint8_t n;
    rt_uint8_t *buf_res;

    ret = -RT_ERROR;
    buf_res = RT_NULL;
    switch (ctrl)
    {
    case RT_DEVICE_CTRL_BLK_SYNC:
        /* Flush dirty buffer if present */
        bsp_spiSd_cs(1);
        bsp_spiSd_cs(0);
        ret = RT_EOK;
        break;

    case RT_DEVICE_CTRL_BLK_GETGEOME:
        {
            struct rt_device_blk_geometry *geometry = buffer;
            
            /* Allocate buffer */
            if ((buf_res = rt_malloc(SD_BLOCK_SIZE_CSD)) == RT_NULL)
            {
                sdcard_debug("SPISD: No memory for RX buffer\n");
                break;
            }
            /* Get number of sectors on the disk (32 bits) */
            if (bsp_spiSd_sendCmd(CMD9, 0x00000000, buf_res))
            {
                sdcard_debug("SPISD: Get CSD failed!\n");
                break;
            }

            if ((buf_res[0] >> 6) == 0x01)
            {
                /* SDv2 */
                /* C_SIZE: Bit 48~69 */
                c_size = ((rt_uint32_t)(buf_res[7] & 0x3f) << 16) + \
                    ((rt_uint32_t)buf_res[8] << 8) + buf_res[9] + 1;
                /* Result = Capacity / Sector Size */
                geometry->sector_count = (rt_uint32_t)c_size << \
                    (19 - SD_SECTOR_SIZE_SHIFT);
            }
            else
            {
                /* SDv1 or MMC */
                /* C_SIZE: Bit 62~73 */
                c_size = ((rt_uint32_t)(buf_res[6] & 0x03) << 10) + \
                    ((rt_uint16_t)buf_res[7] << 2) + (buf_res[8] >> 6) + 1;
                /* READ_BL_LEN: Bit 80~83, C_SIZE_MULT: Bit 47~49 */
                n = ((buf_res[9] & 0x03) << 1) + ((buf_res[10] & 0x80) >> 7) + \
                    2 + (buf_res[5] & 0x0f);
                /* Result = Capacity / Sector Size */
                geometry->sector_count = (rt_uint32_t)c_size << \
                    (n - SD_SECTOR_SIZE_SHIFT);
            }

            /* Get sector size */
            geometry->bytes_per_sector = SD_SECTOR_SIZE;
            
            /* Get erase block size in unit of sectors (32 bits) */
            if (sdType & CT_SD2)
            {
                /* Allocate buffer */
                if ((buf_res = rt_malloc(SD_BLOCK_SIZE_SDSTAT)) == RT_NULL)
                {
                    sdcard_debug("SPISD: No memory for RX buffer\n");
                    break;
                }
                /* SDv2 */
                if (bsp_spiSd_sendCmd(ACMD13, 0x00000000, BSP_NO_POINTER))
                {
                    sdcard_debug("SPISD: Get SD status failed!\n");
                    break;
                }
                if (bsp_spiSd_readBlock(buf_res, SD_BLOCK_SIZE_SDSTAT))
                {
                    sdcard_debug("SPISD: Read SD status failed!\n");
                    break;
                }
                /* AU_SIZE: Bit 428~431 */
                geometry->block_size = 16UL << ((buf_res[10] >> 4) + 9 - \
                    SD_SECTOR_SIZE_SHIFT);
            }
            else
            {
                /* Allocate buffer */
                if ((buf_res = rt_malloc(SD_BLOCK_SIZE_CSD)) == RT_NULL)
                {
                    sdcard_debug("SPISD: No memory for RX buffer\n");
                    break;
                }
                /* SDv1 or MMC */
                if (bsp_spiSd_sendCmd(CMD9, 0x00000000, buf_res))
                {
                    sdcard_debug("SPISD: Get CSD failed!\n");
                    break;
                }

                if (sdType & CT_SD1)
                {
                    /* SECTOR_SIZE: Bit 39~45, WRITE_BL_LEN: Bit 22~25 (9, 10 or 11) */
                    geometry->block_size = (((buf_res[10] & 0x3f) << 1) + \
                        ((rt_uint32_t)(buf_res[11] & 0x80) >> 7) + 1) << \
                        (8 + (buf_res[13] >> 6) - SD_SECTOR_SIZE_SHIFT);
                }
                else
                {
                    /* ERASE_GRP_SIZE: Bit 42~46, ERASE_GRP_MULT: Bit 37~41 */
                    geometry->block_size = \
                        ((rt_uint16_t)((buf_res[10] & 0x7c) >> 2) + 1) * \
                        (((buf_res[10] & 0x03) << 3) + \
                        ((buf_res[11] & 0xe0) >> 5) + 1);
                }
            }
            ret = RT_EOK;
            break;
        }

    case RT_DEVICE_CTRL_BLK_ERASE:
        // TODO 
        ret = RT_ERROR;
        break;

    default:
        break;
    }

    if (buf_res)
    {
        rt_free(buf_res);
    }
    return ret;
}

/***************************************************************************//**
* @brief
*   Initialize all memory card related hardware and register the device to
*  kernel
*
* @details
*
* @note
*
* @return
*   Error code
******************************************************************************/
rt_err_t bsp_hw_spiSd_init(void)
{
    do
    {
        /* Find SPI device */
        spi_dev = rt_device_find(SPISD_USING_DEVICE_NAME);
        if (spi_dev == RT_NULL)
        {
            sdcard_debug("SPISD: Can't find device %s!\n",
                SPISD_USING_DEVICE_NAME);
            break;
        }
        sdcard_debug("SPISD: Find device %s\n", SPISD_USING_DEVICE_NAME);

#if (defined(SD_CS_PORT) && defined(SD_CS_PIN))
        /* Config chip slect pin */
        {
            GPIO_InitTypeDef gpio_init;

            RCC_APB2PeriphClockCmd(SD_CS_CLOCK, ENABLE);
            gpio_init.GPIO_Pin          = SD_CS_PIN;
            gpio_init.GPIO_Speed        = GPIO_Speed_50MHz;
            gpio_init.GPIO_Mode         = GPIO_Mode_Out_PP;
            GPIO_Init(SD_CS_PORT, &gpio_init);
            GPIO_SetBits(SD_CS_PORT, SD_CS_PIN);
        }
#endif
        /* Register SPI SD device */
        sd_dev.type      = RT_Device_Class_MTD;
        sd_dev.init      = rt_spiSd_init;
        sd_dev.open      = rt_spiSd_open;
        sd_dev.close     = rt_spiSd_close;
        sd_dev.read      = rt_spiSd_read;
        sd_dev.write     = rt_spiSd_write;
        sd_dev.control   = rt_spiSd_control;
        sd_dev.user_data = RT_NULL;
        if (rt_device_register(
            &sd_dev,
            SPISD_DEVICE_NAME,
            RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_REMOVABLE | RT_DEVICE_FLAG_STANDALONE
            ) != RT_EOK)
        {
            break;
        }
        sdcard_debug("SPISD: HW init OK\n");
        return RT_EOK;
    } while (0);

    rt_kprintf("SPISD: HW init failed!\n");
    return -RT_ERROR;
}

/*******************************************************************************
 *  Export to FINSH
 ******************************************************************************/
#ifdef RT_USING_FINSH
#include <finsh.h>

void list_sd(void)
{
    rt_uint8_t i, buf_res[16];
    rt_uint32_t temp;
    struct sd_register_cid *cid = buf_res;
    struct rt_device_blk_geometry geometry;

    /* Receive CID as a data block (16 bytes) */
    if (bsp_spiSd_sendCmd(CMD10, 0x00000000, buf_res))
    {
        rt_kprintf("SPISD: Get CID failed!\n");
        return;
    }

    rt_kprintf("    SD Card on %s\n", SPISD_USING_DEVICE_NAME);
    rt_kprintf(" ------------------------------\n");
    rt_kprintf(" Manufacturer ID:\t%X\n", cid->man_id);
    rt_kprintf(" OEM/Application ID:\t%c%c\n", cid->app_id[0], cid->app_id[1]);
    buf_res[13] = 0;
    rt_kprintf(" Product name:\t\t%c%c%c%c%c\n", cid->name[0], cid->name[1], 
        cid->name[2], cid->name[3], cid->name[4]);
    rt_kprintf(" Product revision:\t%X.%X\n", (cid->rev & 0xF0) >> 4, 
        (cid->rev & 0x0F));
    temp = ((rt_uint32_t)cid->sn[0] << 24) + ((rt_uint32_t)cid->sn[1] << 16) + \
        ((rt_uint32_t)cid->sn[2] << 8) + (rt_uint32_t)cid->sn[3];
    rt_kprintf(" Serial number:\t\t%X\n", temp);
    rt_kprintf(" Manufacturing date:\t%d.%d\n", \
        2000 + ((cid->date[0] & 0x0F) * 10) + ((cid->date[1] & 0xF0) >> 4), \
        cid->date[1] & 0x0F);
    
    rt_kprintf(" Card type:\t\t");
    if (sdType & CT_MMC)
    {
        rt_kprintf("%s\n", "MMC");
    }
    else if (sdType & CT_SDC)
    {
        rt_kprintf("%s\n", "SDXC");
    }
    else if (sdType & CT_SD1)
    {
        rt_kprintf("%s\n", "SDSC");
    }
    else if (sdType & CT_SD2)
    {
        rt_kprintf("%s\n", "SDHC");
    }
	else
	{
		rt_kprintf("0x%X\n", sdType);
	}
    
    sd_dev.control(&sd_dev, RT_DEVICE_CTRL_BLK_GETGEOME, &geometry);
    temp = ((geometry.sector_count & 0x0000FFFF) * geometry.bytes_per_sector) >> 16;
    temp += ((geometry.sector_count >> 16) * geometry.bytes_per_sector);
    temp >>= 4;
    rt_kprintf(" Card capacity:\t\t%dMB\n", temp);
}
FINSH_FUNCTION_EXPORT(list_sd, list the SD card.)
#endif

#endif /* defined(BSP_USING_SPISD) */
/***************************************************************************//**
 * @}
 ******************************************************************************/
