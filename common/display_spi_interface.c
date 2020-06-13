#include "esp_log.h"
#include "display_spi_interface.h"
#include "driver/spi_master.h"
#include "string.h"

#define DMA_CHAN            2

#if defined(CONFIG_DISPLAY_3WIRE_SPI)
/*
 * 3-wire SPI is 9-bit SPI where the 9th
 * bit is the data/~command bit.
 * It takes 27-SPI bits per pixel.
 * 
 * It needs 8-pixels to make it byte-align.
 * That's 216 bits or 27 bytes.
 */
#define BITS_PER_BYTE           (9)
#define BITS_PER_PIXEL          (3 * BITS_PER_BYTE)
#define BUF_MIN_SIZE            (BITS_PER_PIXEL)
#define PIXEL_IN_MIN_BUF        (8)
#define MAX_CMD_PARAM_COUNT     (15)
#define CMD_BUF_SIZE            ((uint32_t)((MAX_CMD_PARAM_COUNT * BITS_PER_BYTE)/8) + 1)

#elif defined(CONFIG_DISPLAY_4WIRE_SPI)
#else
#error "Invalid SPI Config"
#endif
/*
 * Number of pixels the buffer can hold
 */
#define PIXEL_IN_BUF        (CONFIG_DISPLAY_PARALLEL_LINE_COUNT * CONFIG_DISPLAY_H_RES)

/*
 * DMA Buffer size to hold Pixel Buffer
 */
#define PIXEL_BUFF_SIZE     ((PIXEL_IN_BUF * BUF_MIN_SIZE) / PIXEL_IN_MIN_BUF)

typedef struct {
    union {
        uint32_t raw;
        uint8_t rawByte[4];
    };
} pixel_t;

static const char * TAG = "display_spi_interface";
static spi_device_handle_t spi_device_handle;
DMA_ATTR static uint8_t pixelDmaBuf[PIXEL_BUFF_SIZE];
DMA_ATTR static uint8_t cmdDmaBuf[CMD_BUF_SIZE];
static bool bInit = false;

#if defined(CONFIG_DISPLAY_3WIRE_SPI)
/*
 * Convert from rgb_t to 27-bit
 */
static esp_err_t conv_buf_rgb_to_27b(rgb_t *pBuf_rgb, uint8_t *pBuf27, uint32_t nPx, uint32_t nBuf27)
{
    uint32_t i = 0;
    uint32_t j = 0;
    pixel_t px;
    uint32_t byte_offset = 0;
    uint8_t bit_offset = 0;
    uint8_t byteToInsert = 0;
    uint8_t red;
    uint8_t green;
    uint8_t blue;

    if(((rgb_t *)0 == pBuf_rgb) || ((uint8_t *)0 == pBuf27)) {
        return ESP_ERR_INVALID_ARG;
    }

    if(nPx == 0) {
        /* Nothing to convert */
        return ESP_OK;
    }

    /* Clear Buffer */
    memset((void*)pBuf27, 0, nBuf27);
    for(i = 0; i < nPx; i++) {
#ifdef CONFIG_DISPLAY_COLOR_RGB565
        red = pBuf_rgb[i].red << 3;
        green = pBuf_rgb[i].green << 2;
        blue = pBuf_rgb[i].blue << 3;
#elif defined(CONFIG_DISPLAY_COLOR_ARGB8888)
        red = pBuf_rgb[i].red;
        green = pBuf_rgb[i].green;
        blue = pBuf_rgb[i].blue;
#endif
        /* translate to 27-bit SPI with d/c bit set */
        px.raw = (1UL << 26) + (((uint32_t)(red)) << 18) +
                (1UL << 17) + (((uint32_t)(green)) << 9) +
                (1UL << 8) + ((uint32_t)(blue));
        px.raw = SPI_SWAP_DATA_TX(px.raw, 27);
        /* Calculate offset to insert the px data */
        byte_offset = (i * BITS_PER_PIXEL) / 8;
        bit_offset = (i * BITS_PER_PIXEL) % 8;
        /* insert px data */
        byteToInsert = px.rawByte[0] >> bit_offset;
        for(j = 0; j < 4; j++) {
            pBuf27[byte_offset + j] |= byteToInsert;
            byteToInsert = (uint8_t)((px.rawByte[j] << (8-bit_offset)));
            if(j < 3) {
                /* prevent out-of-bound array access */
                byteToInsert += (uint8_t)((px.rawByte[j+1] >> bit_offset)); 
            }
        }
    }

    return ESP_OK;
}

/*
 * Convert from 8-bit to 9-bit (D/C set + byte)
 */
static esp_err_t conv_buf_8b_to_9b(uint8_t *pBuf8, uint8_t *pBuf9, uint32_t nBuf8, uint32_t nBuf9)
{
    uint32_t i = 0;
    uint8_t dataField[2];
    uint32_t byte_offset = 0;
    uint8_t bit_offset = 0;
    uint8_t byteToInsert = 0;

    if(((uint8_t *)0 == pBuf8) || ((uint8_t *)0 == pBuf9)) {
        ESP_LOGE(TAG, "%s:%d\n", __FUNCTION__, __LINE__);
        return ESP_ERR_INVALID_ARG;
    }

    if(nBuf9 < ((uint32_t)(nBuf8 * BITS_PER_BYTE / 8) + 1)) {
        ESP_LOGE(TAG, "%s: Cannot fit to buffer!", __func__);
        return ESP_ERR_INVALID_SIZE;
    }

    if(nBuf8 != 0) {
        /* Clear Buffer */
        memset((void *)pBuf9, 0, nBuf9);

        for (i = 0; i < nBuf8; i++) {
            /* translate to 9-bit SPI with d/c bit set */
            dataField[0] = 0x80 | ((pBuf8[i] >> 1) & 0x7F);
            dataField[1] = (uint8_t)((pBuf8[i] & 0x01) << 7);
            /* Calculate offset to insert the data */
            byte_offset = (i * BITS_PER_BYTE) / 8;
            bit_offset = (i * BITS_PER_BYTE) % 8;
            /* insert px data */
            byteToInsert = dataField[0] >> bit_offset;
            pBuf9[byte_offset] |= byteToInsert;
            byteToInsert = (dataField[0] << (8-bit_offset)) + 
                        (dataField[1] >> bit_offset);
            pBuf9[byte_offset + 1] = byteToInsert;
        }
    }

    return ESP_OK;
}
#endif /* defined(CONFIG_DISPLAY_3WIRE_SPI) */

esp_err_t display_interface_init(void)
{
    esp_err_t ret = ESP_OK;
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_DISPLAY_SPI_SDI_IO_PIN,
        .mosi_io_num = CONFIG_DISPLAY_SPI_SDO_IO_PIN,
        .sclk_io_num = CONFIG_DISPLAY_SPI_CLK_IO_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = PIXEL_BUFF_SIZE
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = CONFIG_DISPLAY_SPI_CLK_FREQ,
        .mode = 0,
        .spics_io_num = CONFIG_DISPLAY_SPI_CS_IO_PIN,
#if defined(CONFIG_DISPLAY_3WIRE_SPI)
        .flags = SPI_DEVICE_3WIRE | SPI_DEVICE_HALFDUPLEX,
#elif defined(CONFIG_DISPLAY_4WIRE_SPI)
#endif
        .queue_size = 7
    };

    /*
     * Initialize SPI bus
     */
    ret = spi_bus_initialize(CONFIG_DISPLAY_SPI_HOST_DEV, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(CONFIG_DISPLAY_SPI_HOST_DEV, &devcfg, &spi_device_handle);
    ESP_ERROR_CHECK(ret);

    bInit = true;    
    return ESP_OK;
}

esp_err_t display_interface_write_px(rgb_t *pBuf, uint32_t nPixel)
{
    esp_err_t ret = ESP_OK;
    static spi_transaction_t t;
    spi_transaction_t * pTrans = &t;

    if(nPixel == 0) {
        /* Nothing to write */
        return ESP_OK;
    }

    if((rgb_t *)0 == pBuf) {
        return ESP_ERR_INVALID_ARG;
    }

    if(nPixel > PIXEL_IN_BUF) {
        return ESP_ERR_INVALID_SIZE;
    }

    if(bInit != true) {
        return ESP_ERR_INVALID_STATE;
    }

#if defined(CONFIG_DISPLAY_3WIRE_SPI)
    /* Translate to 27bit/pixel SPI Format */
    ret = conv_buf_rgb_to_27b(pBuf, pixelDmaBuf, nPixel, PIXEL_BUFF_SIZE);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }
#endif
    memset(&t, 0, sizeof(spi_transaction_t));
#if defined(CONFIG_DISPLAY_3WIRE_SPI)
    t.length = nPixel * BITS_PER_PIXEL;
    t.tx_buffer = pixelDmaBuf;
#elif defined(CONFIG_DISPLAY_4WIRE_SPI)
    /// TODO
#endif

    ret = spi_device_queue_trans(spi_device_handle, &t, portMAX_DELAY);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }
    ret = spi_device_get_trans_result(spi_device_handle, &pTrans, portMAX_DELAY);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }
    return ESP_OK;
}

esp_err_t display_interface_write_command(uint8_t cmd, uint8_t *pBuf, uint32_t nBuf)
{
    static spi_transaction_ext_t t_cmd;
    spi_transaction_t * pTrans = &(t_cmd.base);
    esp_err_t ret = ESP_OK;

    if(((uint8_t *)0 == pBuf) && (nBuf > 0)) {
        return ESP_ERR_INVALID_ARG;
    }
    if(bInit != true) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(&t_cmd, 0, sizeof(spi_transaction_ext_t));
    t_cmd.base.flags = SPI_TRANS_VARIABLE_CMD;
    t_cmd.base.cmd = 0x00FF & cmd;
#if defined(CONFIG_DISPLAY_3WIRE_SPI)
    t_cmd.command_bits = 9;
    t_cmd.base.length = nBuf * 9;
#elif defined(CONFIG_DISPLAY_4WIRE_SPI)
    t_cmd.command_bits = 8;
    t_cmd.base.length = nBuf * 8;
#endif

    if(nBuf != 0) {
#if defined(CONFIG_DISPLAY_3WIRE_SPI)
        ret = conv_buf_8b_to_9b(pBuf, cmdDmaBuf, nBuf, CMD_BUF_SIZE);
        t_cmd.base.tx_buffer = cmdDmaBuf;
#elif defined(CONFIG_DISPLAY_4WIRE_SPI)
#endif
        if(ESP_OK != ret) {
            ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
            return(ret);
        }
    } else {
        t_cmd.base.tx_buffer = (void *)0;
    }

    ret = spi_device_queue_trans(spi_device_handle, &(t_cmd.base), portMAX_DELAY);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }

    ret = spi_device_get_trans_result(spi_device_handle, &pTrans, portMAX_DELAY);
    if(ESP_OK != ret) {
        ESP_LOGE(TAG, "%s:%d:Err%d\n", __FUNCTION__, __LINE__, ret);
        return(ret);
    }

    return ESP_OK;
}