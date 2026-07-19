#include "defines.h"


#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_err.h"

#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"

#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "es8311.h"

/* Optional: PCM-Daten aus Linker (wie im Beispiel) */
//extern const uint8_t music_pcm_start[] asm("_binary_canon_pcm_start");
//extern const uint8_t music_pcm_end[]   asm("_binary_canon_pcm_end");



/* Example configurations (wie bei deinem bisherigen Code) */
// #define EXAMPLE_RECV_BUF_SIZE   (2400)
// #define EXAMPLE_SAMPLE_RATE     (16000)
#define EXAMPLE_SAMPLE_RATE     (AY_FREQUENCY)



//#define EXAMPLE_MCLK_MULTIPLE   (384) // If not using 24-bit data width, 256 should be enough
#define EXAMPLE_MCLK_MULTIPLE   (256) 
#define EXAMPLE_MCLK_FREQ_HZ    (EXAMPLE_SAMPLE_RATE * EXAMPLE_MCLK_MULTIPLE)
#define EXAMPLE_VOICE_VOLUME    (20)

/* I2C port and GPIOs (wie bisher) */
#define I2C_NUM         (0)
#define I2C_SCL_IO      (GPIO_NUM_8)
#define I2C_SDA_IO      (GPIO_NUM_7)
#define GPIO_OUTPUT_PA  (GPIO_NUM_53)

/* I2S port and GPIOs (wie bisher) */
#define I2S_NUM         (0)
#define I2S_MCK_IO      (GPIO_NUM_13)
#define I2S_BCK_IO      (GPIO_NUM_12)
#define I2S_WS_IO       (GPIO_NUM_10)
#define I2S_DO_IO       (GPIO_NUM_9)
#define I2S_DI_IO       (GPIO_NUM_11)

static const char *TAG_A = "audio";

/* Handles für neue Treiber / esp_codec_dev */
//static i2c_master_bus_handle_t      s_i2c_bus_handle = NULL;
#define s_i2c_bus_handle i2c_bus // done in main 
static i2s_chan_handle_t            s_i2s_tx_chan    = NULL;
static i2s_chan_handle_t            s_i2s_rx_chan    = NULL;
static const audio_codec_data_if_t *s_codec_data_if  = NULL;
static esp_codec_dev_handle_t       s_codec_dev      = NULL;

/* Power-Amp / Lautsprecher-Versorgung anschalten */
static void audio_gpio_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_OUTPUT_PA),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    /* PA default aus; es8311-Treiber steuert über pa_pin */
    gpio_set_level(GPIO_OUTPUT_PA, 0);
}

/* I2C-Bus mit neuem Treiber aufsetzen */
static esp_err_t audio_i2c_init(void)
{
    // done in init of main
    /*

    if (s_i2c_bus_handle) {
        return ESP_OK;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM,
        .sda_io_num = I2C_SDA_IO,
        .scl_io_num = I2C_SCL_IO,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus_handle),
                        TAG_A, "i2c_new_master_bus failed");
    */
    return ESP_OK;
}

/* I2S + audio_codec_new_i2s_data */
static esp_err_t audio_i2s_init(void)
{
    if (s_i2s_tx_chan && s_codec_data_if) {
        return ESP_OK;
    }

    /* I2S-Kanäle anlegen */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &s_i2s_tx_chan, &s_i2s_rx_chan),
                        TAG_A, "i2s_new_channel failed");

    /* Standard-I2S-Konfiguration */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(EXAMPLE_SAMPLE_RATE),

#if AY_CHANNEL==2
#if AY_BITS==16
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
#else
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_8BIT, I2S_SLOT_MODE_STEREO),
#endif         
#else
#if AY_BITS==16
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
#else
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_8BIT, I2S_SLOT_MODE_MONO),
#endif         
#endif
        .gpio_cfg = {
            .mclk = I2S_MCK_IO,
            .bclk = I2S_BCK_IO,
            .ws   = I2S_WS_IO,
            .dout = I2S_DO_IO,
            .din  = I2S_DI_IO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_tx_chan, &std_cfg),
                        TAG_A, "i2s_channel_init_std_mode(TX) failed");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_tx_chan),
                        TAG_A, "i2s_channel_enable(TX) failed");

    /* RX wird für reines Playback nicht benötigt, aber mit angelegt */
    if (s_i2s_rx_chan) {
        ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(s_i2s_rx_chan, &std_cfg),
                            TAG_A, "i2s_channel_init_std_mode(RX) failed");
        ESP_RETURN_ON_ERROR(i2s_channel_enable(s_i2s_rx_chan),
                            TAG_A, "i2s_channel_enable(RX) failed");
    }

    /* esp_codec_dev Daten-Interface */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port      = I2S_NUM,
        .tx_handle = s_i2s_tx_chan,
        .rx_handle = s_i2s_rx_chan,
    };
    s_codec_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    ESP_RETURN_ON_FALSE(s_codec_data_if, ESP_FAIL, TAG_A, "audio_codec_new_i2s_data failed");

    return ESP_OK;
}

/* ES8311 + esp_codec_dev initialisieren */
static esp_err_t audio_codec_init(void)
{
    if (s_codec_dev) {
        return ESP_OK;
    }

    /* I2C-Steuer-Interface für Codec */
    audio_codec_i2c_cfg_t i2c_cfg = {
        .port       = I2C_NUM,
        .addr       = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus_handle,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    ESP_RETURN_ON_FALSE(ctrl_if, ESP_FAIL, TAG_A, "audio_codec_new_i2c_ctrl failed");

    /* GPIO-Interface für PA usw. */
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    ESP_RETURN_ON_FALSE(gpio_if, ESP_FAIL, TAG_A, "audio_codec_new_gpio failed");

    /* Gain-Konfiguration (angepasst an dein Board) */
    esp_codec_dev_hw_gain_t gain = {
        .pa_voltage        = 5.0f,
        .codec_dac_voltage = 3.3f,
    };

    /* ES8311 Codec-Konfiguration */
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if     = ctrl_if,
        .gpio_if     = gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_DAC,   // nur Playback
        .pa_pin      = GPIO_OUTPUT_PA,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk    = true,
        .digital_mic = false,
        .invert_mclk = false,
        .invert_sclk = false,
        .hw_gain     = gain,
    };

    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    ESP_RETURN_ON_FALSE(codec_if, ESP_FAIL, TAG_A, "es8311_codec_new failed");

    /* esp_codec_dev-Device anlegen */
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
        .codec_if = codec_if,
        .data_if  = s_codec_data_if,
    };
    s_codec_dev = esp_codec_dev_new(&dev_cfg);
    ESP_RETURN_ON_FALSE(s_codec_dev, ESP_FAIL, TAG_A, "esp_codec_dev_new failed");

    /* Lautstärke und Sample-Format setzen */
    ESP_RETURN_ON_ERROR(esp_codec_dev_set_out_vol(s_codec_dev, (float)EXAMPLE_VOICE_VOLUME),
                        TAG_A, "set volume failed");
/*
    esp_codec_dev_sample_info_t fs = {
        .sample_rate     = EXAMPLE_SAMPLE_RATE,
        .channel         = 2,
        .bits_per_sample = 16,
    };
*/    
esp_codec_dev_sample_info_t fs = {
    .sample_rate     = EXAMPLE_SAMPLE_RATE,
    .channel         = AY_CHANNEL,         // <<< STEREO
    .bits_per_sample = AY_BITS,
};

ESP_RETURN_ON_ERROR(esp_codec_dev_open(s_codec_dev, &fs),
                        TAG_A, "esp_codec_dev_open failed");

    return ESP_OK;
}

/* Öffentliche Init-Funktion für dein Projekt */
esp_err_t audio_init(void)
{
    // Jul 26 assuming done in main!
    // audio_gpio_init();

    ESP_RETURN_ON_ERROR(audio_i2c_init(),   TAG_A, "audio_i2c_init failed");
    ESP_RETURN_ON_ERROR(audio_i2s_init(),   TAG_A, "audio_i2s_init failed");
    ESP_RETURN_ON_ERROR(audio_codec_init(), TAG_A, "audio_codec_init failed");

	int freq = AY_FREQUENCY;
	int chans = AY_CHANNEL;  /* 1=mono, 2=stereo */
	int bits = AY_BITS;  /* 16 or 8 bit */


	/* Allocate audio buffer for one AY frame */
	audio_bufsize = freq * chans * (bits >> 3);

	// that is the buf size for 1 second
	// if I want to generate 50 times per second, then divide by 50
	// 16 bits - but the buffer size is in bytes - not bits
	audio_bufsize /= 50; // 3528 4 bytes -> 882 "frames" per call

	if ((audio_buf = (char*) heap_caps_malloc((size_t)audio_bufsize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)) == NULL) 
	{
		printf ( "Can't allocate sound buffer\n");
		return ESP_OK;
	}
    void initAY(int freq, int chans, int bits); // libayemu
    initAY(freq, chans, bits); 

    return ESP_OK;
}

/* PCM-Daten schreiben (z.B. aus deinem canon_pcm-Array) */
esp_err_t audio_write(uint8_t *data, size_t len, size_t *bytes_written)
{
    if (!s_codec_dev) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t out = 0;
    esp_err_t ret = esp_codec_dev_write(s_codec_dev, (void *)data, len);
    if (ret == ESP_OK) {
        out = len;
    }
    if (bytes_written) {
        *bytes_written = out;
    }
    return ret;
}

/* Optional: Aufräumen */
void audio_deinit(void)
{
    if (s_codec_dev) {
        esp_codec_dev_close(s_codec_dev);
        esp_codec_dev_delete(s_codec_dev);
        s_codec_dev = NULL;
    }

    if (s_i2s_tx_chan) {
        i2s_channel_disable(s_i2s_tx_chan);
        i2s_del_channel(s_i2s_tx_chan);
        s_i2s_tx_chan = NULL;
    }
    if (s_i2s_rx_chan) {
        i2s_channel_disable(s_i2s_rx_chan);
        i2s_del_channel(s_i2s_rx_chan);
        s_i2s_rx_chan = NULL;
    }

    if (s_i2c_bus_handle) {
        i2c_del_master_bus(s_i2c_bus_handle);
        s_i2c_bus_handle = NULL;
    }

	if (audio_buf != NULL) 
	{
		free(audio_buf);
		audio_buf = NULL;
	}

}

//DRAM_ATTR int16_t audioBuffer[AUDIO_FRAME_SAMPLES];
/* Callback-Typ: mono, 16 Bit, length = Anzahl Samples */
typedef void (*audio_sample_callback_t)(void *userdata, int16_t *stream, int length);

/* Callback setzen (z. B. e8910_callback) */
void audio_set_callback(audio_sample_callback_t cb, void *userdata);
/* Callback-Mechanismus */
static audio_sample_callback_t s_audio_cb          = NULL;
static void                   *s_audio_cb_userdata = NULL;


void audio_set_callback(audio_sample_callback_t cb, void *userdata)
{
    s_audio_cb          = cb;
    s_audio_cb_userdata = userdata;
}
static TaskHandle_t s_audio_task_handle = NULL;
/* FreeRTOS-Task: spielt das eingebettete Musikstück in einer Endlosschleife ab */


/*
 * HDMI audio: the board's I2S pins (9-13) reach the LT8912B via the SN74AVC4T245
 * translator (BOARD_PIN_VCV_NOE). No second I2S channel is needed — we bypass
 * esp_codec_dev and write directly to the already-open s_i2s_tx_chan.
 *
 * The LT8912B audio input must be enabled via I2C register 0xB2 on the CEC bank.
 * Call audio_hdmi_enable_lt8912b() AFTER hdmi_start() has configured the bridge.
 */

/* Enable audio embedding in LT8912B (CEC I2C bank, reg 0xB2 bit0 = audio enable) */
static esp_err_t audio_hdmi_enable_lt8912b(const esp_lcd_panel_lt8912b_io_t *lt_io)
{
    /* LT8912B CEC bank reg 0xB2: bit0 enables I2S audio input */
    uint8_t val = 0x01;
    return esp_lcd_panel_io_tx_param(lt_io->cec_dsi, 0xB2, &val, 1);
}

/* Write mono PCM directly to the I2S TX channel, bypassing esp_codec_dev.
 * The same I2S frame reaches both ES8311 and LT8912B — in HDMI mode the
 * ES8311 PA pin (GPIO_OUTPUT_PA) should be low so the speaker stays silent. */
static esp_err_t audio_hdmi_write(const int16_t *mono, int samples)
{
    size_t written = 0;
    return i2s_channel_write(s_i2s_tx_chan,
                             mono,
                             (size_t)samples * sizeof(int16_t),
                             &written,
                             portMAX_DELAY);
}
