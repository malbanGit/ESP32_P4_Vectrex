//////////////////////////// SD Example stuff
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"


#if SOC_SDMMC_IO_POWER_EXTERNAL
#include "sd_pwr_ctrl_by_on_chip_ldo.h"
// YES error
#endif

#define EXAMPLE_MAX_CHAR_SIZE    64
#define MOUNT_POINT "/sdcard"
#define EXAMPLE_IS_UHS1    (CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50 || CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50 || CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR104)

////////////////////////////

/* SD Card debug output

I (1055) vectrex_example: Initializing SD card
I (1059) vectrex_example: Using SDMMC peripheral
CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
W (1068) ldo: The voltage value 0 is out of the recommended range [500, 2700]
CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
I (1178) vectrex_example: Mounting filesystem
I (1328) vectrex_example: Filesystem mounted
Name: USDU1
Type: SDHC
Speed: 20.00 MHz (limit: 20.00 MHz)
Size: 15223MB
CSD: ver=2, sector_size=512, capacity=31176704 read_bl_len=9
SSR: bus_width=4
I (1335) vectrex_example: Opening file /sdcard/hello.txt
I (1364) vectrex_example: File written
File Write Ok!
I (1364) vectrex_example: Reading file /sdcard/hello.txt
I (1368) vectrex_example: Read from file: 'Hello USDU1!'
File Read Ok!
I (1370) vectrex_example: Card unmounted
I (1373) vectrex_example: Init LCD / DSI
I (1377) vectrex_example: Create VSYNC semaphore
I (1381) vectrex_example: Create LCD via BSP (Waveshare P4 NANO)
W (1387) i2c.master: Please check pull-up resistances whether be connected properly. Otherwise unexpected behavior would happen. For more detailed information, please read docs
I (1402) ESP32_P4_EV: MIPI DSI PHY Powered on
I (1407) ESP32_P4_EV: Install MIPI DSI LCD control panel
I (1411) ESP32_P4_EV: Install Waveshare LCD control panel
*/
/*

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}
    */
#if CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
static esp_err_t s_example_reset_card_power(void)
{
    esp_err_t ret = ESP_FAIL;
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL<<CONFIG_EXAMPLE_PIN_CARD_POWER_RESET),
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config GPIO");
        return ret;
    }

    ret = gpio_set_level(CONFIG_EXAMPLE_PIN_CARD_POWER_RESET, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ret;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    ret = gpio_set_level(CONFIG_EXAMPLE_PIN_CARD_POWER_RESET, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set GPIO level");
        return ret;
    }

    return ESP_OK;
}
#endif // CONFIG_EXAMPLE_PIN_CARD_POWER_RESET

sdmmc_card_t *card;
const char mount_point[] = MOUNT_POINT;
void unmountSD()
{
    // All done, unmount partition and disable SDMMC peripheral
    esp_vfs_fat_sdcard_unmount(mount_point, card);
    ESP_LOGI(TAG, "Card unmounted");
}

esp_err_t initSD()
{
    esp_err_t ret;

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    ESP_LOGI(TAG, "Initializing SD card");
    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
    // Please check its source code and implement error recovery when developing
    // production applications.

    ESP_LOGI(TAG, "Using SDMMC peripheral");



    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    
#if CONFIG_EXAMPLE_SDMMC_SPEED_HS
printf("CONFIG_EXAMPLE_SDMMC_SPEED_HS\n");
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50
printf(" CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR50\n");
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR50;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50
printf(" CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_DDR50\n");
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_DDR50;
#elif CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR104
printf("CONFIG_EXAMPLE_SDMMC_SPEED_UHS_I_SDR104\n");
    host.slot = SDMMC_HOST_SLOT_0;
    host.max_freq_khz = SDMMC_FREQ_SDR104;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
#endif

// malban:
// changed from default (1) to 0 - since conflict with BlueTooth (C6)
host.slot = SDMMC_HOST_SLOT_0;

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
printf("CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO\n");
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif
#if CONFIG_EXAMPLE_PIN_CARD_POWER_RESET
printf("CONFIG_EXAMPLE_PIN_CARD_POWER_RESET\n");
    ESP_ERROR_CHECK(s_example_reset_card_power());
#endif

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // Set bus width to use:
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
printf("CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4\n");
    slot_config.width = 4;
#else
    slot_config.width = 1;
#endif

    // On chips where the GPIOs used for SD card can be configured, set them in
    // the slot_config structure:
#ifdef CONFIG_SOC_SDMMC_USE_GPIO_MATRIX
printf("CONFIG_SOC_SDMMC_USE_GPIO_MATRIX\n");
    slot_config.clk = CONFIG_EXAMPLE_PIN_CLK;
    slot_config.cmd = CONFIG_EXAMPLE_PIN_CMD;
    slot_config.d0 = CONFIG_EXAMPLE_PIN_D0;
#ifdef CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
printf("CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4\n");
    slot_config.d1 = CONFIG_EXAMPLE_PIN_D1;
    slot_config.d2 = CONFIG_EXAMPLE_PIN_D2;
    slot_config.d3 = CONFIG_EXAMPLE_PIN_D3;
#endif  // CONFIG_EXAMPLE_SDMMC_BUS_WIDTH_4
#endif  // CONFIG_SOC_SDMMC_USE_GPIO_MATRIX

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) 
    {
        if (ret == ESP_FAIL) 
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files:
/*
    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    ret = s_example_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return ret;
    }
    printf("File Write Ok!\n");

    ret = s_example_read_file(file_hello);
    if (ret != ESP_OK) {
        return ret;
    }
    printf("File Read Ok!\n");

	// unmountSD();
    // All done, unmount partition and disable SDMMC peripheral
*/    
    return ESP_OK;
}


