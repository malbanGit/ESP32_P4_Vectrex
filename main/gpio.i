static void configure_gpios(void)
{
    // Outputs
    #define BOARD_PIN_GPIO22 22
    #define BOARD_PIN_RESET_DISPLAY 23
    #define BOARD_PIN_TST_LED 26
    #define BOARD_PIN_CTRL_P3V_EN 27
    #define BOARD_PIN_BL_EN 34
    #define BOARD_PIN_MUTE_JACK 47
    #define BOARD_PIN_VCV_NOE 52
    #define BOARD_PIN_PA_CTRL_CPU 53

    /*
        GPIO22 22 (free)
        RESET_DISPLAY 23 (active low)
        TST_LED 26 (on when high)
        CTRL_P3V_EN 27 (on when high)
        BL_EN 34 (active high)
        MUTE_JACK 47   (mute audio jack when low. MUTE_JACK is to be controlled by software so that it is equal to "not PA_CTRL"). Use HPH_DET
        VCV_NOE 52 (active low. activate level translator 3V3<->1V8 between cpu and lt8912b)
        PA_CTRL_CPU 53 (audio PA active high, mute when 0)

        on start, we want: all at 0 except VCV_NOE bit 52
        0b00110000010000000000010000001100110000000000000000000000 0x3040040CC00000
        0b00010000000000000000000000000000000000000000000000000000 0x10000000000000
    */
    gpio_set_level(BOARD_PIN_VCV_NOE,1);
    //zero-initialize the config structure for basic outputs
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set uint64_t
    // 0b00110000010000000000010000001100110000000000000000000000
    // 0b00110000 01000000 00000100 00001100 11000000 00000000 00000000
    // io_conf.pin_bit_mask = 0x3040040CC00000;
    io_conf.pin_bit_mask =
    (1ULL << BOARD_PIN_GPIO22) |
    (1ULL << BOARD_PIN_RESET_DISPLAY) |
    (1ULL << BOARD_PIN_TST_LED) |
    (1ULL << BOARD_PIN_CTRL_P3V_EN) |
    (1ULL << BOARD_PIN_BL_EN) |
    (1ULL << BOARD_PIN_MUTE_JACK) |
    (1ULL << BOARD_PIN_VCV_NOE) |
    (1ULL << BOARD_PIN_PA_CTRL_CPU);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);


    #define BOARD_PIN_FAULT_PROTECT_HDMI 20
    #define BOARD_PIN_LT_INT 21
    #define BOARD_PIN_VIDEO_MODE 28
    #define BOARD_PIN_SOFT_RESET 29
    #define BOARD_PIN_CTRL_SW1 30
    #define BOARD_PIN_CTRL_SW2 31
    #define BOARD_PIN_CTRL_SW3 32
    #define BOARD_PIN_CTRL_SW4 33
    #define BOARD_PIN_CEC_IC 45
    #define BOARD_PIN_HPH_DET_P 46
    #define BOARD_PIN_SD_DET 48

    /*
        ALL Inputs are ALL with pull ups

        FAULT_PROTECT_HDMI 20 (active low, activate pull-up)
        LT_INT 21 // activate pull-up
        VIDEO_MODE 28 (switch activate pull-up)
        SOFT_RESET 29 (switch activate pull-up)
        CTRL_SW1 30 (switch activate pull-up)
        CTRL_SW2 31 (switch activate pull-up)
        CTRL_SW3 32 (switch activate pull-up)
        CTRL_SW4 33 (switch activate pull-up)
        CEC_IC 45 (hdmi bidirectional use, configured as input for low risk. activate pull-up. if output, USE OPEN DRAIN mode with pull up)
        HPH_DET_P 46 (head phones jack insertion detection --  activate pull-up)
        SD_DET 48 (active low, sdcard insertion detection -- activate pull-up)

        all pull ups active it seems.
        0b000000001 01100000 00000011 11110000 00110000 00000000 00000000 0x016003F0300000

    */
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set uint64_t
    // 0b00110000010000000000010000001100110000000000000000000000
    // 0b00110000 01000000 00000100 00001100 11000000 00000000 00000000
    // io_conf.pin_bit_mask = 0x016003F0300000;
    io_conf.pin_bit_mask =
    (1ULL << BOARD_PIN_FAULT_PROTECT_HDMI) |
    (1ULL << BOARD_PIN_LT_INT) |
    (1ULL << BOARD_PIN_VIDEO_MODE) |
    (1ULL << BOARD_PIN_SOFT_RESET) |
    (1ULL << BOARD_PIN_CTRL_SW1) |
    (1ULL << BOARD_PIN_CTRL_SW2) |
    (1ULL << BOARD_PIN_CTRL_SW3) |
    (1ULL << BOARD_PIN_CTRL_SW4) |
    (1ULL << BOARD_PIN_CEC_IC) |
    (1ULL << BOARD_PIN_HPH_DET_P) |
    (1ULL << BOARD_PIN_SD_DET);

    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}

IRAM_ATTR static void p3v_en(void)
{
	gpio_set_level(BOARD_PIN_CTRL_P3V_EN, 1);
}

IRAM_ATTR int getGPIOSwitches(void) // includes 9 pin if available
{
    mini9PinButton = 0xf;// non pressed
//    ESP_LOGI(TAG, "%d%d%d%d%d%d%d%d\n", 
    mini9PinButton  =   gpio_get_level(BOARD_PIN_CTRL_SW1)*0x1
                  + gpio_get_level(BOARD_PIN_CTRL_SW2)*0x2
                  + gpio_get_level(BOARD_PIN_CTRL_SW3)*0x4
                  + gpio_get_level(BOARD_PIN_CTRL_SW4)*0x8;
    miniVideoButton = gpio_get_level(BOARD_PIN_VIDEO_MODE);
    miniResetButton = gpio_get_level(BOARD_PIN_SOFT_RESET);
    miniHPD = gpio_get_level(BOARD_PIN_HPH_DET_P);
    miniSD = gpio_get_level(BOARD_PIN_SD_DET);
    return mini9PinButton;
}

