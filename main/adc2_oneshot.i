/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0


ADC — Analog to Digital Converter
A microcontroller only understands digital values (0 or 1). 
The real world produces voltages — a joystick potentiometer, 
a temperature sensor, a battery voltage divider. 
The ADC is the hardware block inside the ESP32-P4 that measures a 
voltage on a pin and converts it to a number. 
With 12-bit resolution the result is 0–4095, 
where 0 = 0 V and 4095 = full-scale voltage.

 */


/*---------------------------------------------------------------
        ADC General Macros
---------------------------------------------------------------*/
#define EXAMPLE_ADC2_CHAN0          ADC_CHANNEL_0
#define EXAMPLE_ADC2_CHAN1          ADC_CHANNEL_1
#define EXAMPLE_ADC2_CHAN2          ADC_CHANNEL_2

#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

DRAM_ATTR static adc_oneshot_unit_handle_t adc2_handle;
DRAM_ATTR static adc_cali_handle_t adc2_cali_chan0_handle = NULL;
DRAM_ATTR static adc_cali_handle_t adc2_cali_chan1_handle = NULL;
DRAM_ATTR static adc_cali_handle_t adc2_cali_chan2_handle = NULL;

static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static void example_adc_calibration_deinit(adc_cali_handle_t handle);
bool do_calibration2_chan0 = false;
bool do_calibration2_chan1 = false;
bool do_calibration2_chan2 = false;

#define TOLERANCE_MV 30 // tolerance to differ from mid point to have any readings at all
DRAM_ATTR static int dataVolume = 0;
DRAM_ATTR static int dataXAxis = 0; // last read voltage on X axis
DRAM_ATTR static int dataYAxis = 0; // last read voltage on Y axis

DRAM_ATTR static int minV_X = 800;  // minimum voltage X axis
DRAM_ATTR static int maxV_X = 2600; // maximum voltage X axis
DRAM_ATTR static int minV_Y = 800;  // minimum voltage Y axis
DRAM_ATTR static int maxV_Y = 2600; // maximum voltage Y axis

// this is not allways 100% correct... jitter, potentiometers... 
DRAM_ATTR static int midV_X = (2600+800)>>1; // calculated "zero" X axis
DRAM_ATTR static int midV_Y = (2600+800)>>1; // calculated "zero" Y axis

void initADC(void)
{
    //-------------ADC2 Init---------------//
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id = ADC_UNIT_2,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    adc_oneshot_chan_cfg_t config2 = {
        .atten = EXAMPLE_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    //-------------ADC2 Calibration Init---------------//
    do_calibration2_chan0 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN0, EXAMPLE_ADC_ATTEN, &adc2_cali_chan0_handle);
    do_calibration2_chan1 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN1, EXAMPLE_ADC_ATTEN, &adc2_cali_chan1_handle);
    do_calibration2_chan2 = example_adc_calibration_init(ADC_UNIT_2, EXAMPLE_ADC2_CHAN2, EXAMPLE_ADC_ATTEN, &adc2_cali_chan2_handle);

    //-------------ADC2 Config---------------//
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN0, &config2));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN1, &config2));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, EXAMPLE_ADC2_CHAN2, &config2));
}

// needs to be called after
// void get_analog()
IRAM_ATTR static inline bool _is9PinJoystickAvailable()
{
    return (dataXAxis+dataYAxis > 200);
}
IRAM_ATTR bool is9PinJoystickAvailable(void)
{
    return (dataXAxis+dataYAxis > 200);
}

IRAM_ATTR int get9PinAnalogX(void)
{
    int mv = dataXAxis;
    int mid = midV_X;

    if (mv < mid - TOLERANCE_MV)
    {
        // negative half: minV_X → -128, mid-TOLERANCE_MV → 0
        int range = mid - TOLERANCE_MV - minV_X;
        if (range <= 0) return -128;
        int val = -128 + ((mv - minV_X) * 128) / range;
        if (val < -128) val = -128;
        if (val >    0) val =     0;
        return val;
    }
    else if (mv > mid + TOLERANCE_MV)
    {
        // positive half: mid+TOLERANCE_MV → 0, maxV_X → +127
        int range = maxV_X - (mid + TOLERANCE_MV);
        if (range <= 0) return 127;
        int val = ((mv - (mid + TOLERANCE_MV)) * 127) / range;
        if (val <   0) val =   0;
        if (val > 127) val = 127;
        return val;
    }
    else
    {
        return 0;  // within tolerance band — dead zone
    }
}
IRAM_ATTR int get9PinAnalogY(void)
{
    int mv = dataYAxis;
    int mid = midV_Y;

    if (mv < mid - TOLERANCE_MV)
    {
        int range = mid - TOLERANCE_MV - minV_Y;
        if (range <= 0) return -128;
        int val = -128 + ((mv - minV_Y) * 128) / range;
        if (val < -128) val = -128;
        if (val >    0) val =     0;
        return val;
    }
    else if (mv > mid + TOLERANCE_MV)
    {
        int range = maxV_Y - (mid + TOLERANCE_MV);
        if (range <= 0) return 127;
        int val = ((mv - (mid + TOLERANCE_MV)) * 127) / range;
        if (val <   0) val =   0;
        if (val > 127) val = 127;
        return val;
    }
    else
    {
        return 0;
    }
}
// with my hardware and my joystick
// rad pad
// center is about 1600 - 1700 mv
// min is about 780 mv
// max is about 2680 mv

// vectrex joystick
// center is about 1550 - 1750 mv
// min is about 560 mv
// max is about 2800 mv

IRAM_ATTR void get9PinAnalog(void)
{
    int dataVolumeRaw;
    int dataXAxisRaw;
    int dataYAxisRaw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN0, &dataVolumeRaw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN1, &dataXAxisRaw));
    ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, EXAMPLE_ADC2_CHAN2, &dataYAxisRaw));

    if (do_calibration2_chan0) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan0_handle, dataVolumeRaw, &dataVolume));
        // ESP_LOGI(TAG, "dataVolume Cali Voltage: %d(%d) mV",dataVolume, dataVolumeRaw);
    }
    else
    {
        dataVolume = dataVolumeRaw;
        // ESP_LOGI(TAG, "dataVolume Voltage: %d mV", dataVolume);
    }

    if (do_calibration2_chan1) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan1_handle, dataXAxisRaw, &dataXAxis));
        // ESP_LOGI(TAG, "dataXAxis Cali Voltage: %d(%d) mV", dataXAxis, dataXAxisRaw);
    }
    else 
    {
        dataXAxis = dataXAxisRaw;
        // ESP_LOGI(TAG, "dataXAxis Voltage: %d mV", dataXAxis);
    }

    if (do_calibration2_chan2) {
        ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc2_cali_chan2_handle, dataYAxisRaw, &dataYAxis));
        // ESP_LOGI(TAG, "dataYAxis Cali Voltage: %d(%d) mV", dataYAxis, dataYAxisRaw);
    }
    else 
    {
        dataYAxis = dataYAxisRaw;
        // ESP_LOGI(TAG, "dataYAxis Voltage: %d mV", dataYAxis);
    }

    // self calibrating
    if (is9PinJoystickAvailable())
    {
        if(dataXAxis<minV_X)
        {
            if (dataXAxis>200) // no NON joytick!, seems double to above... but may be a timing issue?
            {
                minV_X=dataXAxis;
                midV_X = (maxV_X+minV_X)>>1;
            }
        }
        if(dataYAxis<minV_Y) 
        {
            if (dataYAxis>200) // no NON joytick!, seems double to above... but may be a timing issue?
            {
                minV_Y=dataYAxis;
                midV_Y = (maxV_Y+minV_Y)>>1;
            }
        }
        if(dataXAxis>maxV_X) 
        {
            maxV_X=dataXAxis;
            midV_X = (maxV_X+minV_X)>>1;
        }
        if(dataYAxis>maxV_Y) 
        {
            maxV_Y=dataYAxis;
            midV_Y = (maxV_Y+minV_Y)>>1;
        }
    }
}

/*---------------------------------------------------------------
        ADC Calibration

Why calibration exists
The ADC hardware is not perfect. Two problems:

1. Non-linearity. The relationship between the real voltage and the raw number 
is not a perfectly straight line. 
At the extremes (near 0 V or near full-scale) the error can be 100–200 mV. 
In the middle it is much better.

2. Unit-to-unit variation. Every chip comes out of the factory slightly different. 
The reference voltage and internal resistors have manufacturing tolerances.

Espressif measures each chip individually during production and burns 
correction data into a one-time-programmable memory called eFuse. 
The calibration code reads those values and uses them to correct the 
raw reading before converting to millivolts. Without calibration you 
still get a number, but adc_cali_raw_to_voltage would just do a 
linear approximation with no chip-specific correction.

The two calibration schemes
Line Fitting (ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED)
A simple correction: take the raw value, apply a linear formula with
a chip-specific offset and gain. Fast, low memory.
Used on older ESP32 silicon and some ESP32-S2/S3 parts.

Curve Fitting (ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED)
A more accurate correction: the raw value is passed through a 
polynomial curve that matches the measured non-linearity of 
the ADC more closely. Requires more eFuse data burned in. 
Used on newer silicon (ESP32-S2, ESP32-S3, ESP32-C3, and 
ESP32-P4 depending on silicon revision).

The code tries Curve Fitting first; if that fails 
(eFuse not available) it falls back to Line Fitting. 
If neither is available it skips software calibration entirely and 
just uses the raw linear approximation.

Where those macros come from
You do not set them yourself. 
They are defined automatically by the IDF headers — 
specifically inside esp_adc/adc_cali_scheme.h — 
based on which chip target you compiled for
and which silicon revision it is. 

So: calibration is not required to get a raw reading, but it is required 
if you want an accurate millivolt value.

---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

static void example_adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
