#include "ED_adc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include <vector>
#include <algorithm>
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace ED_ADC {

static inline const char *TAG = "ED_ADC";

// ADCChannel implementations
ADCChannel* ADCChannel::create(ADCUnit* unit, adc_channel_t channel, adc_atten_t atten) {
    ADCChannel* new_channel = new ADCChannel(unit, channel, atten);
    if (new_channel->isInitialized()) {
        return new_channel;
    }
    delete new_channel;
    return nullptr;
}

std::vector<int> ADCChannel::sampleForDuration(uint32_t duration_ms) {
    std::vector<int> voltages;
    uint32_t buffer_size = 1024;
    uint8_t* buffer = (uint8_t*)malloc(buffer_size);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate ADC buffer for continuous sampling");
        return voltages;
    }
    memset(buffer, 0, buffer_size);

    esp_err_t start_err = adc_continuous_start(_cont_handle);
    if (start_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start continuous ADC: %s", esp_err_to_name(start_err));
        free(buffer);
        return voltages;
    }

    uint64_t start_time = esp_timer_get_time();
    while ((esp_timer_get_time() - start_time) / 1000 < duration_ms) {
        uint32_t bytes_read = 0;
        esp_err_t ret = adc_continuous_read(_cont_handle, buffer, buffer_size, &bytes_read, 0);

        if (ret == ESP_OK) {
            // Process data for ADC_DIGI_OUTPUT_FORMAT_TYPE2
            for (size_t i = 0; i < bytes_read; i += 2) {
                int raw_reading = (buffer[i + 1] << 8) | buffer[i];
                raw_reading &= 0xFFF; // Mask to get only the 12-bit ADC value

                int voltage;
                adc_cali_raw_to_voltage(_cali_handle, raw_reading, &voltage);
                voltages.push_back(voltage);
            }
        } else if (ret != ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "ADC continuous read error: %s", esp_err_to_name(ret));
        }
    }

    esp_err_t stop_err = adc_continuous_stop(_cont_handle);
    if (stop_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop continuous ADC: %s", esp_err_to_name(stop_err));
    }

    free(buffer);
    return voltages;
}

esp_err_t ADCChannel::read(int sample_count, int sample_delay_ms, ADCReadResult& result) {
    std::vector<int> voltages;
    voltages.reserve(sample_count);

    uint32_t sum = 0;
    int min = INT32_MAX;
    int max = INT32_MIN;

    for (int i = 0; i < sample_count; i++) {
        int raw_reading;
        esp_err_t err = adc_oneshot_read(_oneshot_handle, _channel, &raw_reading);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Oneshot read failed: %s", esp_err_to_name(err));
            return err;
        }

        int voltage;
        adc_cali_raw_to_voltage(_cali_handle, raw_reading, &voltage);

        voltages.push_back(voltage);
        sum += voltage;
        if (voltage < min) min = voltage;
        if (voltage > max) max = voltage;

        if (sample_delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(sample_delay_ms));
        }
    }

    result.average_mv = sum / sample_count;
    result.min_mv = min;
    result.max_mv = max;
    result.p30_width_mv = calculatePercWidth(voltages, 30);
    result.p60_width_mv = calculatePercWidth(voltages, 60);

    return ESP_OK;
}

ADCChannel::ADCChannel(ADCUnit* unit, adc_channel_t channel, adc_atten_t atten)
    : _oneshot_handle(unit->getOneshotHandle()),
      _cont_handle(unit->getContinuousHandle()),
      _channel(channel) {

    _is_initialized = false;

    // Oneshot channel configuration
    adc_oneshot_chan_cfg_t oneshot_chan_cfg = {
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };

    esp_err_t err = adc_oneshot_config_channel(_oneshot_handle, _channel, &oneshot_chan_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADCChannel - Failed to configure oneshot channel: %s", esp_err_to_name(err));
        return;
    }

    // Calibration setup - fix missing field initializers
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = unit->getUnitId(),
        .chan = _channel,  // Added missing field
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &_cali_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADCChannel - Failed to create calibration scheme: %s", esp_err_to_name(err));
        return;
    }
    _is_initialized = true;
}

int ADCChannel::calculatePercWidth(std::vector<int>& data, int8_t percentile) {
    if (percentile < 10 || percentile > 90) return 0;
    if (data.empty()) return 0;

    std::sort(data.begin(), data.end());

    const size_t n = data.size();
    const float perc_decimal = static_cast<float>(percentile) / 100.0f;

    // Calculate the indices for the percentile range
    // For example, if percentile = 30:
    // - Lower bound: 30th percentile (30% from bottom)
    // - Upper bound: 70th percentile (100% - 30% from bottom)

    size_t lower_index = static_cast<size_t>((n - 1) * perc_decimal);
    size_t upper_index = static_cast<size_t>((n - 1) * (1.0f - perc_decimal));

    // Ensure indices are within bounds
    lower_index = std::min(lower_index, n - 1);
    upper_index = std::min(upper_index, n - 1);

    // The width is the difference between the upper and lower percentile values
    // Since data is sorted: data[upper_index] >= data[lower_index]
    int lower_value = data[lower_index];
    int upper_value = data[upper_index];

    return upper_value - lower_value;
}

bool ADCChannel::isInitialized() const {
    return _is_initialized;
}

// ADCUnit implementations
ADCUnit* ADCUnit::create(adc_unit_t unit_id) {
    ADCUnit* new_unit = new ADCUnit(unit_id);
    if (new_unit->isInitialized()) {
        return new_unit;
    }
    delete new_unit;
    return nullptr;
}

adc_oneshot_unit_handle_t ADCUnit::getOneshotHandle() const {
    return _oneshot_handle;
}

adc_continuous_handle_t ADCUnit::getContinuousHandle() {
    if (ensureContinuousInitialized() == ESP_OK) {
        return _cont_handle;
    }
    return nullptr;
}

adc_unit_t ADCUnit::getUnitId() const {
    return _unit_id;
}

// Fixed constructor with proper member initialization order
ADCUnit::ADCUnit(adc_unit_t unit_id)
    : _is_initialized(false),
      _unit_id(unit_id),
      _cont_handle(nullptr),
      _continuous_initialized(false) {

    adc_oneshot_unit_init_cfg_t oneshot_init_cfg = {
        .unit_id = unit_id,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&oneshot_init_cfg, &_oneshot_handle);
    _is_initialized = (err == ESP_OK);

    if (!_is_initialized) {
        ESP_LOGE(TAG, "Failed to create oneshot unit: %s", esp_err_to_name(err));
    }
}

esp_err_t ADCUnit::ensureContinuousInitialized() {
    if (_continuous_initialized) return ESP_OK;

    // Fixed missing field initializer
    adc_continuous_handle_cfg_t continuous_handle_cfg = {
        .max_store_buf_size = 1024,
        .conv_frame_size = 256,
        .flags = 0  // Added missing field
    };

    esp_err_t err = adc_continuous_new_handle(&continuous_handle_cfg, &_cont_handle);
    if (err != ESP_OK) return err;

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_0,
        .unit = _unit_id,
        .bit_width = ADC_BITWIDTH_12,
    };

    adc_continuous_config_t continuous_config = {
        .pattern_num = 1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz = 20000,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2,
    };

    err = adc_continuous_config(_cont_handle, &continuous_config);
    if (err != ESP_OK) {
        adc_continuous_deinit(_cont_handle);
        return err;
    }

    _continuous_initialized = true;
    return ESP_OK;
}

bool ADCUnit::isInitialized() const {
    return _is_initialized;
}

} // namespace ED_ADC