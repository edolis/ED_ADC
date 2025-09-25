#pragma once
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_continuous.h" // Include for continuous mode
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_timer.h" // Include for high-resolution timer
#include <algorithm>
#include <cstdint>
#include <vector>

namespace ED_ADC {

// Define a struct to hold the results of the ADC reading
typedef struct {
  int average_mv;
  int min_mv;
  int max_mv;
  int p30_width_mv; // Width of the 50th percentile
  int p60_width_mv; // Width of the 50th percentile
} ADCReadResult;

// Forward declaration
class ADCUnit;

class ADCChannel {
public:
  /**
   * @example
   *
   *
   * // ADC Setup
  ADCUnit *ADC = ADCUnit::create();
  ADCChannel *ADC0 = ADCChannel::create(
      ADC, ADC_CHANNEL_0, ADC_ATTEN_DB_6); // MAX voltage: plan for 1300mV

  ADCReadResult reading = {};
  ADC0->read(200, 10, reading);

   *
   *
   *
   *
   *
   * @brief creates an ADC channel object and initializes it
   *
   * @param unit Reference to the initialized ADCUnit object.
   * @param channel channel-pin mapping:
   *  * ADC1 Channels:
   *   - ADC1_CH0  → XTAL_32K_P
   *   - ADC1_CH1  → XTAL_32K_N
   *   - ADC1_CH2  → GPIO2
   *   - ADC1_CH3  → GPIO3
   *   - ADC1_CH4  → MTMS
   *
   * * ADC2 Channels:
   *   - ADC2_CH0  → MTDI
   * @param atten attenuation. refer to the table
   *  * Enum Value         | Attenuation | Effective Range (mV) | Total Error
   * (mV) | Notes
   * -------------------|-------------|-----------------------|------------------|-------------------------------
   * ADC_ATTEN_DB_0     | 0 dB        | 0 – 750               | ±10
   *  | Best for low-voltage sensors ADC_ATTEN_DB_2_5   | 2.5 dB      | 0 – 1050
   *              | ±10              | Matches internal Vref (1100 mV)
   * ADC_ATTEN_DB_6     | 6 dB        | 0 – 1300              | ±10
   *  | Slightly above Vref ADC_ATTEN_DB_12    | 12 dB       | 0 – 2500
   *      | ±35              | Max usable range; higher error
   * @return ADCChannel* A pointer to the initialized ADCChannel object.
   */
  static ADCChannel *create(ADCUnit *unit, adc_channel_t channel,
                            adc_atten_t atten);
  /**
   * @brief performs a sequence of reading from the Analogue channel
   *
   * @param sample_count [in] number of readings
   * @param sample_delay_ms [in] delay in ms between readings
   * @param result [out] the result of the reading
   * @return esp_err_t
   */
  esp_err_t read(int sample_count, int sample_delay_ms, ADCReadResult &result);

  /**
   * @brief Samples the channel for a given duration using continuous mode.
   * @param duration_ms The total time to sample in milliseconds.
   * @return A vector of calibrated voltage readings (in mV).
   */
  std::vector<int> sampleForDuration(uint32_t duration_ms);

private:
  /**
   * @brief calculates the xth percentile to give an idea of the concentration
   * of data
   *
   * @param data
   * @param percentile percentile value - valid range 10 to 90
   * @return int the calculated percentile
   */
  int calculatePercWidth(std::vector<int> &data, int8_t percentile = 50);
  ADCChannel(ADCUnit *unit, adc_channel_t channel, adc_atten_t atten);
  ~ADCChannel();
  bool isInitialized() const;

  adc_oneshot_unit_handle_t _oneshot_handle;
  adc_continuous_handle_t _cont_handle;
  adc_channel_t _channel;
  adc_cali_handle_t _cali_handle;
  bool _is_initialized = false;
};

class ADCUnit {
public:
  /// @brief creates an ADC unit.
  /// please notice ADC_UNIT_2 has just 1 channel, and speecial features,
  /// usually just the channels of UNIT1 will be used in ESP. Check tech doc.
  /// @param unit_id
  /// @return a pointer to the initialized unit
  static ADCUnit *create(adc_unit_t unit_id = ADC_UNIT_1);
  ~ADCUnit();
  // Getters for both types of handles
  /**
   * @brief Get the Oneshot Handle for one shot readings of the Analog channel
   * @return adc_oneshot_unit_handle_t
   */
  adc_oneshot_unit_handle_t getOneshotHandle() const;
  /**
   * @brief Get the Continuous Handle object for continuous reading from the
   * Analog channel
   * @return adc_continuous_handle_t
   */
  adc_continuous_handle_t
  getContinuousHandle(); // Removed const since it calls
                         // ensureContinuousInitialized()

  adc_unit_t getUnitId() const;

private:
  ADCUnit(adc_unit_t unit_id);
  bool isInitialized() const;
  esp_err_t ensureContinuousInitialized(); // Fixed indentation

  // Fixed member declaration order to match constructor initialization list
  bool _is_initialized = false;
  adc_unit_t _unit_id;
  adc_continuous_handle_t _cont_handle;
  bool _continuous_initialized = false; // Added default initialization
  adc_oneshot_unit_handle_t _oneshot_handle;
};

} // namespace ED_ADC