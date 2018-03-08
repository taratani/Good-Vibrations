#ifndef TOF_HW_H_
#define TOF_HW_H_

#include "stm32f4xx_hal_tof.h"



enum reg_addr
    {
      SYSRANGE_START                              = 0x00,

      SYSTEM_THRESH_HIGH                          = 0x0C,
      SYSTEM_THRESH_LOW                           = 0x0E,

      SYSTEM_SEQUENCE_CONFIG                      = 0x01,
      SYSTEM_RANGE_CONFIG                         = 0x09,
      SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

      SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

      GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

      SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

      RESULT_INTERRUPT_STATUS                     = 0x13,
      RESULT_RANGE_STATUS                         = 0x14,

      RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
      RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
      RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
      RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
      RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

      ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

      I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

      MSRC_CONFIG_CONTROL                         = 0x60,

      PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
      PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
      PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
      PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

      FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
      FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
      FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
      FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

      PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
      PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

      PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
      PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

      SYSTEM_HISTOGRAM_BIN                        = 0x81,
      HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
      HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

      FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
      FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
      CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

      MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

      SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
      IDENTIFICATION_MODEL_ID                     = 0xC0,
      IDENTIFICATION_REVISION_ID                  = 0xC2,

      OSC_CALIBRATE_VAL                           = 0xF8,

      GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
      GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

      GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
      DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
      DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
      POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

      VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

      ALGO_PHASECAL_LIM                           = 0x30,
      ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};


typedef enum
{
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vcselPeriodType;

typedef struct
{
      bool tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct
{
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;



// Write an 8-bit register
void tof_write_reg(uint8_t reg, uint8_t value);

// Write a 16-bit register
// First send MSB then LSB
void tof_write_reg_16Bit(uint8_t reg, uint16_t value);

// Write a 32-bit register
void tof_write_reg_32Bit(uint8_t reg, uint32_t value);

// Read an 8-bit register
uint8_t tof_read_reg(uint8_t reg);

// Read a 16-bit register
uint16_t tof_read_reg_16Bit(uint8_t reg);

// Read a 32-bit register
uint32_t tof_read_reg_32Bit(uint8_t reg);


uint16_t tof_get_timeout();

void tof_set_timeout(uint16_t timeout);


/**
 * @brief tof_init
 * Initialize sensor using sequence.
 * This function does not perform reference SPAD calibration
 * If io_2v8 is true, the sensor is configured for 2V8
 * mode.
 * @param io_2v8
 * True - sensor configured for 2V8 mode, false - 1V8
 * @return
 * True if successful, otherwise false
 */
bool tof_init(bool io_2v8);

/**
 * @brief tof_set_signal_rate_limit
 * @param limit_Mcps
 * @return
 * True if successful, otherwise false
 */
bool tof_set_signal_rate_limit(float limit_Mcps);

/**
 * @brief tof_get_signal_rate_limit
 * Get the return signal rate limit check value in MCPS
 * @return
 * Signal rate limit in MCPS
 */
float tof_get_signal_rate_limit(void);

/**
 * @brief tof_set_measurement_timing_budget
 * Set the measurement timing budget in microseconds, which is the time allowed
 * for one measurement.Increasing the budget by a
 * factor of N decreases the range measurement standard deviation by a factor of
 * sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
 * @param budget_us
 * Desired budget in us
 * @return
 * True if successful, otherwise false
 */
bool tof_set_measurement_timing_budget(uint32_t budget_us);

/**
 * @brief tof_get_measurement_timing_budget
 * Get the measurement timing budget in microseconds
 * @return
 * measurement timing budget in microseconds
 */
uint32_t tof_get_measurement_timing_budget(void);

/**
 * @brief tof_set_vcsel_pulse_period
 * Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
 * given period type (pre-range or final range) to the given value in PCLKs.
 * Longer periods seem to increase the potential range of the sensor.
 * Valid values are (even numbers only):
 * pre:  12 to 18 (initialized default: 14)
 * final: 8 to 14 (initialized default: 10)
 * @param type
 * period type
 * @param period_pclks
 * given value in PCLKs
 * @return
 * True if successful, false if failed
 */
bool tof_set_vcsel_pulse_period(vcselPeriodType type, uint8_t period_pclks);

/**
 * @brief tof_get_vcsel_pulse_period
 * Get the VCSEL pulse period in PCLKs for the given period type.
 * @param type
 * period type
 * @return
 * The pulse period
 */
uint8_t tof_get_vcsel_pulse_period(vcselPeriodType type);

/**
 * @brief tof_start_continuous
 * Start continuous ranging measurements. If period_ms is 0
 * continuous back-to-back mode is used (the sensor takes measurements as
 * often as possible); otherwise, continuous timed mode is used, with the given
 * inter-measurement period in milliseconds determining how often the sensor
 * takes a measurement.
 * @param period_ms
 * inter-measurement period in milliseconds
 */
void tof_start_continuous(uint32_t period_ms);
/**
 * @brief tof_stop_continuous
 * Stop continuous measurements
 */
void tof_stop_continuous(void);
/**
 * @brief tof_read_range_continuous_millimeters
 * Returns a range reading in millimeters when continuous mode is active
 * (read_range_single_millimeters() also calls this function after starting a
 * single-shot range measurement)
 * @return
 * Range in millimeters
 */
uint16_t tof_read_range_continuous_millimeters(void);

/**
 * @brief tof_read_range_single_millimeters
 * Performs a single-shot range measurement and returns the reading in
 * millimeters
 * @return
 * Range in millimeters
 */
uint16_t tof_read_range_single_millimeters(void);

/**
 * @brief tof_timeout_occurred
 * Checks if a timeout has occured
 * @return
 * True if timeout occured, otherwise false
 */
bool tof_timeout_occurred();
/**
 * @brief tof_get_spad_info
 * Get reference SPAD (single photon avalanche diode) count and type
 * @param count
 * Buffer to store the count
 * @param type_is_aperture
 * Buffer to store the type
 * @return
 * Returns true if successful, false if failed
 */
bool tof_get_spad_info(uint8_t * count, bool * type_is_aperture);
/**
 * @brief tof_get_sequence_step_enables
 * Get sequence step enables
 * @param enables
 * Buffer to store the enables
 */
void tof_get_sequence_step_enables(SequenceStepEnables * enables);
/**
 * @brief tof_get_sequence_step_timeouts
 * Get sequence step timeouts
 * @param enables
 * Desired enables
 * @param timeouts
 * Buffer to store the timeouts
 */
void tof_get_sequence_step_timeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

/**
 * @brief tof_decode_timeout
 * Decode sequence step timeout in MCLKs from register value
 * @param reg_val
 * Register value
 * @return
 * Conversion result
 */
uint16_t tof_decode_timeout(uint16_t reg_val);

/**
 * @brief tof_encode_timeout
 * Encode sequence step timeout register value from timeout in MCLKs
 * @param timeout_mclks
 * Timeout in mclks
 * @return
 * Conversion result
 */
uint16_t tof_encode_timeout(uint16_t timeout_mclks);

/**
 * @brief tof_timeout_mclks_to_microseconds
 * Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
 * @param timeout_period_mclks
 * Timeout period in mclks
 * @param vcsel_period_pclks
 * VCSEL period in pclks
 * @return
 * Conversion result
 */
uint32_t tof_timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);

/**
 * @brief tof_timeout_microseconds_to_mclks
 * Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
 * @param timeout_period_us
 * Timeout period in us
 * @param vcsel_period_pclks
 * VCSEL period in PCLKs
 * @return
 * Conversion result
 */
uint32_t tof_timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

bool tof_perform_single_ref_calibration(uint8_t vhv_init_byte);

#endif
