#include "tof_hw.h"



extern uint64_t milliseconds_counter;
uint16_t io_timeout;
bool did_timeout;
uint16_t timeout_start_ms;

uint8_t stop_variable; // read by init and used when starting measurement;
uint32_t measurement_timing_budget_us;



uint64_t milliseconds()
{
    return milliseconds_counter;
}

bool tof_check_timeout_expired()
{
  return (io_timeout > 0 && ((uint16_t)milliseconds() - timeout_start_ms) > io_timeout);
}

void tof_set_timeout(uint16_t timeout)
{
     io_timeout = timeout;
}

uint16_t tof_get_timeout()
{
         return io_timeout;
}


#define tof_start_timeout() (timeout_start_ms = milliseconds())


// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
#define tof_decode_vcsel_period(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
#define tof_encode_vcsel_period(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define tof_calc_macro_period(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)



void tof_write_reg(uint8_t reg, uint8_t value)
{
        uint8_t temp[2];
        temp[0] = reg;
        temp[1] = value;
        tof_hal_write(VL53L0X_ADDR, temp, 2);
}



void tof_write_reg_16Bit(uint8_t reg, uint16_t value)
{
        uint8_t temp[3];
        temp[0] = reg;
        temp[1] = (uint8_t) (value >> 8);
        temp[2] = (uint8_t) (value);
        tof_hal_write(VL53L0X_ADDR, temp, 3);
}


void tof_write_reg_32Bit(uint8_t reg, uint32_t value)
{
        uint8_t temp[3];
        temp[0] = reg;
        temp[1] = (uint8_t) (value >> 24);
        temp[2] = (uint8_t) (value >> 16);
        temp[3] = (uint8_t) (value >> 8);
        temp[4] = (uint8_t) (value);
        tof_hal_write(VL53L0X_ADDR, temp, 5);
}


uint8_t tof_read_reg(uint8_t reg)
{
        uint8_t value = reg;
        tof_hal_read(VL53L0X_ADDR, &value, 1);
        return value;
}


uint16_t tof_read_reg_16Bit(uint8_t reg)
{
        uint16_t value;
        uint8_t temp[2];
        temp[0] = reg;

        tof_hal_read(VL53L0X_ADDR, temp, 2);
        value = (uint16_t) (temp[0]);
        value <<= 8;
        value |= (uint16_t) (temp[1]);
        return value;
}


uint32_t tof_read_reg_32Bit(uint8_t reg)
{
        uint32_t value;
        uint8_t temp[3];
        temp[0] = reg;

        tof_hal_read(VL53L0X_ADDR, temp, 3);

        value |= ((uint32_t) temp[0]) << 24;
        value |= ((uint32_t) temp[1]) << 16;
        value |= ((uint32_t) temp[2]) << 8;
        value |= (uint32_t) (temp[3]);
        return value;
}


bool tof_init(bool io_2v8)
{
        uint8_t spad_count;
        bool spad_type_is_aperture;
        uint8_t first_spad_to_enable;
        uint8_t spads_enabled;
        uint8_t ref_spad_map[6];
        uint8_t i;
        uint8_t send_buffer[7];




        if (io_2v8)
        {
                tof_write_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                              tof_read_reg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
        }


        tof_write_reg(0x88, 0x00);

        tof_write_reg(0x80, 0x01);
        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);
        stop_variable = tof_read_reg(0x91);
        tof_write_reg(0x00, 0x01);
        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x00);


        tof_write_reg(MSRC_CONFIG_CONTROL, tof_read_reg(MSRC_CONFIG_CONTROL) | 0x12);


        tof_set_signal_rate_limit(0.25);

        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xFF);


        if (!tof_get_spad_info(&spad_count, &spad_type_is_aperture)) { return false; }

        ref_spad_map[0] = GLOBAL_CONFIG_SPAD_ENABLES_REF_0;

        tof_hal_read(VL53L0X_ADDR, ref_spad_map, 6);


        tof_write_reg(0xFF, 0x01);
        tof_write_reg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
        tof_write_reg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
        tof_write_reg(0xFF, 0x00);
        tof_write_reg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

        first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
        spads_enabled = 0;

        for ( i = 0; i < 48; i++)
        {
                if (i < first_spad_to_enable || spads_enabled == spad_count)
                {
                        ref_spad_map[i / 8] &= ~(1 << (i % 8));
                }
                else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
                {
                        spads_enabled++;
                }
        }
        
        send_buffer[0] = GLOBAL_CONFIG_SPAD_ENABLES_REF_0;
        for (i = 1; i < 7; i++)
        {
           send_buffer[i] = ref_spad_map[i-1];
        }

        tof_hal_write(VL53L0X_ADDR, ref_spad_map, 6);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x09, 0x00);
        tof_write_reg(0x10, 0x00);
        tof_write_reg(0x11, 0x00);

        tof_write_reg(0x24, 0x01);
        tof_write_reg(0x25, 0xFF);
        tof_write_reg(0x75, 0x00);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x4E, 0x2C);
        tof_write_reg(0x48, 0x00);
        tof_write_reg(0x30, 0x20);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x30, 0x09);
        tof_write_reg(0x54, 0x00);
        tof_write_reg(0x31, 0x04);
        tof_write_reg(0x32, 0x03);
        tof_write_reg(0x40, 0x83);
        tof_write_reg(0x46, 0x25);
        tof_write_reg(0x60, 0x00);
        tof_write_reg(0x27, 0x00);
        tof_write_reg(0x50, 0x06);
        tof_write_reg(0x51, 0x00);
        tof_write_reg(0x52, 0x96);
        tof_write_reg(0x56, 0x08);
        tof_write_reg(0x57, 0x30);
        tof_write_reg(0x61, 0x00);
        tof_write_reg(0x62, 0x00);
        tof_write_reg(0x64, 0x00);
        tof_write_reg(0x65, 0x00);
        tof_write_reg(0x66, 0xA0);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x22, 0x32);
        tof_write_reg(0x47, 0x14);
        tof_write_reg(0x49, 0xFF);
        tof_write_reg(0x4A, 0x00);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x7A, 0x0A);
        tof_write_reg(0x7B, 0x00);
        tof_write_reg(0x78, 0x21);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x23, 0x34);
        tof_write_reg(0x42, 0x00);
        tof_write_reg(0x44, 0xFF);
        tof_write_reg(0x45, 0x26);
        tof_write_reg(0x46, 0x05);
        tof_write_reg(0x40, 0x40);
        tof_write_reg(0x0E, 0x06);
        tof_write_reg(0x20, 0x1A);
        tof_write_reg(0x43, 0x40);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x34, 0x03);
        tof_write_reg(0x35, 0x44);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x31, 0x04);
        tof_write_reg(0x4B, 0x09);
        tof_write_reg(0x4C, 0x05);
        tof_write_reg(0x4D, 0x04);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x44, 0x00);
        tof_write_reg(0x45, 0x20);
        tof_write_reg(0x47, 0x08);
        tof_write_reg(0x48, 0x28);
        tof_write_reg(0x67, 0x00);
        tof_write_reg(0x70, 0x04);
        tof_write_reg(0x71, 0x01);
        tof_write_reg(0x72, 0xFE);
        tof_write_reg(0x76, 0x00);
        tof_write_reg(0x77, 0x00);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x0D, 0x01);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x01);
        tof_write_reg(0x01, 0xF8);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x8E, 0x01);
        tof_write_reg(0x00, 0x01);
        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x00);


        tof_write_reg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
        tof_write_reg(GPIO_HV_MUX_ACTIVE_HIGH, tof_read_reg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);
        tof_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        measurement_timing_budget_us = tof_get_measurement_timing_budget();


        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        tof_set_measurement_timing_budget(measurement_timing_budget_us);

        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x01);
        if (!tof_perform_single_ref_calibration(0x40)) { return false; }


        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        if (!tof_perform_single_ref_calibration(0x00)) { return false; }


        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

        return true;
}


bool tof_set_signal_rate_limit(float limit_Mcps)
{
        if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }


        tof_write_reg_16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
        return true;
}

float tof_get_signal_rate_limit(void)
{
        return (float)tof_read_reg_16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

bool tof_set_measurement_timing_budget(uint32_t budget_us)
{
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        uint16_t const StartOverhead      = 1320;
        uint16_t const EndOverhead        = 960;
        uint16_t const MsrcOverhead       = 660;
        uint16_t const TccOverhead        = 590;
        uint16_t const DssOverhead        = 690;
        uint16_t const PreRangeOverhead   = 660;
        uint16_t const FinalRangeOverhead = 550;

        uint32_t const MinTimingBudget = 20000;

        uint32_t used_budget_us;
        uint32_t final_range_timeout_us;
        uint16_t final_range_timeout_mclks;

        if (budget_us < MinTimingBudget) { return false; }

        used_budget_us = StartOverhead + EndOverhead;

        tof_get_sequence_step_enables(&enables);
        tof_get_sequence_step_timeouts(&enables, &timeouts);

        if (enables.tcc)
        {
                used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss)
        {
                used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
                used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range)
        {
                used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range)
        {
                used_budget_us += FinalRangeOverhead;

                if (used_budget_us > budget_us)
                {
                        return false;
                }

                final_range_timeout_us = budget_us - used_budget_us;

                final_range_timeout_mclks =
                    tof_timeout_microseconds_to_mclks(final_range_timeout_us,
                                               timeouts.final_range_vcsel_period_pclks);

                if (enables.pre_range)
                {
                        final_range_timeout_mclks += timeouts.pre_range_mclks;
                }

                tof_write_reg_16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    tof_encode_timeout(final_range_timeout_mclks));


                measurement_timing_budget_us = budget_us;
        }
        return true;
}

uint32_t tof_get_measurement_timing_budget(void)
{
        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        uint16_t const StartOverhead     = 1910;
        uint16_t const EndOverhead        = 960;
        uint16_t const MsrcOverhead       = 660;
        uint16_t const TccOverhead        = 590;
        uint16_t const DssOverhead        = 690;
        uint16_t const PreRangeOverhead   = 660;
        uint16_t const FinalRangeOverhead = 550;

        uint32_t budget_us = StartOverhead + EndOverhead;

        tof_get_sequence_step_enables(&enables);
        tof_get_sequence_step_timeouts(&enables, &timeouts);

        if (enables.tcc)
        {
                budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
        }

        if (enables.dss)
        {
                budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
        }
        else if (enables.msrc)
        {
                budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
        }

        if (enables.pre_range)
        {
                budget_us += (timeouts.pre_range_us + PreRangeOverhead);
        }

        if (enables.final_range)
        {
                budget_us += (timeouts.final_range_us + FinalRangeOverhead);
        }

        measurement_timing_budget_us = budget_us;
        return budget_us;
}

bool tof_set_vcsel_pulse_period(vcselPeriodType type, uint8_t period_pclks)
{
        uint8_t vcsel_period_reg = tof_encode_timeout(period_pclks);
        uint16_t new_pre_range_timeout_mclks;
        uint16_t new_msrc_timeout_mclks;
        uint16_t new_final_range_timeout_mclks;
        uint8_t sequence_config;

        SequenceStepEnables enables;
        SequenceStepTimeouts timeouts;

        tof_get_sequence_step_enables(&enables);
        tof_get_sequence_step_timeouts(&enables, &timeouts);


        if (type == VcselPeriodPreRange)
        {
                switch (period_pclks)
                {
                case 12:
                        tof_write_reg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
                        break;

                case 14:
                        tof_write_reg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
                        break;

                case 16:
                        tof_write_reg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
                        break;

                case 18:
                        tof_write_reg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
                        break;

                default:
                        // invalid period
                        return false;
                }
                tof_write_reg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);


                tof_write_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

                new_pre_range_timeout_mclks =
                    tof_timeout_microseconds_to_mclks(timeouts.pre_range_us, period_pclks);

                tof_write_reg_16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    tof_encode_timeout(new_pre_range_timeout_mclks));


                new_msrc_timeout_mclks =
                    tof_timeout_microseconds_to_mclks(timeouts.msrc_dss_tcc_us, period_pclks);

                tof_write_reg(MSRC_CONFIG_TIMEOUT_MACROP,
                              (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));


        }
        else if (type == VcselPeriodFinalRange)
        {
                switch (period_pclks)
                {
                case 8:
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                        tof_write_reg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                        tof_write_reg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                        tof_write_reg(0xFF, 0x01);
                        tof_write_reg(ALGO_PHASECAL_LIM, 0x30);
                        tof_write_reg(0xFF, 0x00);
                        break;

                case 10:
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                        tof_write_reg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                        tof_write_reg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                        tof_write_reg(0xFF, 0x01);
                        tof_write_reg(ALGO_PHASECAL_LIM, 0x20);
                        tof_write_reg(0xFF, 0x00);
                        break;

                case 12:
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                        tof_write_reg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                        tof_write_reg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                        tof_write_reg(0xFF, 0x01);
                        tof_write_reg(ALGO_PHASECAL_LIM, 0x20);
                        tof_write_reg(0xFF, 0x00);
                        break;

                case 14:
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                        tof_write_reg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
                        tof_write_reg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                        tof_write_reg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                        tof_write_reg(0xFF, 0x01);
                        tof_write_reg(ALGO_PHASECAL_LIM, 0x20);
                        tof_write_reg(0xFF, 0x00);
                        break;

                default:

                        return false;
                }


                tof_write_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);


                new_final_range_timeout_mclks =
                    tof_timeout_microseconds_to_mclks(timeouts.final_range_us, period_pclks);

                if (enables.pre_range)
                {
                        new_final_range_timeout_mclks += timeouts.pre_range_mclks;
                }

                tof_write_reg_16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                    tof_encode_timeout(new_final_range_timeout_mclks));


        }
        else
        {

                return false;
        }



        tof_set_measurement_timing_budget(measurement_timing_budget_us);


        sequence_config = tof_read_reg(SYSTEM_SEQUENCE_CONFIG);
        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, 0x02);
        tof_perform_single_ref_calibration(0x0);
        tof_write_reg(SYSTEM_SEQUENCE_CONFIG, sequence_config);


        return true;
}

uint8_t tof_get_vcsel_pulse_period(vcselPeriodType type)
{
        if (type == VcselPeriodPreRange)
        {
                return tof_decode_vcsel_period(tof_read_reg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else if (type == VcselPeriodFinalRange)
        {
                return tof_decode_vcsel_period(tof_read_reg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
        }
        else { return 255; }
}

void tof_start_continuous(uint32_t period_ms)
{
        tof_write_reg(0x80, 0x01);
        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);
        tof_write_reg(0x91, stop_variable);
        tof_write_reg(0x00, 0x01);
        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x00);

        if (period_ms != 0)
        {

                uint16_t osc_calibrate_val = tof_read_reg_16Bit(OSC_CALIBRATE_VAL);

                if (osc_calibrate_val != 0)
                {
                        period_ms *= osc_calibrate_val;
                }

                tof_write_reg_32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);



                tof_write_reg(SYSRANGE_START, 0x04);
        }
        else
        {
                tof_write_reg(SYSRANGE_START, 0x02);
        }
}

void tof_stop_continuous(void)
{
        tof_write_reg(SYSRANGE_START, 0x01);

        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);
        tof_write_reg(0x91, 0x00);
        tof_write_reg(0x00, 0x01);
        tof_write_reg(0xFF, 0x00);
}

uint16_t tof_read_range_continuous_millimeters(void)
{
        uint16_t range;
        tof_start_timeout();
        while ((tof_read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
                if (tof_check_timeout_expired())
                {
                        did_timeout = true;
                        return 65535;
                }
        }

        range = tof_read_reg_16Bit(RESULT_RANGE_STATUS + 10);

        tof_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        return range;
}

uint16_t tof_read_range_single_millimeters(void)
{
        tof_write_reg(0x80, 0x01);
        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);
        tof_write_reg(0x91, stop_variable);
        tof_write_reg(0x00, 0x01);
        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x00);

        tof_write_reg(SYSRANGE_START, 0x01);

        tof_start_timeout();
        while (tof_read_reg(SYSRANGE_START) & 0x01)
        {
                if (tof_check_timeout_expired())
                {
                        did_timeout = true;
                        return 65535;
                }
        }

        return tof_read_range_continuous_millimeters();
}

bool tof_timeout_occurred()
{
        bool tmp = did_timeout;
        did_timeout = false;
        return tmp;
}

bool tof_get_spad_info(uint8_t * count, bool * type_is_aperture)
{
        uint8_t tmp;

        tof_write_reg(0x80, 0x01);
        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x00);

        tof_write_reg(0xFF, 0x06);
        tof_write_reg(0x83, tof_read_reg(0x83) | 0x04);
        tof_write_reg(0xFF, 0x07);
        tof_write_reg(0x81, 0x01);

        tof_write_reg(0x80, 0x01);

        tof_write_reg(0x94, 0x6b);
        tof_write_reg(0x83, 0x00);
        tof_start_timeout();
        while (tof_read_reg(0x83) == 0x00)
        {
                if (tof_check_timeout_expired()) { return false; }
        }
        tof_write_reg(0x83, 0x01);
        tmp = tof_read_reg(0x92);

        *count = tmp & 0x7f;
        *type_is_aperture = (tmp >> 7) & 0x01;

        tof_write_reg(0x81, 0x00);
        tof_write_reg(0xFF, 0x06);
        tof_write_reg(0x83, tof_read_reg( 0x83  & ~0x04));
        tof_write_reg(0xFF, 0x01);
        tof_write_reg(0x00, 0x01);

        tof_write_reg(0xFF, 0x00);
        tof_write_reg(0x80, 0x00);

        return true;
}

void tof_get_sequence_step_enables(SequenceStepEnables * enables)
{
        uint8_t sequence_config = tof_read_reg(SYSTEM_SEQUENCE_CONFIG);

        enables->tcc          = (sequence_config >> 4) & 0x1;
        enables->dss          = (sequence_config >> 3) & 0x1;
        enables->msrc         = (sequence_config >> 2) & 0x1;
        enables->pre_range    = (sequence_config >> 6) & 0x1;
        enables->final_range  = (sequence_config >> 7) & 0x1;
}

void tof_get_sequence_step_timeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
        timeouts->pre_range_vcsel_period_pclks = tof_get_vcsel_pulse_period(VcselPeriodPreRange);

        timeouts->msrc_dss_tcc_mclks = tof_read_reg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
        timeouts->msrc_dss_tcc_us =
            tof_timeout_mclks_to_microseconds(timeouts->msrc_dss_tcc_mclks,
                                       timeouts->pre_range_vcsel_period_pclks);

        timeouts->pre_range_mclks =
            tof_decode_timeout(tof_read_reg_16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
        timeouts->pre_range_us =
            tof_timeout_mclks_to_microseconds(timeouts->pre_range_mclks,
                                       timeouts->pre_range_vcsel_period_pclks);

        timeouts->final_range_vcsel_period_pclks = tof_get_vcsel_pulse_period(VcselPeriodFinalRange);

        timeouts->final_range_mclks =
            tof_decode_timeout(tof_read_reg_16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

        if (enables->pre_range)
        {
                timeouts->final_range_mclks -= timeouts->pre_range_mclks;
        }

        timeouts->final_range_us =
            tof_timeout_mclks_to_microseconds(timeouts->final_range_mclks,
                                       timeouts->final_range_vcsel_period_pclks);
}

uint16_t tof_decode_timeout(uint16_t reg_val)
{
        return (uint16_t)((reg_val & 0x00FF) <<
                          (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

uint16_t tof_encode_timeout(uint16_t timeout_mclks)
{
        uint32_t ls_byte = 0;
        uint16_t ms_byte = 0;

        if (timeout_mclks > 0)
        {
                ls_byte = timeout_mclks - 1;

                while ((ls_byte & 0xFFFFFF00) > 0)
                {
                        ls_byte >>= 1;
                        ms_byte++;
                }

                return (ms_byte << 8) | (ls_byte & 0xFF);
        }
        else { return 0; }
}

uint32_t tof_timeout_mclks_to_microseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
        uint32_t macro_period_ns = tof_calc_macro_period(vcsel_period_pclks);

        return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

uint32_t tof_timeout_microseconds_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
        uint32_t macro_period_ns = tof_calc_macro_period(vcsel_period_pclks);

        return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

bool tof_perform_single_ref_calibration(uint8_t vhv_init_byte)
{
        tof_write_reg(SYSRANGE_START, 0x01 | vhv_init_byte);

        tof_start_timeout();
        while ((tof_read_reg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
        {
                if (tof_check_timeout_expired()) { return false; }
        }

        tof_write_reg(SYSTEM_INTERRUPT_CLEAR, 0x01);

        tof_write_reg(SYSRANGE_START, 0x00);

        return true;
}