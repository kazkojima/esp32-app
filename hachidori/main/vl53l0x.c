/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#define I2C_PORT                        I2C_NUM_1
#define WRITE_BIT                       I2C_MASTER_WRITE
#define READ_BIT                        I2C_MASTER_READ
#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

// VL53L0X
#define VL53L0X_ADDRESS                 0x29

#define VL53L0X_MODEL_ID                0xc0
# define VL53L0X_MID                    0xee
#define VL53L0X_REVISION_ID             0xc2
# define VL53L0X_RID                    0x10
#define VL53L0X_PRE_RANGE_CONFIG        0x50
#define VL53L0X_FINAL_RANGE_CONFIG      0x70
#define VL53L0X_MSRC_CONFIG_CONTROL     0x60

#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP				0x46
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI  	0x51
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO  	0x52
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI	0x71
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO    0x72

#define VL53L0X_SYSRANGE_START          0x00
#define VL53L0X_SEQUENCE_CONFIG         0x01
#define VL53L0X_INTERRUPT_CONFIG_GPIO   0x0a
#define VL53L0X_INTERRUPT_CLEAR         0x0b
#define VL53L0X_INTERRUPT_STATUS        0x13
#define VL53L0X_RANGE_STATUS            0x14

#define RANGE_STATUS_OK                 0x0b


#define LONG_RANGE 1

#if 0
extern SemaphoreHandle_t i2c_sem;
#endif

// VL53L0X data are big endian
#define uint16_val(v, idx)(((uint16_t)v[2*idx] << 8) | v[2*idx+1])

static uint8_t vl53l0x_read(uint8_t reg)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1 )|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return 0;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1 )|READ_BIT, ACK_CHECK_EN);
    uint8_t rv = 0;
    i2c_master_read_byte(cmd, &rv, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        rv = 0;
    }
    return rv;
}

static bool vl53l0x_write(uint8_t reg, uint8_t val)
{
    uint8_t d = val;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, &d, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    return true;
}

static bool vl53l0x_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, buf + len - 1, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }
    return true;
}

static bool vl53l0x_writen(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (VL53L0X_ADDRESS << 1)|WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write(cmd, buf, len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000/portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        return false;
    }

    return true;
}

typedef struct cmd { uint8_t reg; uint8_t val; } cmd_t;
#define CMDLEN(c) (sizeof(c)/sizeof(cmd_t))

static cmd_t tuning[] = {
    { 0xFF, 0x01 },
    { 0x00, 0x00 },

    { 0xFF, 0x00 },
    { 0x09, 0x00 },
    { 0x10, 0x00 },
    { 0x11, 0x00 },

    { 0x24, 0x01 },
    { 0x25, 0xFF },
    { 0x75, 0x00 },

    { 0xFF, 0x01 },
    { 0x4E, 0x2C },
    { 0x48, 0x00 },
    { 0x30, 0x20 },

    { 0xFF, 0x00 },
    { 0x30, 0x09 },
    { 0x54, 0x00 },
    { 0x31, 0x04 },
    { 0x32, 0x03 },
    { 0x40, 0x83 },
    { 0x46, 0x25 },
    { 0x60, 0x00 },
    { 0x27, 0x00 },
    { 0x50, 0x06 },
    { 0x51, 0x00 },
    { 0x52, 0x96 },
    { 0x56, 0x08 },
    { 0x57, 0x30 },
    { 0x61, 0x00 },
    { 0x62, 0x00 },
    { 0x64, 0x00 },
    { 0x65, 0x00 },
    { 0x66, 0xA0 },

    { 0xFF, 0x01 },
    { 0x22, 0x32 },
    { 0x47, 0x14 },
    { 0x49, 0xFF },
    { 0x4A, 0x00 },

    { 0xFF, 0x00 },
    { 0x7A, 0x0A },
    { 0x7B, 0x00 },
    { 0x78, 0x21 },

    { 0xFF, 0x01 },
    { 0x23, 0x34 },
    { 0x42, 0x00 },
    { 0x44, 0xFF },
    { 0x45, 0x26 },
    { 0x46, 0x05 },
    { 0x40, 0x40 },
    { 0x0E, 0x06 },
    { 0x20, 0x1A },
    { 0x43, 0x40 },

    { 0xFF, 0x00 },
    { 0x34, 0x03 },
    { 0x35, 0x44 },

    { 0xFF, 0x01 },
    { 0x31, 0x04 },
    { 0x4B, 0x09 },
    { 0x4C, 0x05 },
    { 0x4D, 0x04 },

    { 0xFF, 0x00 },
    { 0x44, 0x00 },
    { 0x45, 0x20 },
    { 0x47, 0x08 },
    { 0x48, 0x28 },
    { 0x67, 0x00 },
    { 0x70, 0x04 },
    { 0x71, 0x01 },
    { 0x72, 0xFE },
    { 0x76, 0x00 },
    { 0x77, 0x00 },

    { 0xFF, 0x01 },
    { 0x0D, 0x01 },

    { 0xFF, 0x00 },
    { 0x80, 0x01 },
    { 0x01, 0xF8 },

    { 0xFF, 0x01 },
    { 0x8E, 0x01 },
    { 0x00, 0x01 },
    { 0xFF, 0x00 },
    { 0x80, 0x00 },
};

static void vl53l0x_write_cmd(struct cmd *p, size_t len)
{
    for (int i=0; i < len; i++, p++) {
        vl53l0x_write(p->reg, p->val);
    }
}

static bool vl53l0x_get_spad_info(uint8_t *count, bool *type)
{
    vl53l0x_write(0x80, 0x01);
    vl53l0x_write(0xFF, 0x01);
    vl53l0x_write(0x00, 0x00);

    vl53l0x_write(0xFF, 0x06);
    vl53l0x_write(0x83, vl53l0x_read(0x83) | 0x04);
    vl53l0x_write(0xFF, 0x07);
    vl53l0x_write(0x81, 0x01);

    vl53l0x_write(0x80, 0x01);

    vl53l0x_write(0x94, 0x6b);
    vl53l0x_write(0x83, 0x00);

    uint8_t tries = 5;
    while (vl53l0x_read(0x83) == 0x00) {
        tries--;
        if (tries == 0) {
            return false;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    vl53l0x_write(0x83, 0x01);
    uint8_t tmp = vl53l0x_read(0x92);

    *count = tmp & 0x7f;
    *type = (tmp >> 7) & 0x01;

    vl53l0x_write(0x81, 0x00);
    vl53l0x_write(0xFF, 0x06);
    vl53l0x_write(0x83, vl53l0x_read(0x83) & ~0x04);
    vl53l0x_write(0xFF, 0x01);
    vl53l0x_write(0x00, 0x01);

    vl53l0x_write(0xFF, 0x00);
    vl53l0x_write(0x80, 0x00);

    return true;
}

static uint8_t sequence_config;
#define ENABLE_MSRC (1<<2)
#define ENABLE_DSS (1<<3)
#define ENABLE_TCC (1<<4)
#define ENABLE_PRE_RANGE (1 << 6)
#define ENABLE_FINAL_RANGE (1 << 7)

static void vl53l0x_get_sequence_step_enables(void)
{
    sequence_config = vl53l0x_read(VL53L0X_SEQUENCE_CONFIG);
}

static uint32_t vcsel_period(uint8_t period)
{
  return (period + 1) << 1;
}

// range type
#define PRE_RANGE 0
#define FINAL_RANGE 1

static uint8_t vl53l0x_get_vcsel_pulse_period(int type)
{
    if (type == PRE_RANGE) {
        return vcsel_period(vl53l0x_read(VL53L0X_PRE_RANGE_CONFIG));
    } else if (type == FINAL_RANGE) {
        return vcsel_period(vl53l0x_read(VL53L0X_FINAL_RANGE_CONFIG));
    }
    return 255;
}

static uint32_t vl53l0x_timeout_mclks2us(uint16_t mclks, uint8_t pclks)
{
    uint32_t ns = (((uint32_t)2304 * pclks * 1655) + 500) / 1000;
    return  ((mclks * ns) + (ns / 2)) / 1000;
}

static uint16_t vl53l0x_decode_timeout(uint16_t reg_val)
{
    // format: "(LSByte * 2^MSByte) + 1"
    return (uint16_t)((reg_val & 0x00FF) <<
                      (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint8_t pre_range_vcsel_period_pclks;
static uint16_t msrc_dss_tcc_mclks;
static uint32_t msrc_dss_tcc_us;
static uint16_t pre_range_mclks;
static uint32_t pre_range_us;
static uint8_t final_range_vcsel_period_pclks;
static uint16_t final_range_mclks;
static uint32_t final_range_us;

static void vl53l0x_get_sequence_step_timeouts(void)
{
    uint16_t hi, lo;
    pre_range_vcsel_period_pclks = vl53l0x_get_vcsel_pulse_period(PRE_RANGE);
    msrc_dss_tcc_mclks = vl53l0x_read(VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    msrc_dss_tcc_us = vl53l0x_timeout_mclks2us(msrc_dss_tcc_mclks,
                                               pre_range_vcsel_period_pclks);
    hi = vl53l0x_read(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    lo = vl53l0x_read(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO);
    pre_range_mclks = vl53l0x_decode_timeout((hi << 8)|lo);
    pre_range_us = vl53l0x_timeout_mclks2us(pre_range_mclks,
                                            pre_range_vcsel_period_pclks);
    final_range_vcsel_period_pclks = vl53l0x_get_vcsel_pulse_period(FINAL_RANGE);
    hi = vl53l0x_read(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    lo = vl53l0x_read(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO);
    final_range_mclks = vl53l0x_decode_timeout((hi << 8)|lo);
    if (ENABLE_PRE_RANGE & sequence_config) {
        final_range_mclks -= pre_range_mclks;
    }
    final_range_us = vl53l0x_timeout_mclks2us(final_range_mclks,
                                              final_range_vcsel_period_pclks);
}

// Memoise
static uint32_t measurement_timing_budget_us;

static uint32_t vl53l0x_get_measurement_timing_budget(void)
{
    uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    // "Start and end overhead times always present"
    uint32_t budget_us = StartOverhead + EndOverhead;

    vl53l0x_get_sequence_step_enables();
    vl53l0x_get_sequence_step_timeouts();

    if (ENABLE_TCC & sequence_config) {
        budget_us += (msrc_dss_tcc_us + TccOverhead);
    }

    if (ENABLE_DSS & sequence_config) {
        budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
    } else if (ENABLE_MSRC & sequence_config) {
        budget_us += (msrc_dss_tcc_us + MsrcOverhead);
    }

    if (ENABLE_PRE_RANGE & sequence_config) {
        budget_us += (pre_range_us + PreRangeOverhead);
    }

    if (ENABLE_FINAL_RANGE & sequence_config) {
        budget_us += (final_range_us + FinalRangeOverhead);
    }

    measurement_timing_budget_us = budget_us; // store for internal reuse
    return budget_us;
}

static uint32_t vl53l0x_timeout_us2mclks(uint32_t us, uint8_t pclks)
{
    uint32_t ns = ((((uint32_t)2304 * pclks * 1655) + 500) / 1000);
    return (((us * 1000) + (ns / 2)) / ns);
}

static uint16_t vl53l0x_encode_timeout(uint16_t mclks)
{
    // format: "(LSByte * 2^MSByte) + 1"
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (mclks > 0) {
        ls_byte = mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}

// Memoise
static uint32_t measurement_timing_budget_us;

static bool vl53l0x_set_measurement_timing_budget(uint32_t budget_us)
{
    uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
    uint16_t const EndOverhead        = 960;
    uint16_t const MsrcOverhead       = 660;
    uint16_t const TccOverhead        = 590;
    uint16_t const DssOverhead        = 690;
    uint16_t const PreRangeOverhead   = 660;
    uint16_t const FinalRangeOverhead = 550;

    uint32_t const MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) { return false; }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    vl53l0x_get_sequence_step_enables();
    vl53l0x_get_sequence_step_timeouts();

    if (ENABLE_TCC & sequence_config) {
        used_budget_us += (msrc_dss_tcc_us + TccOverhead);
    }

    if (ENABLE_DSS & sequence_config) {
        used_budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
    } else if (ENABLE_MSRC & sequence_config) {
        used_budget_us += (msrc_dss_tcc_us + MsrcOverhead);
    }

    if (ENABLE_PRE_RANGE & sequence_config) {
        used_budget_us += (pre_range_us + PreRangeOverhead);
    }

    if (ENABLE_FINAL_RANGE & sequence_config) {
        used_budget_us += FinalRangeOverhead;

        // "Note that the final range timeout is determined by the timing
        // budget and the sum of all other timeouts within the sequence.
        // If there is no room for the final range timeout, then an error
        // will be set. Otherwise the remaining time will be applied to
        // the final range."

        if (used_budget_us > budget_us) {
            // "Requested timeout too big."
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t final_range_timeout_mclks =
            vl53l0x_timeout_us2mclks(final_range_timeout_us,
                                     final_range_vcsel_period_pclks);

        if (ENABLE_PRE_RANGE & sequence_config) {
            final_range_timeout_mclks += pre_range_mclks;
        }

        uint16_t timeout = vl53l0x_encode_timeout(final_range_timeout_mclks);
        vl53l0x_write(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                      (timeout >> 8));
        vl53l0x_write(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO,
                      (timeout & 0xff));

        // set_sequence_step_timeout() end
        measurement_timing_budget_us = budget_us; // store for internal reuse
    }
    return true;
}

static bool vl53l0x_perform_single_ref_calibration(uint8_t vhv_init_byte)
{
    // VL53L0X_REG_SYSRANGE_MODE_START_STOP
    vl53l0x_write(VL53L0X_SYSRANGE_START, 0x01 | vhv_init_byte);

    uint8_t tries = 20;
    while ((vl53l0x_read(VL53L0X_INTERRUPT_STATUS) & 0x07) == 0) {
        if (tries-- == 0) {
            return false;
        }
        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    vl53l0x_write(VL53L0X_INTERRUPT_CLEAR, 0x01);
    vl53l0x_write(VL53L0X_SYSRANGE_START, 0x00);

    return true;
}

static bool vl53l0x_set_vcsel_pulse_period(int type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = (period_pclks >> 1) - 1;

    vl53l0x_get_sequence_step_enables();
    vl53l0x_get_sequence_step_timeouts();

    // "Apply specific settings for the requested clock period"
    // "Re-calculate and apply timeouts, in macro periods"

    // "When the VCSEL period for the pre or final range is changed,
    // the corresponding timeout must be read from the device using
    // the current VCSEL period, then the new VCSEL period can be
    // applied. The timeout then must be written back to the device
    // using the new VCSEL period.
    //
    // For the MSRC timeout, the same applies - this timeout being
    // dependant on the pre-range vcsel period."

    if (type == PRE_RANGE) {
        // "Set phase check limits"
        switch (period_pclks) {
        case 12:
            vl53l0x_write(0x57, 0x18);
            break;
        case 14:
            vl53l0x_write(0x57, 0x30);
            break;
        case 16:
            vl53l0x_write(0x57, 0x40);
            break;
        case 18:
            vl53l0x_write(0x57, 0x50);
            break;
        default:
            // invalid period
            return false;
        }
        vl53l0x_write(0x56, 0x08);

        // apply new VCSEL period
        vl53l0x_write(VL53L0X_PRE_RANGE_CONFIG, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

        uint16_t new_pre_range_timeout_mclks =
            vl53l0x_timeout_us2mclks(pre_range_us, period_pclks);

        uint16_t tout = vl53l0x_encode_timeout(new_pre_range_timeout_mclks);
        vl53l0x_write(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, (tout>>8));
        vl53l0x_write(VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, (tout&0xff));

        // set_sequence_step_timeout() end

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

        uint16_t new_msrc_timeout_mclks =
            vl53l0x_timeout_us2mclks(msrc_dss_tcc_us, period_pclks);

        vl53l0x_write(VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP,
                      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

        // set_sequence_step_timeout() end
    } else if (type == FINAL_RANGE) {
        switch (period_pclks) {
        case 8:
            vl53l0x_write(0x48, 0x10);
            vl53l0x_write(0x47,  0x08);
            vl53l0x_write(0x32, 0x02);
            vl53l0x_write(0x30, 0x0C);
            vl53l0x_write(0xFF, 0x01);
            vl53l0x_write(0x30, 0x30);
            vl53l0x_write(0xFF, 0x00);
            break;
        case 10:
            vl53l0x_write(0x48, 0x28);
            vl53l0x_write(0x47,  0x08);
            vl53l0x_write(0x32, 0x03);
            vl53l0x_write(0x30, 0x09);
            vl53l0x_write(0xFF, 0x01);
            vl53l0x_write(0x30, 0x20);
            vl53l0x_write(0xFF, 0x00);
            break;
        case 12:
            vl53l0x_write(0x48, 0x38);
            vl53l0x_write(0x47,  0x08);
            vl53l0x_write(0x32, 0x03);
            vl53l0x_write(0x30, 0x08);
            vl53l0x_write(0xFF, 0x01);
            vl53l0x_write(0x30, 0x20);
            vl53l0x_write(0xFF, 0x00);
            break;
        case 14:
            vl53l0x_write(0x48, 0x48);
            vl53l0x_write(0x47,  0x08);
            vl53l0x_write(0x32, 0x03);
            vl53l0x_write(0x30, 0x07);
            vl53l0x_write(0xFF, 0x01);
            vl53l0x_write(0x30, 0x20);
            vl53l0x_write(0xFF, 0x00);
            break;

        default:
            // invalid period
            return false;
        }

        // apply new VCSEL period
        vl53l0x_write(VL53L0X_FINAL_RANGE_CONFIG, vcsel_period_reg);

        // update timeouts

        // set_sequence_step_timeout() begin
        // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

        // "For the final range timeout, the pre-range timeout
        //  must be added. To do this both final and pre-range
        //  timeouts must be expressed in macro periods MClks
        //  because they have different vcsel periods."

        uint16_t new_final_range_timeout_mclks =
            vl53l0x_timeout_us2mclks(final_range_us, period_pclks);

        if (ENABLE_PRE_RANGE & sequence_config) {
            new_final_range_timeout_mclks += pre_range_mclks;
        }

        uint16_t to = vl53l0x_encode_timeout(new_final_range_timeout_mclks);
        vl53l0x_write(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, (to>>8));
        vl53l0x_write(VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, (to&0xff));

        // set_sequence_step_timeout end
    } else {
        // invalid type
        return false;
    }

    // "Finally, the timing budget must be re-applied"
    vl53l0x_set_measurement_timing_budget(measurement_timing_budget_us);

    // "Perform the phase calibration. This is needed after changing on vcsel period."
    // VL53L0X_perform_phase_calibration() begin

    uint8_t sequence_config = vl53l0x_read(VL53L0X_SEQUENCE_CONFIG);
    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0x02);
    vl53l0x_perform_single_ref_calibration(0x0);
    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, sequence_config);

    // VL53L0X_perform_phase_calibration() end

    return true;
}

static uint8_t stop_variable;

static void vl53l0x_start_continuous(void)
{
    vl53l0x_write(0x80, 0x01);
    vl53l0x_write(0xFF, 0x01);
    vl53l0x_write(0x00, 0x00);
    vl53l0x_write(0x91, stop_variable);
    vl53l0x_write(0x00, 0x01);
    vl53l0x_write(0xFF, 0x00);
    vl53l0x_write(0x80, 0x00);

    // continuous back-to-back mode
    vl53l0x_write(VL53L0X_SYSRANGE_START, 0x02);
}

static bool vl53l0x_init(void)
{
    uint8_t mid = vl53l0x_read(VL53L0X_MODEL_ID);
    vTaskDelay(10/portTICK_PERIOD_MS);

    uint8_t rid = vl53l0x_read(VL53L0X_REVISION_ID);
    vTaskDelay(10/portTICK_PERIOD_MS);

    if (mid != VL53L0X_MID || rid != VL53L0X_RID) {
        printf("VL53L0X id values %02x %02x\n", mid, rid);
        return false;
    }

    // "Set I2C standard mode"
    vl53l0x_write(0x88, 0x00);

    vl53l0x_write(0x80, 0x01);
    vl53l0x_write(0xFF, 0x01);
    vl53l0x_write(0x00, 0x00);
    stop_variable = vl53l0x_read(0x91);
    vl53l0x_write(0x00, 0x01);
    vl53l0x_write(0xFF, 0x00);
    vl53l0x_write(0x80, 0x00);

    // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
    vl53l0x_write(VL53L0X_MSRC_CONFIG_CONTROL,
                  vl53l0x_read(VL53L0X_MSRC_CONFIG_CONTROL) | 0x12);

#if LONG_RANGE
    // set final range signal rate limit to 0.1 MCPS
    uint16_t lim = (uint16_t)(0.1 * (1 << 7));
#else
    // set final range signal rate limit to 0.25 MCPS
    uint16_t lim = (uint16_t)(0.25 * (1 << 7));
#endif
    uint8_t buf[2];
    buf[0] = lim>>8;
    buf[1] = lim & 0xff;
    vl53l0x_writen(VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                   buf, 2);

    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0xFF);

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!vl53l0x_get_spad_info(&spad_count, &spad_type_is_aperture)) {
        printf("Failed to get SPAD info\n");
        return false;
    }

    // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
    // the API, but the same data seems to be more easily readable from
    // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
    uint8_t ref_spad_map[6];
    if (!vl53l0x_readn(0xB0, ref_spad_map, 6)) {
        printf("Failed to read SPAD map\n");
        return false;
    }

    // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
    vl53l0x_write(0xFF, 0x01);
    vl53l0x_write(0x4F, 0x00);
    vl53l0x_write(0x4E, 0x2C);
    vl53l0x_write(0xFF, 0x00);
    vl53l0x_write(0xB6, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            // This bit is lower than the first one that should be enabled, or
            // (reference_spad_count) bits have already been enabled, so zero this bit
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    vl53l0x_writen(0xB0, ref_spad_map, 6);

    vl53l0x_write_cmd(tuning, CMDLEN(tuning));

    vl53l0x_write(VL53L0X_INTERRUPT_CONFIG_GPIO, 0x04);
    vl53l0x_write(0x84, vl53l0x_read(0x84) & ~0x10); // active low
    vl53l0x_write(VL53L0X_INTERRUPT_CLEAR, 0x01);

    // -- VL53L0X_SetGpioConfig() end

    measurement_timing_budget_us = vl53l0x_get_measurement_timing_budget();

    // "Disable MSRC and TCC by default"
    // MSRC = Minimum Signal Rate Check
    // TCC = Target CentreCheck
    // -- VL53L0X_SetSequenceStepEnable() begin

    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0xE8);

    // -- VL53L0X_SetSequenceStepEnable() end

    // "Recalculate timing budget"
    vl53l0x_set_measurement_timing_budget(measurement_timing_budget_us);

    // VL53L0X_StaticInit() end

    // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

    // -- VL53L0X_perform_vhv_calibration() begin

    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0x01);
    if (!vl53l0x_perform_single_ref_calibration(0x40)) {
        printf("Failed SingleRefCalibration1\n");
        return false;
    }

    // -- VL53L0X_perform_vhv_calibration() end

    // -- VL53L0X_perform_phase_calibration() begin

    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0x02);
    if (!vl53l0x_perform_single_ref_calibration(0x00)) {
        printf("Failed SingleRefCalibration2\n");
        return false;
    }

    // -- VL53L0X_perform_phase_calibration() end

    // "restore the previous Sequence Config"
    vl53l0x_write(VL53L0X_SEQUENCE_CONFIG, 0xE8);

    vl53l0x_start_continuous();

    return true;
}

extern int sockfd;
extern xQueueHandle pkt_queue;
extern int pkt_queue_error;

void rn_task(void *pvParameters)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // ? Wait POR
    vTaskDelay(500/portTICK_PERIOD_MS);

#if 0
    xSemaphoreTake(i2c_sem, portMAX_DELAY);
#endif
    if (!vl53l0x_init()) {
#if 0
        xSemaphoreGive(i2c_sem);
#endif
        vTaskDelete(NULL);
    }

#if LONG_RANGE
    vl53l0x_set_vcsel_pulse_period(PRE_RANGE, 18);
    vl53l0x_set_vcsel_pulse_period(FINAL_RANGE, 14);
#endif
#if 0
    xSemaphoreGive(i2c_sem);
#endif

    vTaskDelay(100/portTICK_PERIOD_MS);

    union { float f; uint8_t bytes[sizeof(float)];} nof;
    nof.f = -2.0f;
    struct B3packet pkt;
    for (int i = 0; i < B3SIZE/sizeof(float); i++) {
        memcpy(&pkt.data[0], nof.bytes, sizeof(nof));
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    int wait_count = 0;
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (33/portTICK_PERIOD_MS > lap) {
            vTaskDelay(33/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        uint16_t d = 0;
        union { float f; uint8_t bytes[sizeof(float)];} df;

#if 0
        xSemaphoreTake(i2c_sem, portMAX_DELAY);
#endif
        if ((vl53l0x_read(VL53L0X_INTERRUPT_STATUS) & 0x07) == 0) {
            if (++wait_count > 6) {
                vl53l0x_start_continuous();
                wait_count = 0;
            }
#if 0
            xSemaphoreGive(i2c_sem);
#endif
            continue;
        }

        wait_count = 0;

        uint8_t buf[2];
        vl53l0x_readn(VL53L0X_RANGE_STATUS+10, buf, 2);
        d = uint16_val(buf, 0);
        vl53l0x_write(VL53L0X_INTERRUPT_CLEAR, 0x01);
#if 0
        xSemaphoreGive(i2c_sem);
#endif

        //printf("range %d\n", d);
        if (d < 8000) {
            df.f = ((float)d) * 0.1f;
        } else {
            df.f = -1.0f;
        }
        memcpy(&pkt.data[0], df.bytes, sizeof(df));

        // Send it
        pkt.head = B3HEADER;
        pkt.tos = TOS_RANGE;

        if (xQueueSend(pkt_queue, &pkt, 0) != pdTRUE) {
            //printf("fail to queue Range finder packet\n");
            ++pkt_queue_error;
        }
    }
}
