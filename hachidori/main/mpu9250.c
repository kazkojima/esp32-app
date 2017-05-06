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
#include "esp_heap_alloc_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"

#include "b3packet.h"

#include "MadgwickAHRS.h"
#include "kfacc.h"

#include "pwm.h"
#include "battery.h"

#if 1
#define ROTATION_YAW	270
//#define  ROTATION_YAW	0
#endif

// MPU9250
#define MPU9250_ID      0x71

// MPU9250 registers
#define SMPLRT_DIV      0x19
#define MPU_CONFIG      0x1A
#define GYRO_CONFIG     0x1B
#define ACCEL_CONFIG    0x1C

#define I2C_MST_CTRL    0x24
#define I2C_SLV0_ADDR   0x25
#define I2C_SLV0_REG    0x26
#define I2C_SLV0_CTRL   0x27

#define I2C_SLV4_CTRL   0x34

#define INT_PIN_CFG     0x37
#define INT_ENABLE      0x38
#define INT_STATUS      0x3A

#define ACCEL_XOUT_H    0x3B
#define ACCEL_XOUT_L    0x3C
#define ACCEL_YOUT_H    0x3D
#define ACCEL_YOUT_L    0x3E
#define ACCEL_ZOUT_H    0x3F
#define ACCEL_ZOUT_L    0x40
#define TEMP_OUT_H      0x41
#define TEMP_OUT_L      0x42
#define GYRO_XOUT_H     0x43
#define GYRO_XOUT_L     0x44
#define GYRO_YOUT_H     0x45
#define GYRO_YOUT_L     0x46
#define GYRO_ZOUT_H     0x47
#define GYRO_ZOUT_L     0x48

#define EXT_SENS_DATA_00 0x49
#define I2C_SLV0_DO     0x63
#define I2C_MST_DELAY_CTRL 0x67

#define USER_CTRL       0x6A
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C

#define WHO_IM_I        0x75

// AK8963
#define AK8963_I2C_ADDR 0x0c
#define AK8963_ID       0x48

/* AK8963 registers */
#define AK8963_WIA      0x00
#define AK8963_HXL      0x03
#define AK8963_CNTL1    0x0A
#define AK8963_CNTL2    0x0B
#define AK8963_ASAX     0x10

#define GRAVITY_MSS     9.80665f

// accelerometer scaling for 16g range
#define MPU9250_ACCEL_SCALE_1G    (GRAVITY_MSS / 2048.0f)

/*
 *  PS-MPU-9250A-00.pdf, page 8, lists LSB sensitivity of
 *  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
static const float GYRO_SCALE = 0.0174532f / 16.4f;

#define AK8963_MILLIGAUSS_SCALE 10.0f
static const float ADC_16BIT_RESOLUTION = 0.15f;

// MPU9250 IMU data are big endian
#define be16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx] << 8) | v[2*idx+1]))
// AK8963 data are little endian
#define le16_val(v, idx) ((int16_t)(((uint16_t)v[2*idx+1] << 8) | v[2*idx]))

extern spi_device_handle_t spi_a;

static uint8_t mpu9250_read(uint8_t reg)
{
    esp_err_t ret;
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length=16;
    trans.tx_data[0] = reg | 0x80;
    trans.tx_data[1] = 0x00;
    trans.flags=SPI_TRANS_USE_TXDATA|SPI_TRANS_USE_RXDATA;
    //printf("do transfer\n");
    ret=spi_device_transmit(spi_a, &trans);
    assert(ret==ESP_OK);

    return trans.rx_data[1];
}

static bool mpu9250_write(uint8_t reg, uint8_t val)
{
    esp_err_t ret;
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length=16;
    trans.tx_data[0] = reg & 0x7f;
    trans.tx_data[1] = val;
    trans.flags=SPI_TRANS_USE_TXDATA;
    //printf("do transfer\n");
    ret=spi_device_transmit(spi_a, &trans);
    if (ret!=ESP_OK) {
        return false;
    }
    return true;
}

static bool mpu9250_readn(uint8_t reg, uint8_t *buf, size_t len)
{
    esp_err_t ret;
    spi_transaction_t trans;
    uint8_t *tbuf = pvPortMallocCaps(len + 1, MALLOC_CAP_DMA);
    uint8_t *rbuf = pvPortMallocCaps(len + 1, MALLOC_CAP_DMA);
    tbuf[0] = reg | 0x80;
    memset(&tbuf[1], 0, len);
    memset(&trans, 0, sizeof(spi_transaction_t));
    trans.length = 8 + 8*len;
    trans.tx_buffer = tbuf;
    trans.rx_buffer = rbuf;
    trans.flags=0;
    //printf("do transfer\n");
    //Queue all transactions.
    ret=spi_device_transmit(spi_a, &trans);
    free(tbuf);
    if (ret!=ESP_OK) {
        free(rbuf);
        return false;
    }
    memcpy(buf, rbuf+1, len);
    free(rbuf);
    return true;
}

static bool mpu9250_ready(void)
{
    uint8_t val = mpu9250_read(INT_STATUS);
    return (val & 1);
}

struct sample {
    uint8_t int_status;
    uint8_t d[14];
};

static bool mpu9250_read_sample(struct sample *rx)
{
    return (mpu9250_readn(INT_STATUS, (uint8_t *)rx, sizeof(struct sample)));
}

static void mpu9250_start(void)
{
    mpu9250_write(PWR_MGMT_2, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // No LPF
    mpu9250_write(MPU_CONFIG, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Sample rate 1000Hz
    mpu9250_write(SMPLRT_DIV, 0);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Gyro 2000dps
    mpu9250_write(GYRO_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Accel full scale 16g
    mpu9250_write(ACCEL_CONFIG, 3<<3);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // INT enable on RDY
    mpu9250_write(INT_ENABLE, 1);
    vTaskDelay(1/portTICK_PERIOD_MS);

    uint8_t val = mpu9250_read(INT_PIN_CFG);
    val |= 0x30;
    mpu9250_write(INT_PIN_CFG, val);
    vTaskDelay(1/portTICK_PERIOD_MS);

    // Enable DMP
    val = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, val | (1<<7));
    vTaskDelay(1/portTICK_PERIOD_MS);
}

static void slv0_readn(uint8_t reg, uint8_t size)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_ADDR, 0x80 | AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | size);
}

static void slv0_write1(uint8_t reg, uint8_t out)
{
    mpu9250_write(I2C_SLV0_CTRL, 0);
    mpu9250_write(I2C_SLV0_DO, out);
    mpu9250_write(I2C_SLV0_ADDR, AK8963_I2C_ADDR);
    mpu9250_write(I2C_SLV0_REG, reg);
    mpu9250_write(I2C_SLV0_CTRL, 0x80 | 1);
}

static uint8_t ak8963_read(uint8_t reg)
{
    slv0_readn(reg, 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    uint8_t rv = mpu9250_read(EXT_SENS_DATA_00);

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return rv;
}

static void ak8963_write(uint8_t reg, uint8_t val)
{
    slv0_write1(reg, val);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    mpu9250_write(I2C_SLV0_CTRL, 0);
}

struct ak_sample {
    uint8_t d[6];
    uint8_t st2;
};

static void ak8963_read_sample_start(void)
{
    slv0_readn(AK8963_HXL, 7);
}

static bool ak8963_read_sample(struct ak_sample *rx)
{
    bool rv;
    rv = mpu9250_readn(EXT_SENS_DATA_00, (uint8_t *)rx,
                       sizeof(struct ak_sample));

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return rv;
}

struct ak_asa {
    uint8_t a[3];
};

static bool ak8963_read_asa(struct ak_asa *rx)
{
    slv0_readn(AK8963_ASAX, 3);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    bool rv;
    rv = mpu9250_readn(EXT_SENS_DATA_00, (uint8_t *)rx, sizeof(struct ak_asa));
    if (rv != true) {
        return rv;
    }

    mpu9250_write(I2C_SLV0_CTRL, 0);
    return true;
}

static struct ak_asa ak8963_asa;
static float ak8963_calib[3];

static void ak8963_start(void)
{
    // Reset
    // ak8963_write(AK8963_CNTL2, 0x01);

    // Calibrate - fuse, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x1f);
    ak8963_read_asa(&ak8963_asa);

    for (int i = 0; i < 3; i++) {
        float data = ak8963_asa.a[i];
        // factory sensitivity
        ak8963_calib[i] = ((data - 128) / 256 + 1);
        // adjust by ADC sensitivity and convert to milligauss
        ak8963_calib[i] *= ADC_16BIT_RESOLUTION * AK8963_MILLIGAUSS_SCALE;
    }

    // Setup mode - continuous mode 2, 16-bit adc
    ak8963_write(AK8963_CNTL1, 0x16);
    // Start measurement
}

extern int sockfd;
extern SemaphoreHandle_t send_sem;

#if 1
static int maybe_inverted;
bool maybe_landed = true;
#endif

void imu_task(void *arg)
{
    while (sockfd < 0) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    uint8_t rv;
    rv = mpu9250_read(WHO_IM_I);
    if (rv != MPU9250_ID) {
        printf("MPU9250: Wrong id: %02x\n", rv);
    }

    uint8_t tries;
    for (tries = 0; tries < 5; tries++) {
        // Disable master I2C here
        if ((rv = mpu9250_read(USER_CTRL)) & (1<<5)) {
            mpu9250_write(USER_CTRL, rv &~ (1<<5));
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }

        // Reset
        mpu9250_write(PWR_MGMT_1, 0x80);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Disable I2C interface
        mpu9250_write(USER_CTRL, 0x10);
        vTaskDelay(100 / portTICK_PERIOD_MS);

        // Wake up with appropriate clock
        mpu9250_write(PWR_MGMT_1, 0x03);
        vTaskDelay(5 / portTICK_PERIOD_MS);
        if (mpu9250_read(PWR_MGMT_1) == 0x03)
            break;

        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (mpu9250_ready())
            break;
    }

    if (tries == 5) {
        printf("Failed to boot MPU9250 5 times");
    }

    mpu9250_start();

    // Configure slaves
    // Set I2C_MST_EN, MST_P_NSR and set bus speed to 400kHz
    rv = mpu9250_read(USER_CTRL);
    mpu9250_write(USER_CTRL, rv | (1<<5));
    mpu9250_write(I2C_MST_CTRL, (1<<4)|13);
    // Sample rate 100Hz
    mpu9250_write(I2C_SLV4_CTRL, 9);
    mpu9250_write(I2C_MST_DELAY_CTRL, 0x0f);

    rv = ak8963_read(AK8963_WIA);
    if (rv != AK8963_ID) {
        printf("AK8963: Wrong id: %02x\n", rv);
    }

    ak8963_start();

    ak8963_read_sample_start();
    vTaskDelay(10/portTICK_PERIOD_MS);

    struct sample rx;
    struct ak_sample akrx;
    struct B3packet pkt;
    int count = 0;
    int fscount = 0;
    float gx, gy, gz, ax, ay, az, mx, my, mz;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        TickType_t lap = xTaskGetTickCount() - xLastWakeTime;
        if (1/portTICK_PERIOD_MS > lap) {
            vTaskDelay(1/portTICK_PERIOD_MS - lap);
        }
        xLastWakeTime = xTaskGetTickCount();

        if (!mpu9250_ready())
            continue;

#if 0
        if (low_battery) {
            // Sleep
            mpu9250_write(PWR_MGMT_1, 0x40);
            printf("low_battery: stop imu_task\n");
            vTaskDelete(NULL);
        }
#endif
        mpu9250_read_sample(&rx);

        // adjust and serialize floats into packet bytes
        // skew accel/gyro frames so to match AK8963 NED frame
        union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
        ux.f = ((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#elif (ROTATION_YAW == 270)
        ux.f = ((float)be16_val(rx.d, 0)) * MPU9250_ACCEL_SCALE_1G;
        uy.f = -((float)be16_val(rx.d, 1)) * MPU9250_ACCEL_SCALE_1G;
        uz.f = -((float)be16_val(rx.d, 2)) * MPU9250_ACCEL_SCALE_1G;
#else
#error "bad ROTATION_YAW value"
#endif
        ax = ux.f; ay = uy.f; az = uz.f;
        memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[8], uz.bytes, sizeof(uz));
#if (ROTATION_YAW == 0)
        ux.f = ((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uy.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#elif (ROTATION_YAW == 270)
        ux.f = ((float)be16_val(rx.d, 4)) * GYRO_SCALE;
        uy.f = -((float)be16_val(rx.d, 5)) * GYRO_SCALE;
        uz.f = -((float)be16_val(rx.d, 6)) * GYRO_SCALE;
#else
#error "bad ROTATION_YAW value"
#endif
        gx = ux.f; gy = uy.f; gz = uz.f;
        memcpy(&pkt.data[12], ux.bytes, sizeof(ux));
        memcpy(&pkt.data[16], uy.bytes, sizeof(uy));
        memcpy(&pkt.data[20], uz.bytes, sizeof(uz));

        pkt.head = B3HEADER;
        pkt.tos = TOS_IMU;
        xSemaphoreTake(send_sem, portMAX_DELAY);
        int n = send(sockfd, &pkt, sizeof(pkt), 0);
        if (n < 0) {
        }
        xSemaphoreGive(send_sem);

        if ((count++ % 10) == 0) {
            ak8963_read_sample(&akrx);
            // trigger next sampling of ak8963
            ak8963_read_sample_start();

            // skip if overflow
            if (akrx.st2 & 0x08) {
                continue;
            }

            // adjust and serialize floats into packet bytes
            union { float f; uint8_t bytes[sizeof(float)];} ux, uy, uz;
#if (ROTATION_YAW == 0)
            ux.f = ((float)le16_val(akrx.d, 0)) * ak8963_calib[0];
            uy.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#elif (ROTATION_YAW == 270)
            ux.f = ((float)le16_val(akrx.d, 1)) * ak8963_calib[0];
            uy.f = -((float)le16_val(akrx.d, 0)) * ak8963_calib[1];
            uz.f = ((float)le16_val(akrx.d, 2)) * ak8963_calib[2];
#else
#error "bad ROTATION_YAW value"
#endif
            mx = ux.f; my = uy.f; mz = uz.f;
            memcpy(&pkt.data[0], ux.bytes, sizeof(ux));
            memcpy(&pkt.data[4], uy.bytes, sizeof(uy));
            memcpy(&pkt.data[8], uz.bytes, sizeof(uz));

            pkt.head = B3HEADER;
            pkt.tos = TOS_MAG;
            xSemaphoreTake(send_sem, portMAX_DELAY);
            int n = send(sockfd, &pkt, sizeof(pkt), 0);
            if (n < 0) {
            }
            xSemaphoreGive(send_sem);

#if 1
            if (prepare_failsafe) {
                beta = (fscount++ < 1000) ? 2.0f : 0.2f;
                MadgwickAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
                KFACCupdate(ax, ay, az);
            } else {
                fscount = 0;
            }
            // DISARM on inversion
            if (-az < -GRAVITY_MSS * 0.4) {
                if(++maybe_inverted > INVERSION_WM) {
                    if (in_arm) {
                        in_arm = false;
                        printf("disarm\n");
                    }
                }
            } else {
                maybe_inverted = 0;
            }
            // Check if maybe-landed
            if ((ax < 0.8 && ax > -0.8)
                && (ay < 0.8 && ay > -0.8)
                && (-az < GRAVITY_MSS + 0.6 && -az > GRAVITY_MSS - 0.6)
                && (gx < 0.05 && gx > -0.05)
                && (gy < 0.05 && gy > -0.05)
                && (gz < 0.05 && gz > -0.05)) {
                maybe_landed = true;
            } else {
                maybe_landed = false;
            }
#endif
        }
    }
}
