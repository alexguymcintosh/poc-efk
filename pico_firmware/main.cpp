#include <cstdio>
#include <cstdint>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"

static constexpr uint I2C_SDA_PIN  = 4;
static constexpr uint I2C_SCL_PIN  = 5;
static constexpr uint I2C_BAUDRATE = 100 * 1000;

static constexpr uint UART_TX_PIN = 0;
static constexpr uint UART_RX_PIN = 1;
#define GPS_UART uart0

static constexpr uint BAUD_CANDIDATES[] = { 9600, 38400, 57600, 115200 };

// ---------- MPU-6050 ----------
static constexpr uint8_t MPU6050_ADDR     = 0x68;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;

static constexpr float ACCEL_LSB_PER_G  = 16384.0f;
static constexpr float GYRO_LSB_PER_DPS = 131.0f;
static constexpr float G_TO_MS2         = 9.80665f;

// ---------- HMC5883L ----------
static constexpr uint8_t HMC_ADDR      = 0x1E;
static constexpr uint8_t HMC_REG_CFGA  = 0x00;
static constexpr uint8_t HMC_REG_CFGB  = 0x01;
static constexpr uint8_t HMC_REG_MODE  = 0x02;
static constexpr uint8_t HMC_REG_DATA  = 0x03; // X_MSB X_LSB Z_MSB Z_LSB Y_MSB Y_LSB

// ---------- QMC5883L ----------
static constexpr uint8_t QMC_ADDR      = 0x0D;
static constexpr uint8_t QMC_REG_DATA  = 0x00; // X_LSB X_MSB Y_LSB Y_MSB Z_LSB Z_MSB (little-endian)
static constexpr uint8_t QMC_REG_CTRL1 = 0x09;
static constexpr uint8_t QMC_REG_CTRL2 = 0x0A;
static constexpr uint8_t QMC_REG_SR    = 0x0B;

enum class MagChip { NONE, HMC5883L, QMC5883L };
static MagChip g_mag_chip = MagChip::NONE;

// ---------- GPS ----------
static constexpr size_t NMEA_BUF_SIZE = 128;
static char nmea_buf[NMEA_BUF_SIZE];
static size_t nmea_len = 0;

// ---------- Calibration ----------
static constexpr int GYRO_CAL_SAMPLES = 500;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float gyro_bias_z = 0.0f;

static constexpr int MAG_CAL_SAMPLES = 100;
static float mag_bias_x = 0.0f;
static float mag_bias_y = 0.0f;
static float mag_bias_z = 0.0f;

// ---------- I2C helpers ----------

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    return i2c_write_blocking(i2c0, addr, buf, 2, false) == 2;
}

static bool i2c_read_burst(uint8_t addr, uint8_t reg, uint8_t *dst, size_t len) {
    if (i2c_write_blocking(i2c0, addr, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c0, addr, dst, len, false) == static_cast<int>(len);
}

static bool i2c_probe(uint8_t addr) {
    uint8_t dummy;
    return i2c_read_blocking(i2c0, addr, &dummy, 1, false) == 1;
}

static int16_t be16(const uint8_t *p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) << 8 | p[1]);
}

static int16_t le16(const uint8_t *p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[1]) << 8 | p[0]);
}

// ---------- IMU ----------

static bool mpu_write_reg(uint8_t reg, uint8_t value) {
    return i2c_write_reg(MPU6050_ADDR, reg, value);
}

static bool mpu_read_burst(uint8_t reg, uint8_t *dst, size_t len) {
    return i2c_read_burst(MPU6050_ADDR, reg, dst, len);
}

static void read_and_print_imu() {
    uint8_t raw[14];
    if (!mpu_read_burst(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        printf("IMU: read failed\n");
        return;
    }

    float ax = (be16(&raw[0]) / ACCEL_LSB_PER_G) * G_TO_MS2;
    float ay = (be16(&raw[2]) / ACCEL_LSB_PER_G) * G_TO_MS2;
    float az = (be16(&raw[4]) / ACCEL_LSB_PER_G) * G_TO_MS2;
    float temp_c = be16(&raw[6]) / 340.0f + 36.53f;
    float gx = be16(&raw[8])  / GYRO_LSB_PER_DPS - gyro_bias_x;
    float gy = be16(&raw[10]) / GYRO_LSB_PER_DPS - gyro_bias_y;
    float gz = be16(&raw[12]) / GYRO_LSB_PER_DPS - gyro_bias_z;

    printf("IMU:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f\n",
           ax, ay, az, gx, gy, gz, temp_c);
}

static void calibrate_gyro() {
    printf("IMU: calibrating gyro — keep sensor stationary (%d samples)...\n",
           GYRO_CAL_SAMPLES);

    double sx = 0.0, sy = 0.0, sz = 0.0;
    int taken = 0;

    for (int i = 0; i < GYRO_CAL_SAMPLES; ++i) {
        uint8_t raw[14];
        if (mpu_read_burst(REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
            sx += be16(&raw[8])  / GYRO_LSB_PER_DPS;
            sy += be16(&raw[10]) / GYRO_LSB_PER_DPS;
            sz += be16(&raw[12]) / GYRO_LSB_PER_DPS;
            ++taken;
        }
        sleep_ms(2);
    }

    if (taken == 0) {
        printf("IMU: calibration failed — no samples read. Bias left at zero.\n");
        return;
    }

    gyro_bias_x = static_cast<float>(sx / taken);
    gyro_bias_y = static_cast<float>(sy / taken);
    gyro_bias_z = static_cast<float>(sz / taken);

    printf("IMU: gyro bias (deg/s) gx=%+.4f gy=%+.4f gz=%+.4f  [n=%d]\n",
           gyro_bias_x, gyro_bias_y, gyro_bias_z, taken);
}

// ---------- Magnetometer ----------

static MagChip mag_probe() {
    sleep_ms(10);
    if (i2c_probe(HMC_ADDR)) return MagChip::HMC5883L;
    sleep_ms(10);
    if (i2c_probe(QMC_ADDR)) return MagChip::QMC5883L;
    return MagChip::NONE;
}

static bool mag_init() {
    if (g_mag_chip == MagChip::HMC5883L) {
        // 8 samples averaged, 75 Hz ODR, normal measurement
        if (!i2c_write_reg(HMC_ADDR, HMC_REG_CFGA, 0x78)) return false;
        // Gain: ±1.3 Ga range, 1090 LSB/Ga
        if (!i2c_write_reg(HMC_ADDR, HMC_REG_CFGB, 0x20)) return false;
        // Continuous measurement mode
        if (!i2c_write_reg(HMC_ADDR, HMC_REG_MODE, 0x00)) return false;
        return true;
    }
    if (g_mag_chip == MagChip::QMC5883L) {
        // Required: set SET/RESET period register before configuring
        if (!i2c_write_reg(QMC_ADDR, QMC_REG_SR,    0x01)) return false;
        // Continuous mode, 200 Hz ODR, 8 G range, 512 OSR
        if (!i2c_write_reg(QMC_ADDR, QMC_REG_CTRL1, 0x1D)) return false;
        if (!i2c_write_reg(QMC_ADDR, QMC_REG_CTRL2, 0x00)) return false;
        return true;
    }
    return false;
}

static bool mag_read_raw(float &x, float &y, float &z) {
    uint8_t raw[6];
    if (g_mag_chip == MagChip::HMC5883L) {
        if (!i2c_read_burst(HMC_ADDR, HMC_REG_DATA, raw, 6)) return false;
        // HMC register order: X Z Y (big-endian)
        x = static_cast<float>(be16(&raw[0]));
        z = static_cast<float>(be16(&raw[2]));
        y = static_cast<float>(be16(&raw[4]));
        return true;
    }
    if (g_mag_chip == MagChip::QMC5883L) {
        if (!i2c_read_burst(QMC_ADDR, QMC_REG_DATA, raw, 6)) return false;
        // QMC register order: X Y Z (little-endian, two's complement)
        x = static_cast<float>(le16(&raw[0]));
        y = static_cast<float>(le16(&raw[2]));
        z = static_cast<float>(le16(&raw[4]));
        return true;
    }
    return false;
}

static void calibrate_mag() {
    printf("MAG: calibrating hard-iron — keep sensor stationary (%d samples)...\n",
           MAG_CAL_SAMPLES);
    double sx = 0.0, sy = 0.0, sz = 0.0;
    int taken = 0;
    for (int i = 0; i < MAG_CAL_SAMPLES; ++i) {
        float x, y, z;
        if (mag_read_raw(x, y, z)) {
            sx += x; sy += y; sz += z;
            ++taken;
        }
        sleep_ms(10);
    }
    if (taken == 0) {
        printf("MAG: calibration failed — no samples read. Bias left at zero.\n");
        return;
    }
    mag_bias_x = static_cast<float>(sx / taken);
    mag_bias_y = static_cast<float>(sy / taken);
    mag_bias_z = static_cast<float>(sz / taken);
    printf("MAG: hard-iron bias x=%+.1f y=%+.1f z=%+.1f [n=%d]\n",
           mag_bias_x, mag_bias_y, mag_bias_z, taken);
}

static void read_and_print_mag() {
    float x, y, z;
    if (!mag_read_raw(x, y, z)) {
        printf("MAG: read failed\n");
        return;
    }
    printf("MAG:%.2f,%.2f,%.2f\n",
           x - mag_bias_x, y - mag_bias_y, z - mag_bias_z);
}

// ---------- GPS ----------

static void uart_drain() {
    while (uart_is_readable(GPS_UART)) {
        (void)uart_getc(GPS_UART);
    }
}

static uint detect_baud() {
    for (uint baud : BAUD_CANDIDATES) {
        printf("Trying %u baud...\n", baud);

        uart_set_baudrate(GPS_UART, baud);
        sleep_ms(150);
        uart_drain();

        uint8_t sample[10];
        int n = 0;
        while (n < 10) {
            if (!uart_is_readable_within_us(GPS_UART, 250000)) break;
            sample[n++] = uart_getc(GPS_UART);
        }

        bool found = false;
        for (int j = 0; j < n; ++j) {
            if (sample[j] == '$') { found = true; break; }
        }

        printf("  read %d bytes:", n);
        for (int j = 0; j < n; ++j) printf(" %02X", sample[j]);
        printf("%s\n", found ? "  <-- '$' found" : "");

        if (found) return baud;
    }
    return 0;
}

static void poll_gps() {
    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);

        if (c == '\r') continue;

        if (c == '\n') {
            nmea_buf[nmea_len] = '\0';
            if (nmea_len > 0) {
                printf("GPS: %s\n", nmea_buf);
            }
            nmea_len = 0;
            continue;
        }

        if (nmea_len < NMEA_BUF_SIZE - 1) {
            nmea_buf[nmea_len++] = c;
        } else {
            nmea_len = 0;
        }
    }
}

// ---------- main ----------

int main() {
    stdio_init_all();

    i2c_init(i2c0, I2C_BAUDRATE);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    uart_init(GPS_UART, BAUD_CANDIDATES[0]);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);

    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    if (!mpu_write_reg(REG_PWR_MGMT_1, 0x00)) {
        printf("IMU: MPU-6050 wake failed (no ACK at 0x68)\n");
    } else {
        printf("IMU: MPU-6050 awake.\n");
    }
    sleep_ms(100);

    calibrate_gyro();

    // Mag probe and init
    g_mag_chip = mag_probe();
    if (g_mag_chip == MagChip::HMC5883L) {
        printf("MAG: HMC5883L found at 0x1E\n");
    } else if (g_mag_chip == MagChip::QMC5883L) {
        printf("MAG: QMC5883L found at 0x0D\n");
    } else {
        printf("MAG: no chip found at 0x1E or 0x0D — heading unavailable\n");
    }

    if (g_mag_chip != MagChip::NONE) {
        if (!mag_init()) {
            printf("MAG: init failed\n");
            g_mag_chip = MagChip::NONE;
        } else {
            sleep_ms(50); // let first measurement settle
            calibrate_mag();
        }
    }

    printf("GPS: baud auto-detect on UART0 (TX=GP%u, RX=GP%u)\n",
           UART_TX_PIN, UART_RX_PIN);

    uint detected = 0;
    while (detected == 0) {
        detected = detect_baud();
        if (detected == 0) {
            printf("GPS: no NMEA '$' at any candidate rate. Retrying...\n");
            sleep_ms(500);
        }
    }

    uart_set_baudrate(GPS_UART, detected);
    uart_drain();
    nmea_len = 0;
    printf("GPS: locked baud %u. Streaming.\n", detected);

    absolute_time_t next_imu = make_timeout_time_ms(10);
    absolute_time_t next_mag = make_timeout_time_ms(20);
    while (true) {
        poll_gps();

        if (absolute_time_diff_us(get_absolute_time(), next_imu) <= 0) {
            read_and_print_imu();
            next_imu = delayed_by_ms(next_imu, 10);
        }

        if (g_mag_chip != MagChip::NONE &&
            absolute_time_diff_us(get_absolute_time(), next_mag) <= 0) {
            read_and_print_mag();
            next_mag = delayed_by_ms(next_mag, 20);
        }

        tight_loop_contents();
    }
}
