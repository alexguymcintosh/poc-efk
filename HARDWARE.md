# Hardware Wiring — EKF Localisation Session

Pico 2 (RP2350). I2C0 bus shared by both sensors on GP4/GP5. No address conflicts.

---

## Raspberry Pi Pico 2 — Pin Reference (used pins only)

```
                     Pico 2 Top View
                   ┌────────────────┐
               GP0 │ 1          40 │ VBUS (5V USB)
               GP1 │ 2          39 │ VSYS
               GND │ 3          38 │ GND  ←── IMU GND, Mag GND
               GP2 │ 4          37 │ 3V3_EN
               GP3 │ 5          36 │ 3V3  ←── IMU VCC, Mag VCC
  I2C0 SDA    GP4  │ 6          35 │ ADC_VREF
  I2C0 SCL    GP5  │ 7          34 │ GP28
               GND │ 8          33 │ GND
                   ...
               GP0 │ 1          2  │ GP1   (GPS TX/RX — UART0)
                   └────────────────┘
```

---

## Sensor 1 — GY-521 (MPU-6050) — Gyroscope + Accelerometer

**Protocol:** I2C0 — Address `0x68` (AD0 tied to GND)

| GY-521 Pin | Pico 2 Pin | GPIO       |
|------------|------------|------------|
| VCC        | Pin 36     | 3V3 OUT    |
| GND        | Pin 38     | GND        |
| SDA        | Pin 6      | GP4 (I2C0 SDA) |
| SCL        | Pin 7      | GP5 (I2C0 SCL) |
| AD0        | Pin 23     | GND        |

**Unused:** XDA, XCL (auxiliary I2C), INT (interrupt)

---

## Sensor 2 — 5883-series Magnetometer — Absolute Heading

**Protocol:** I2C0 — Address probed on boot: `0x1E` (HMC5883L) or `0x0D` (QMC5883L)
**Shares I2C bus with MPU-6050 — no conflict**

| Mag Pin | Pico 2 Pin | GPIO       |
|---------|------------|------------|
| VCC     | Pin 36     | 3V3 OUT    |
| GND     | Pin 38     | GND        |
| SDA     | Pin 6      | GP4 (I2C0 SDA) |
| SCL     | Pin 7      | GP5 (I2C0 SCL) |

**Chip identity unknown from marking — firmware probes both addresses on boot and reports which responded.**

---

## Sensor 3 — GY-GPSV3 (NEO-M9N) — GPS Position

**Protocol:** UART0 — Baud auto-detected on boot (57600 confirmed last session)

| GPS Pin | Pico 2 Pin | GPIO       |
|---------|------------|------------|
| VCC     | 3.3V or 5V | —          |
| GND     | GND        | —          |
| TX      | Pin 2      | GP1 (UART0 RX) |
| RX      | Pin 1      | GP0 (UART0 TX) |

---

## I2C Bus Summary

```
GP4/GP5 (I2C0)
    ├── 0x68  MPU-6050  (GY-521)
    ├── 0x1E  HMC5883L  (if this chip)
    └── 0x0D  QMC5883L  (if this chip)
```

All three addresses are distinct — no conflicts.

---

## What Each Sensor Does for the EKF

| Sensor     | Data             | EKF role                                  |
|------------|------------------|-------------------------------------------|
| MPU-6050   | Gyro + Accel     | Angular velocity (rotation rate). Accel re-enabled once heading known. |
| Mag chip   | X/Y/Z field      | Absolute heading (yaw) — prevents position drift |
| NEO-M9N    | Lat/Lon          | Position corrections at 1Hz              |
