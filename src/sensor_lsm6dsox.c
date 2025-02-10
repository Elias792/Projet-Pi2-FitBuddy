// sensor_lsm6dsox.c
#include "sensor_lsm6dsox.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/i2c.h>
#include <logging/log.h>

LOG_MODULE_DECLARE(main, LOG_LEVEL_INF);

// Exemple: Nom du bus I2C défini dans le device tree (à adapter)
#define I2C_BUS DT_LABEL(DT_NODELABEL(i2c1))
// Adresse I2C (à vérifier selon config du capteur)
#define LSM6DSO_ADDR 0x6A  

static const struct device *i2c_dev;

// Exemple d’initialisation
int sensor_lsm6dsox_init(void)
{
    i2c_dev = device_get_binding(I2C_BUS);
    if (!i2c_dev) {
        LOG_ERR("I2C bus not found");
        return -ENODEV;
    }

    // Exemple d’écriture dans un registre pour activer l’IMU (ODR, range, etc.)
    // Registres LSM6DSOX à adapter selon la datasheet
    uint8_t config[2];
    int ret;

    // Exemple: CTRL1_XL = 0x10 -> 0x60 = ODR 416 Hz, ±2g
    config[0] = 0x10; 
    config[1] = 0x60; 
    ret = i2c_write(i2c_dev, config, 2, LSM6DSO_ADDR);
    if (ret) {
        LOG_ERR("Failed to configure ACC");
        return ret;
    }

    // Exemple: CTRL2_G = 0x11 -> 0x60 = ODR 416 Hz, 2000 dps
    config[0] = 0x11; 
    config[1] = 0x60; 
    ret = i2c_write(i2c_dev, config, 2, LSM6DSO_ADDR);
    if (ret) {
        LOG_ERR("Failed to configure GYRO");
        return ret;
    }

    LOG_INF("LSM6DSOX initialized");
    return 0;
}

// Lecture basique (accel + gyro)
int sensor_lsm6dsox_read(struct imu_data *out)
{
    uint8_t data[12];
    int ret;

    // Les registres OUTX_L_G (0x22) à OUTZ_H_XL (0x2D) -> 12 octets
    // 2 octets par axe gyro X/Y/Z, puis 2 octets par axe accel X/Y/Z
    // Dans un ordre à vérifier selon la doc du LSM6DSO(X).
    const uint8_t start_reg = 0x22;
    ret = i2c_write_read(i2c_dev, LSM6DSO_ADDR, &start_reg, 1, data, 12);
    if (ret) {
        LOG_ERR("Error reading LSM6DSOX data");
        return ret;
    }

    // Conversion brute -> float ; coefficients à adapter
    // Gyro dps/LSB et Acc mg/LSB (exemples approximatifs)
    int16_t gx_raw = (int16_t)((data[1] << 8) | data[0]);
    int16_t gy_raw = (int16_t)((data[3] << 8) | data[2]);
    int16_t gz_raw = (int16_t)((data[5] << 8) | data[4]);

    int16_t ax_raw = (int16_t)((data[7] << 8) | data[6]);
    int16_t ay_raw = (int16_t)((data[9] << 8) | data[8]);
    int16_t az_raw = (int16_t)((data[11] << 8) | data[10]);

    // Facteurs de conversion approximatifs (à ajuster en fonction de la config)
    const float gyro_sensitivity = 70.0f / 1000.0f;   // ex. 70 mdps/LSB -> 0.07 dps/LSB
    const float accel_sensitivity = 0.061f;          // ex. 0.061 mg/LSB -> 0.000061 g/LSB

    out->gyro_x  = gx_raw * gyro_sensitivity;
    out->gyro_y  = gy_raw * gyro_sensitivity;
    out->gyro_z  = gz_raw * gyro_sensitivity;

    out->accel_x = ax_raw * accel_sensitivity;
    out->accel_y = ay_raw * accel_sensitivity;
    out->accel_z = az_raw * accel_sensitivity;

    return 0;
}
