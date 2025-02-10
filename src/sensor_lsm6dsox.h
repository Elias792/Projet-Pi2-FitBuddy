// sensor_lsm6dsox.h
#ifndef SENSOR_LSM6DSOX_H
#define SENSOR_LSM6DSOX_H

#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct imu_data {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};

int sensor_lsm6dsox_init(void);
int sensor_lsm6dsox_read(struct imu_data *out);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_LSM6DSOX_H
