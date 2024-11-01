#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include <mpu6050.h>
#include <Fusion.h>

#define SAMPLE_PERIOD (0.01f)

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 4;
const int I2C_SCL_GPIO = 5;

typedef struct
{
    int axis;
    int val;
} adc_t;

void mpu6050_task(void *p)
{
    imu_c imu_config;
    mpu6050_set_config(&imu_config, i2c1, I2C_SDA_GPIO, I2C_SCL_GPIO, 2);
    mpu6050_init(imu_config);
    mpu6050_reset(imu_config);

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t acceleration[3], gyro[3], temp[1];
    while(1)
    {
        mpu6050_read_acc(imu_config, acceleration);
        mpu6050_read_gyro(imu_config, gyro);
        mpu6050_read_temp(imu_config, temp);

        FusionVector gyroscope =
        {
            .axis.x = gyro[0] / 131.0f, // Conversão para graus/s
            .axis.y = gyro[1] / 131.0f,
            .axis.z = gyro[2] / 131.0f,
        };

        FusionVector accelerometer =
        {
            .axis.x = acceleration[0] / 16384.0f, // Conversão para g
            .axis.y = acceleration[1] / 16384.0f,
            .axis.z = acceleration[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        printf("Aceleração: X: %0.2f g, Y: %0.2f g, Z: %0.2f g\n", accelerometer.axis.x, accelerometer.axis.y, accelerometer.axis.z);
        printf("Giroscópio: X: %0.2f °/s, Y: %0.2f °/s, Z: %0.2f °/s\n", gyroscope.axis.x, gyroscope.axis.y, gyroscope.axis.z);
        printf("Temperatura: %0.2f °C\n", temp[0]);
        printf("Euler: Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main()
{
    stdio_init_all();

    xTaskCreate(mpu6050_task, "mpu6050_Task 1", 8192, NULL, 1, NULL);

    vTaskStartScheduler();
    while (1);
}
