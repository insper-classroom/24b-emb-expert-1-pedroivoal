#include <mpu6050.h>

void mpu6050_set_config(imu_c *config, i2c_inst_t *i2c, int pin_sda, int pin_scl, int acc_scale)
{
    config->i2c = i2c;
    config->pin_sda = pin_sda;
    config->pin_scl = pin_scl;
    config->acc_scale = acc_scale;
}

int mpu6050_init(imu_c config)
{
    i2c_init(config.i2c, 400 * 1000);
    gpio_set_function(config.pin_sda, GPIO_FUNC_I2C);
    gpio_set_function(config.pin_scl, GPIO_FUNC_I2C);
    gpio_pull_up(config.pin_sda);
    gpio_pull_up(config.pin_scl);

    return 1;
}

int mpu6050_reset(imu_c config)
{
    uint8_t buf[] = {MPUREG_PWR_MGMT_1, 0x00};
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, buf, 2, false);

    return 1;
}

int mpu6050_read_acc(imu_c config, int16_t accel[3])
{
    uint8_t buffer[6];

    uint8_t val = MPUREG_ACCEL_XOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &val, 1, true);
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    return 1;
}

int mpu6050_read_gyro(imu_c config, int16_t gyro[3])
{
    uint8_t buffer[6];

    uint8_t val = MPUREG_GYRO_XOUT_H;
    i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &val, 1, true);
    i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 6, false);

    for (int i = 0; i < 3; i++)
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
    return 1;
}

int mpu6050_read_temp(imu_c config, int16_t *temp)
{
    uint8_t buffer[2];

    uint8_t temp_reg_addr = MPUREG_TEMP_OUT_H;
    
    // Escreve no registrador de temperatura para iniciar a leitura
    int ret = i2c_write_blocking(config.i2c, MPU6050_I2C_DEFAULT, &temp_reg_addr, 1, true);
    if (ret != 1) {
        return -1;  // erro na escrita
    }

    // Lê os 2 bytes da temperatura
    ret = i2c_read_blocking(config.i2c, MPU6050_I2C_DEFAULT, buffer, 2, false);
    if (ret != 2) {
        return -2;  // erro na leitura
    }

    // Combina os 2 bytes de temperatura e armazena no parâmetro temp
    *temp = (buffer[0] << 8 | buffer[1]);

    // Opcional: Convertendo para graus Celsius (se necessário)
    // *temp = (*temp / 340.0) + 36.53;

    return 1;  // Sucesso
}
