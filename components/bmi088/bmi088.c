/**
 * @file bmi088.c
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-05-30 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"

#include "math_tools.h"

#include "bmi088_interface.h"
#include "bmi08x.h"
#include "main.h"
#include <math.h>

/* Private macros ------------------------------------------------------------*/
#define GRAVITY_EARTH (9.80665f)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/
static int8_t Bmi088_EnableInterrupt(Bmi088_t *self);
static void Bmi088_CalibrateGyroBiasZ(Bmi088_t *self, volatile float gyro_z);
/*!
 *  @brief This internal function converts lsb to meter per second squared for 16 bit accelerometer
 * for range 2G, 4G, 8G or 16G.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] g_range   : Gravity range.
 *  @param[in] bit_width : Resolution for accel.
 *
 *  @return Accel values in meter per second square.
 */
/**
 * @brief 这个内部函数将lsb转换为米每二次方秒,适用于16位加速度计,范围为2G,4G,8G或16G
 *
 * @param[in] val       : 每个轴的LSB
 * @param[in] g_range   : 重力范围
 * @param[in] bit_width : 加速度计的分辨率
 *
 * @return m/s2
 */
static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return deg/s
 */
/**
 * @brief 这个函数将lsb转换为度每秒,适用于16位陀螺仪,范围为125,250,500,1000或2000dps
 *
 * @param[in] val       : 每个轴的LSB
 * @param[in] dps       : 度每秒
 * @param[in] bit_width : 陀螺仪的分辨率
 *
 * @return deg/s
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief Bmi088初始化
 *
 * @param
 */
int8_t Bmi088_Init(Bmi088_t *self)
{
    int8_t rslt;
    rslt = bmi08_interface_init(&self->bmi08dev, BMI08_SPI_INTF, BMI088_VARIANT);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08xa_init(&self->bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_soft_reset(&self->bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&self->bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_soft_reset(&self->bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_load_config_file(&self->bmi08dev);
    }

    if (rslt == BMI08_OK)
    {
        self->bmi08dev.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;

        if (self->bmi08dev.variant == BMI085_VARIANT)
        {
            self->bmi08dev.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
        }
        else if (self->bmi08dev.variant == BMI088_VARIANT)
        {
            self->bmi08dev.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
        }

        self->bmi08dev.accel_cfg.power =
            BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        self->bmi08dev.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&self->bmi08dev);

        rslt = bmi08xa_set_meas_conf(&self->bmi08dev);

        self->bmi08dev.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
        self->bmi08dev.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
        self->bmi08dev.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
        self->bmi08dev.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&self->bmi08dev);

        rslt = bmi08g_set_meas_conf(&self->bmi08dev);
    }

    /* Enable data ready interrupts */
    if (rslt == BMI08_OK)
    {
        rslt = Bmi088_EnableInterrupt(self);
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable bmi08 interrupt
 *
 *  @param[in] void
 *  @return void
 */
static int8_t Bmi088_EnableInterrupt(Bmi088_t *self)
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    self->accel_int_config.int_channel = BMI08_INT_CHANNEL_1;
    self->accel_int_config.int_type = BMI08_ACCEL_INT_DATA_RDY;
    self->accel_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    self->accel_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
    self->accel_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&self->accel_int_config,
                                 &self->bmi08dev);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        self->gyro_int_config.int_channel = BMI08_INT_CHANNEL_3;
        self->gyro_int_config.int_type = BMI08_GYRO_INT_DATA_RDY;
        self->gyro_int_config.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        self->gyro_int_config.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
        self->gyro_int_config.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&self->gyro_int_config,
                                     &self->bmi08dev);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &self->bmi08dev);
    }

    return rslt;
}

/**
 * @brief 外部调用的中断回调函数
 *
 * @param
 */
void Bmi088_ExtiReadCallback(Bmi088_t *self, uint16_t gpio_pin)
{
    static int32_t sensor_temp = 0;
    if (gpio_pin == INT1_ACCEL_Pin)
    {
        bmi08a_get_data(&self->raw_accel, &self->bmi08dev);

        if (self->bmi08dev.variant == BMI085_VARIANT)
        {
            /* Converting lsb to meter per second squared for 16 bit accelerometer at 4G range. */
            self->rx_data.accel[0] = lsb_to_mps2(self->raw_accel.x, 4, 16);
            self->rx_data.accel[1] = lsb_to_mps2(self->raw_accel.y, 4, 16);
            self->rx_data.accel[2] = lsb_to_mps2(self->raw_accel.z, 4, 16);
        }
        else if (self->bmi08dev.variant == BMI088_VARIANT)
        {
            /* Converting lsb to meter per second squared for 16 bit accelerometer at 6G range. */
            self->rx_data.accel[0] = lsb_to_mps2(self->raw_accel.x, 6, 16);
            self->rx_data.accel[1] = lsb_to_mps2(self->raw_accel.y, 6, 16);
            self->rx_data.accel[2] = lsb_to_mps2(self->raw_accel.z, 6, 16);
        }

        bmi08a_get_sensor_temperature(&self->bmi08dev, &sensor_temp);
        self->rx_data.temp = (float)sensor_temp / 1000.0f;

    }
    else if (gpio_pin == INT1_GYRO_Pin)
    {
        bmi08g_get_data(&self->raw_gyro, &self->bmi08dev);

        /* Converting lsb to rad per second for 16 bit gyro at 2000 dps range. */
        self->rx_data.gyro[0] = lsb_to_dps(self->raw_gyro.x, (float)2000, 16) * DEG_TO_RAD;
        self->rx_data.gyro[1] = lsb_to_dps(self->raw_gyro.y, (float)2000, 16) * DEG_TO_RAD;
        self->rx_data.gyro[2] = lsb_to_dps(self->raw_gyro.z, (float)2000, 16) * DEG_TO_RAD;

        Bmi088_CalibrateGyroBiasZ(self, self->rx_data.gyro[2]);
    }
}

static void Bmi088_CalibrateGyroBiasZ(Bmi088_t *self, volatile float gyro_z)
{
    static float sum = 0.0f;
    static uint16_t cnt = 0;

    if (cnt < 2000)
    {
        cnt++;
    }
    else if (cnt < 22000)
    {
        sum += gyro_z;
        cnt++;
    }
    else if (cnt == 22000)
    {
        self->gyro_bias_z = sum / 20000.0f;
        cnt++;
    }
    else
    {
    }
}

static float lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
