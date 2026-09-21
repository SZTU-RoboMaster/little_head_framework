/**
 * @file bmi270_driver.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-09-18 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi270_driver.h"

#include "math_tools.h"

#include "bmi270.h"
#include "bmi270_interface.h"
#include "main.h"

/* Private macros ------------------------------------------------------------*/
#define GRAVITY_EARTH (9.80665f)

#define ACCEL UINT8_C(0x00)
#define GYRO UINT8_C(0x01)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] bmi       : Structure instance of bmi2_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

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
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

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

/*!
 *  @brief This internal API is used to convert raw temperature data to temperature value in degrees
 * Celsius.
 *
 *  @param[in] temperature_data : Raw temperature data obtained from the sensor.
 *
 *  @return Temperature value in degrees Celsius.
 */
static float lsb_temp(int16_t temperature_data);

/* function prototypes -------------------------------------------------------*/

/**
 * @brief Bmi270初始化
 *
 * @param
 */
int8_t Bmi270::init(void)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Assign accel and gyro sensor to variable. */
    uint8_t sensor_list[2] = {BMI2_ACCEL, BMI2_GYRO};

    struct bmi2_sens_config config;

    rslt = bmi2_interface_init(&bmi270dev_, BMI2_SPI_INTF);

    if (rslt == BMI2_OK)
    {
        rslt = bmi270_init(&bmi270dev_);
    }

    // 关闭低功耗模式, 否则每次执行 bmi2_get_regs() 都会忙等 450µs
    if (rslt == BMI2_OK)
    {
        rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &bmi270dev_);
    }

    if (rslt == BMI2_OK)
    {
        /* Accel and gyro configuration settings. */
        rslt = set_accel_gyro_config(&bmi270dev_);

        if (rslt == BMI2_OK)
        {
            /* NOTE:
             * Accel and Gyro enable must be done after setting configurations
             */
            rslt = bmi2_sensor_enable(sensor_list, 2, &bmi270dev_);

            if (rslt == BMI2_OK)
            {
                config.type = BMI2_ACCEL;

                /* Get the accel configurations. */
                rslt = bmi2_get_sensor_config(&config, 1, &bmi270dev_);
            }
        }
        /* Enable data ready interrupts */
        if (rslt == BMI2_OK)
        {
            rslt = enable_bmi2_interrupt();
        }
    }
    return rslt;
}

/**
 * @brief 外部调用的中断回调函数
 *
 * @param[in] gpio_pin
 */
uint8_t Bmi270::exti_read_callback(uint16_t gpio_pin)
{
    int8_t rslt;
    int16_t temperature_data;

    rslt = bmi2_get_temperature_data(&temperature_data, &bmi270dev_);

    rx_data_.temp = lsb_temp(temperature_data);

    rslt = bmi2_get_sensor_data(&sensor_data_, &bmi270dev_);

    if ((rslt == BMI2_OK) && (sensor_data_.status & BMI2_DRDY_ACC) &&
        (sensor_data_.status & BMI2_DRDY_GYR))
    {
        /* Converting lsb to meter per second squared for 16 bit accelerometer at 8G range. */
        rx_data_.accel[0] = lsb_to_mps2(sensor_data_.acc.x, (float)8, 16);
        rx_data_.accel[1] = lsb_to_mps2(sensor_data_.acc.y, (float)8, 16);
        rx_data_.accel[2] = lsb_to_mps2(sensor_data_.acc.z, (float)8, 16);

        /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
        rx_data_.gyro[0] =
            lsb_to_dps(sensor_data_.gyr.x, (float)2000, bmi270dev_.resolution) * DEG_TO_RAD;
        rx_data_.gyro[1] =
            lsb_to_dps(sensor_data_.gyr.y, (float)2000, bmi270dev_.resolution) * DEG_TO_RAD;
        rx_data_.gyro[2] =
            lsb_to_dps(sensor_data_.gyr.z, (float)2000, bmi270dev_.resolution) * DEG_TO_RAD;

        calibrate_gyro_bias_z(rx_data_.gyro[2]);

        return true;
    }
    return false;
}

/*!
 *  @brief This API is used to enable bmi2 interrupt
 *
 *  @param[in] void
 *  @return void
 */
int8_t Bmi270::enable_bmi2_interrupt()
{
    int8_t rslt;
    bmi2_int_pin_config int_config;
    /* Set interrupt pin configuration */
    int_config.pin_type = BMI2_INT1;
    int_config.int_latch = BMI2_INT_NON_LATCH;
    int_config.pin_cfg->input_en = BMI2_INT_INPUT_DISABLE;
    int_config.pin_cfg->output_en = BMI2_INT_OUTPUT_ENABLE;
    int_config.pin_cfg->lvl = BMI2_INT_ACTIVE_LOW;
    int_config.pin_cfg->od = BMI2_INT_PUSH_PULL;

    /* Enable data ready interrupt channel */
    rslt = bmi2_set_int_pin_config((const struct bmi2_int_pin_config *)&int_config, &bmi270dev_);

    return rslt;
}

void Bmi270::calibrate_gyro_bias_z(volatile float gyro_z)
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
        gyro_bias_z_ = sum / 20000.0f;
        cnt++;
    }
    else
    {
    }
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi2_get_sensor_config(config, 2, bmi);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their
         * requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_800HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_8G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are
         * averaged if it is set to 2, then 2^(bandwidth parameter) samples are averaged, resulting
         * in 4 averaged samples. Note1 : For more information, refer the datasheet. Note2 : A
         * higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_OSR2_AVG2;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their
         * requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_800HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_OSR2_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi2_set_sensor_config(config, 2, bmi);
    }

    return rslt;
}

static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
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

static float lsb_temp(int16_t temperature_data)
{
    /* Perform temperature conversion */
    float temperature_value = (float)((((float)((int16_t)temperature_data)) / 512.0) + 23.0);

    return temperature_value;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
