/**
 * @file template.cpp
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
#include "main.h"
#include "math_tools.h"

/* Private macros ------------------------------------------------------------*/

#define GRAVITY_EARTH (9.80665f)

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* function prototypes -------------------------------------------------------*/

/**
 * @brief Bmi088初始化
 *
 * @param
 */
int8_t Bmi088::init(void)
{
    int8_t rslt;
    rslt = bmi08_interface_init(&bmi08dev_, BMI08_SPI_INTF, BMI088_VARIANT);

    if (rslt == BMI08_OK)
    {
        rslt = bmi08xa_init(&bmi08dev_);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_soft_reset(&bmi08dev_);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_init(&bmi08dev_);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08g_soft_reset(&bmi08dev_);
    }

    if (rslt == BMI08_OK)
    {
        rslt = bmi08a_load_config_file(&bmi08dev_);
    }

    if (rslt == BMI08_OK)
    {
        bmi08dev_.accel_cfg.odr = BMI08_ACCEL_ODR_1600_HZ;

        if (bmi08dev_.variant == BMI085_VARIANT)
        {
            bmi08dev_.accel_cfg.range = BMI085_ACCEL_RANGE_4G;
        }
        else if (bmi08dev_.variant == BMI088_VARIANT)
        {
            bmi08dev_.accel_cfg.range = BMI088_ACCEL_RANGE_6G;
        }

        bmi08dev_.accel_cfg.power =
            BMI08_ACCEL_PM_ACTIVE; /*user_accel_power_modes[user_bmi088_accel_low_power]; */
        bmi08dev_.accel_cfg.bw = BMI08_ACCEL_BW_NORMAL; /* Bandwidth and OSR are same */

        rslt = bmi08a_set_power_mode(&bmi08dev_);

        rslt = bmi08xa_set_meas_conf(&bmi08dev_);

        bmi08dev_.gyro_cfg.odr = BMI08_GYRO_BW_116_ODR_1000_HZ;
        bmi08dev_.gyro_cfg.range = BMI08_GYRO_RANGE_2000_DPS;
        bmi08dev_.gyro_cfg.bw = BMI08_GYRO_BW_116_ODR_1000_HZ;
        bmi08dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;

        rslt = bmi08g_set_power_mode(&bmi08dev_);

        rslt = bmi08g_set_meas_conf(&bmi08dev_);
    }

    /* Enable data ready interrupts */
    if (rslt == BMI08_OK)
    {
        rslt = enable_bmi08_interrupt();
    }

    return rslt;
}

/*!
 *  @brief This API is used to enable bmi08 interrupt
 *
 *  @param[in] void
 *  @return void
 */
int8_t Bmi088::enable_bmi08_interrupt()
{
    int8_t rslt;
    uint8_t data = 0;

    /* Set accel interrupt pin configuration */
    accel_int_config_.int_channel = BMI08_INT_CHANNEL_1;
    accel_int_config_.int_type = BMI08_ACCEL_INT_DATA_RDY;
    accel_int_config_.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
    accel_int_config_.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
    accel_int_config_.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

    /* Enable accel data ready interrupt channel */
    rslt = bmi08a_set_int_config((const struct bmi08_accel_int_channel_cfg *)&accel_int_config_,
                                 &bmi08dev_);

    if (rslt == BMI08_OK)
    {
        /* Set gyro interrupt pin configuration */
        gyro_int_config_.int_channel = BMI08_INT_CHANNEL_3;
        gyro_int_config_.int_type = BMI08_GYRO_INT_DATA_RDY;
        gyro_int_config_.int_pin_cfg.output_mode = BMI08_INT_MODE_PUSH_PULL;
        gyro_int_config_.int_pin_cfg.lvl = BMI08_INT_ACTIVE_LOW;
        gyro_int_config_.int_pin_cfg.enable_int_pin = BMI08_ENABLE;

        /* Enable gyro data ready interrupt channel */
        rslt = bmi08g_set_int_config((const struct bmi08_gyro_int_channel_cfg *)&gyro_int_config_,
                                     &bmi08dev_);

        rslt = bmi08g_get_regs(BMI08_REG_GYRO_INT3_INT4_IO_MAP, &data, 1, &bmi08dev_);
    }

    return rslt;
}

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
float Bmi088::lsb_to_mps2(int16_t val, int8_t g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

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
float Bmi088::lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

/**
 * @brief 外部调用的中断回调函数
 *
 * @param
 */
void Bmi088::exti_read_callback(uint16_t gpio_pin)
{
    static uint8_t mod160 = 160;
    static int32_t sensor_temp = 0;
    if (gpio_pin == INT1_ACCEL_Pin)
    {
        bmi08a_get_data(&raw_accel_, &bmi08dev_);

        if (bmi08dev_.variant == BMI085_VARIANT)
        {
            /* Converting lsb to meter per second squared for 16 bit accelerometer at 4G range. */
            rx_data_.accel[0] = lsb_to_mps2(raw_accel_.x, 4, 16);
            rx_data_.accel[1] = lsb_to_mps2(raw_accel_.y, 4, 16);
            rx_data_.accel[2] = lsb_to_mps2(raw_accel_.z, 4, 16);
        }
        else if (bmi08dev_.variant == BMI088_VARIANT)
        {
            /* Converting lsb to meter per second squared for 16 bit accelerometer at 6G range. */
            rx_data_.accel[0] = lsb_to_mps2(raw_accel_.x, 6, 16);
            rx_data_.accel[1] = lsb_to_mps2(raw_accel_.y, 6, 16);
            rx_data_.accel[2] = lsb_to_mps2(raw_accel_.z, 6, 16);
        }

        if (mod160++ >= 160)
        {
            mod160 = 0;
            bmi08a_get_sensor_temperature(&bmi08dev_, &sensor_temp);
            rx_data_.temp = (float)sensor_temp / 1000.0f;
        }
    }
    else if (gpio_pin == INT1_GYRO_Pin)
    {
        bmi08g_get_data(&raw_gyro_, &bmi08dev_);

        /* Converting lsb to rad per second for 16 bit gyro at 2000 dps range. */
        rx_data_.gyro[0] = lsb_to_dps(raw_gyro_.x, (float)2000, 16) * DEG_TO_RAD;
        rx_data_.gyro[1] = lsb_to_dps(raw_gyro_.y, (float)2000, 16) * DEG_TO_RAD;
        rx_data_.gyro[2] = lsb_to_dps(raw_gyro_.z, (float)2000, 16) * DEG_TO_RAD;

        calibrate_gyro_bias_z(rx_data_.gyro[2]);
    }
}

void Bmi088::calibrate_gyro_bias_z(volatile float gyro_z)
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
/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
