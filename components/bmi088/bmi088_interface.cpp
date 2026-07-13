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

#include "bmi088_interface.h"
#include "bsp_dwt.h"
#include "main.h"
#include "cmsis_os.h"

/* Private macros ------------------------------------------------------------*/

#define BMI088_SPI SPI1
#define BMI088_SPI_ACCEL 0
#define BMI088_SPI_GYRO 1

#define BMI08_READ_WRITE_LEN UINT8_C(64)

#define BMI08_TIMEOUT_CNT 1680000

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/* Private function declarations ---------------------------------------------*/

static inline void bmi088_accel_select(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

static inline void bmi088_accel_unselect(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

static inline void bmi088_gyro_select(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

static inline void bmi088_gyro_unselect(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

/* function prototypes -------------------------------------------------------*/

uint8_t spi_rw_byte(uint8_t byte)
{
    uint32_t timeout_cnt = 0;
    SET_BIT(BMI088_SPI->CR1, SPI_CR1_SPE);
    while ((BMI088_SPI->SR & SPI_SR_TXE) == RESET)
    {
        if (timeout_cnt < BMI08_TIMEOUT_CNT)
        {
            timeout_cnt++;
        }
        else
        {
            return 0;
        }
    }
    BMI088_SPI->DR = byte;
    timeout_cnt = 0;
    while ((BMI088_SPI->SR & SPI_SR_RXNE) == RESET)
    {
        if (timeout_cnt < BMI08_TIMEOUT_CNT)
        {
            timeout_cnt++;
        }
        else
        {
            return 0;
        }
    }
    return BMI088_SPI->DR;
}

/*!
 * Delay function for stm32(systick)
 */
void bmi08_delay_us(uint32_t period, void *intf_ptr)
{
    delay_us(period);
}

/*!
 * SPI read function for stm32
 */
BMI08_INTF_RET_TYPE bmi08_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                                   void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    if (dev_addr == acc_dev_add)
    {
        bmi088_accel_select();
        spi_rw_byte(reg_addr);
        while (len != 0)
        {
            *reg_data = spi_rw_byte(0x55);
            reg_data++;
            len--;
        }
        bmi088_accel_unselect();
    }
    else
    {
        bmi088_gyro_select();
        spi_rw_byte(reg_addr);
        while (len != 0)
        {
            *reg_data = spi_rw_byte(0x55);
            reg_data++;
            len--;
        }
        bmi088_gyro_unselect();
    }
    return 0;
}

/*!
 * SPI write function for stm32
 */
BMI08_INTF_RET_TYPE bmi08_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                    void *intf_ptr)
{
    uint8_t dev_addr = *(uint8_t *)intf_ptr;
    if (dev_addr == acc_dev_add)
    {
        bmi088_accel_select();
        spi_rw_byte(reg_addr);
        while (len != 0)
        {
            spi_rw_byte(*reg_data);
            reg_data++;
            len--;
        }
        bmi088_accel_unselect();
    }
    else
    {
        bmi088_gyro_select();
        spi_rw_byte(reg_addr);
        while (len != 0)
        {
            spi_rw_byte(*reg_data);
            reg_data++;
            len--;
        }
        bmi088_gyro_unselect();
    }
    return 0;
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi08_interface_init(struct bmi08_dev *bmi08, uint8_t intf, enum bmi08_variant variant)
{
    int8_t rslt = BMI08_OK;

    if (bmi08 != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI08_I2C_INTF)
        {
            return BMI08_E_FEATURE_NOT_SUPPORTED;
        }
        /* Bus configuration : SPI */
        else if (intf == BMI08_SPI_INTF)
        {

            /* To initialize the user SPI function */
            bmi08->intf = BMI08_SPI_INTF;
            bmi08->read = bmi08_spi_read;
            bmi08->write = bmi08_spi_write;
            acc_dev_add = BMI088_SPI_ACCEL;
            gyro_dev_add = BMI088_SPI_GYRO;
        }

        /* Selection of bmi085 or bmi088 sensor variant */
        bmi08->variant = variant;

        /* Assign accel device address to accel interface pointer */
        bmi08->intf_ptr_accel = &acc_dev_add;

        /* Assign gyro device address to gyro interface pointer */
        bmi08->intf_ptr_gyro = &gyro_dev_add;

        /* Configure delay in microseconds */
        bmi08->delay_us = bmi08_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        bmi08->read_write_len = BMI08_READ_WRITE_LEN;

        osDelay(10);
    }
    else
    {
        rslt = BMI08_E_NULL_PTR;
    }

    return rslt;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
