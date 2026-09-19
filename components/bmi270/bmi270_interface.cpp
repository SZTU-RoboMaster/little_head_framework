/**
 * @file bmi270_interface.cpp
 * @author anchengc
 * @brief
 * @version 0.1
 * @date 2026-09-18 0.1 初版
 *
 * @copyright SZTU-HJ (c) 2026
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "bmi270_interface.h"

#include "bsp_dwt.h"

#include "cmsis_os2.h"
#include "main.h"

/* Private macros ------------------------------------------------------------*/
#define BMI270_SPI SPI1

#define BMI270_READ_WRITE_LEN UINT8_C(46)

#define BMI270_TIMEOUT_CNT 1700000

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/*! Variable that holds the I2C device address or SPI chip selection for accel */
uint8_t acc_dev_add;

/*! Variable that holds the I2C device address or SPI chip selection for gyro */
uint8_t gyro_dev_add;

/* Private function declarations ---------------------------------------------*/

static inline void bmi270_cs_select(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

static inline void bmi270_cs_unselect(void)
{
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

/* function prototypes -------------------------------------------------------*/

uint8_t spi_rw_byte(uint8_t byte)
{
    uint32_t timeout_cnt = 0;
    SET_BIT(BMI270_SPI->CR1, SPI_CR1_SPE);
    while ((BMI270_SPI->SR & SPI_SR_TXE) == RESET)
    {
        if (timeout_cnt < BMI270_TIMEOUT_CNT)
        {
            timeout_cnt++;
        }
        else
        {
            return 0;
        }
    }
    // STM32默认是 __IO uint32_t DR，G4是 SPI FIFO，默认会进行宽访问，这里限制为8位字节访问
    *(__IO uint8_t *)&BMI270_SPI->DR = byte;
    timeout_cnt = 0;
    while ((BMI270_SPI->SR & SPI_SR_RXNE) == RESET)
    {
        if (timeout_cnt < BMI270_TIMEOUT_CNT)
        {
            timeout_cnt++;
        }
        else
        {
            return 0;
        }
    }
    return *(__IO uint8_t *)&BMI270_SPI->DR;
}

/*!
 * Delay function for stm32(systick)
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr)
{
    delay_us(period);
}

/*!
 * SPI read function for stm32
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                                    void *intf_ptr)
{
    bmi270_cs_select();
    spi_rw_byte(reg_addr);
    while (len != 0)
    {
        *reg_data = spi_rw_byte(0x55);
        reg_data++;
        len--;
    }
    bmi270_cs_unselect();
    return BMI2_OK;
}

/*!
 * SPI write function for stm32
 */
BMI2_INTF_RETURN_TYPE bmi2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                                     void *intf_ptr)
{
    bmi270_cs_select();
    spi_rw_byte(reg_addr);
    while (len != 0)
    {
        spi_rw_byte(*reg_data);
        reg_data++;
        len--;
    }
    bmi270_cs_unselect();
    return BMI2_OK;
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bmi2_interface_init(struct bmi2_dev *bmi, uint8_t intf)
{
    int8_t rslt = BMI2_OK;

    if (bmi != NULL)
    {
        /* Bus configuration : I2C */
        if (intf == BMI2_I2C_INTF)
        {
            return BMI2_E_COM_FAIL;
        }
        /* Bus configuration : SPI */
        else if (intf == BMI2_SPI_INTF)
        {
            /* To initialize the user SPI function */
            bmi->intf = BMI2_SPI_INTF;
            bmi->read = bmi2_spi_read;
            bmi->write = bmi2_spi_write;

            /* Configure delay in microseconds */
            bmi->delay_us = bmi2_delay_us;

            /* Configure max read/write length (in bytes) ( Supported length depends on target
             * machine) */
            bmi->read_write_len = BMI270_READ_WRITE_LEN;

            /* Assign to NULL to load the default config file. */
            bmi->config_file_ptr = NULL;
        }
        else
        {
            rslt = BMI2_E_DEV_NOT_FOUND;
        }
    }
    else
    {
        rslt = BMI2_E_NULL_PTR;
    }

    return rslt;
}

/*************************** COPYRIGHT(C) SZTU-HJ *****************************/
