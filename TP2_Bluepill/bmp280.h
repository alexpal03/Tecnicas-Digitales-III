/*
 * bmp280.h
 *
 *  Created on: May 19, 2025
 *      Author: HENTER SA
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Registros
#define BMP280_CHIP_ID_REG     0xD0
#define BMP280_RESET_REG       0xE0
#define BMP280_CTRL_MEAS_REG   0xF4
#define BMP280_CONFIG_REG      0xF5
#define BMP280_PRESS_MSB_REG   0xF7
#define BMP280_TEMP_MSB_REG    0xFA
#define BMP280_CALIB_START     0x88


// Estructura para los coeficientes de calibración
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

// Funciones públicas
void BMP280_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
float BMP280_ReadTemperature(void);
float BMP280_ReadPressure(void);


#endif /* INC_BMP280_H_ */
