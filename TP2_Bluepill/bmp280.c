#include "bmp280.h"

static SPI_HandleTypeDef *spi;
static GPIO_TypeDef *cs_port;
static uint16_t cs_pin;
static BMP280_CalibData calib_data;
static int32_t t_fine;

// Funciones internas para SPI
static void CS_Select(void) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
}

static void CS_Unselect(void) {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
}


static void bmp280_read_bytes(uint8_t reg, uint8_t *data, uint16_t len) {
    reg |= 0x80;
    CS_Select();
    HAL_SPI_Transmit(spi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(spi, data, len, HAL_MAX_DELAY);
    CS_Unselect();
}

static void bmp280_write8(uint8_t reg, uint8_t val) {
    uint8_t data[2] = { reg & 0x7F, val };
    CS_Select();
    HAL_SPI_Transmit(spi, data, 2, HAL_MAX_DELAY);
    CS_Unselect();
}

// Lectura de coeficientes de calibración
static void bmp280_read_calibration(void) {
    uint8_t calib[24];
    bmp280_read_bytes(BMP280_CALIB_START, calib, 24);

    calib_data.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    calib_data.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    calib_data.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    calib_data.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    calib_data.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    calib_data.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    calib_data.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    calib_data.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    calib_data.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    calib_data.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    calib_data.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    calib_data.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

void BMP280_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_gpio, uint16_t cs_gpio_pin) {
    spi = hspi;
    cs_port = cs_gpio;
    cs_pin = cs_gpio_pin;

    CS_Unselect();
    HAL_Delay(100);

    bmp280_write8(BMP280_RESET_REG, 0xB6); // Soft reset
    HAL_Delay(10);

    uint8_t data[1];
    bmp280_read_bytes(BMP280_CHIP_ID_REG, data, 1);
    if (data[0] != 0x58) while(1);  // 0x58 = BMP280 ID


    bmp280_read_calibration();

    // Configurar el sensor:
    // - Oversampling x1 para temp y presión
    // - Modo normal
    bmp280_write8(BMP280_CTRL_MEAS_REG, 0x27);  // 0010 0111
    // - Tiempo de espera 0.5ms, filtro desactivado
    bmp280_write8(BMP280_CONFIG_REG, 0x00);
}

float BMP280_ReadTemperature(void) {
    uint8_t data[3];
    bmp280_read_bytes(BMP280_TEMP_MSB_REG, data, 3);
    int32_t adc_T = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));

    // Fórmulas del datasheet sección 4.2.3
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) *
                    ((int32_t)calib_data.dig_T3)) >> 14;
    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float BMP280_ReadPressure(void) {
    uint8_t data[3];
    bmp280_read_bytes(BMP280_PRESS_MSB_REG, data, 3);
    int32_t adc_P = (int32_t)(((uint32_t)data[0] << 12) | ((uint32_t)data[1] << 4) | (data[2] >> 4));

    // Fórmulas del datasheet sección 4.2.3
    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
    var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;

    if (var1 == 0) return 0;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
    return (float)p / 25600.0f;
}
