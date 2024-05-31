#include <stdio.h>
#include "driver/spi_master.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "sdkconfig.h"

// Definición de pines
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_CS   GPIO_NUM_5

// Dirección I2C del BMP280
#define BMP280_ADDR 0x76

// Registro del BMP280
#define BMP280_REG_TEMP_XLSB 0xFC
#define BMP280_REG_TEMP_LSB  0xFB
#define BMP280_REG_TEMP_MSB  0xFA
#define BMP280_REG_PRESS_XLSB 0xF9
#define BMP280_REG_PRESS_LSB  0xF8
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_CALIB00    0x88

// Coeficientes de calibración
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint16_t dig_P1;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

// t_fine debe ser global
int32_t t_fine;
spi_device_handle_t spi;


// Función para inicializar el SPI
void init_spi() {
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_bus_initialize(HSPI_HOST, &buscfg, 1);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 10000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7
    };
    spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
}

void bmp280_write_reg(uint8_t reg, uint8_t value) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t tx_data[2] = {reg & 0x7F, value};
    t.length = 8 * 2; // 8 bits por byte, 2 bytes
    t.tx_buffer = tx_data;
    spi_device_transmit(spi, &t);
}

// Función para leer registros del BMP280
void bmp280_read_regs(uint8_t reg, uint8_t *data, size_t len) {
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    uint8_t tx_data = reg | 0x80;
    t.length = 8 * (len + 1); // 8 bits por byte, más 1 byte para el registro
    t.tx_buffer = &tx_data;
    t.rxlength = 8 * len;
    t.rx_buffer = data;
    spi_device_transmit(spi, &t);
}

// Función para inicializar el BMP280
void bmp280_init() {
    // Reiniciar el dispositivo
    bmp280_write_reg(BMP280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(100));

    // Leer los coeficientes de calibración
    uint8_t calib[24];
    bmp280_read_regs(BMP280_REG_CALIB00, calib, 24);

    dig_T1 = (calib[1] << 8) | calib[0];
    dig_T2 = (calib[3] << 8) | calib[2];
    dig_T3 = (calib[5] << 8) | calib[4];
    dig_P1 = (calib[7] << 8) | calib[6];
    dig_P2 = (calib[9] << 8) | calib[8];
    dig_P3 = (calib[11] << 8) | calib[10];
    dig_P4 = (calib[13] << 8) | calib[12];
    dig_P5 = (calib[15] << 8) | calib[14];
    dig_P6 = (calib[17] << 8) | calib[16];
    dig_P7 = (calib[19] << 8) | calib[18];
    dig_P8 = (calib[21] << 8) | calib[20];
    dig_P9 = (calib[23] << 8) | calib[22];

    // Configurar el sensor (oversampling y modo normal)
    bmp280_write_reg(BMP280_REG_CTRL_MEAS, 0x27);  // Oversampling x1 y modo normal
    bmp280_write_reg(BMP280_REG_CONFIG, 0xA0);     // Tiempo de espera de 1000 ms
}

// Función para leer la temperatura
int32_t bmp280_read_temperature() {
    uint8_t data[3];
    bmp280_read_regs(BMP280_REG_TEMP_MSB, data, 3);
    int32_t adc_T = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    int32_t T = (t_fine * 5 + 128) >> 8;
    return T / 100;
}

// Función para leer la presión
int64_t bmp280_read_pressure() {
    uint8_t data[3];
    bmp280_read_regs(BMP280_REG_PRESS_MSB, data, 3);
    int32_t adc_P = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);

    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
    var2 = var2 + (((int64_t)dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

    if (var1 == 0) {
        return 0; // evitar división por cero
    }

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
    return p/256;
}

void app_main(void) {
    // Inicializar el SPI
    init_spi();

    // Inicializar el BMP280
    bmp280_init();

    while (1) {
        // Leer y mostrar la temperatura
        int temperature = (int)bmp280_read_temperature();
        printf("Temperature: %d C\n", temperature);

        // Leer y mostrar la presión
        int pressure = (int)bmp280_read_pressure();
        printf("Pressure: %d hPa\n", pressure/256);

        // Esperar 1 segundo antes de la próxima lectura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
