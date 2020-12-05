/* cmd_i2ctools.c

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

// SPI-3
#define MY_SPI_HOST SPI3_HOST
#define PIN_SPI_MISO 25
#define PIN_SPI_MOSI 23
#define PIN_SPI_CLK 18
#define DMA_CHAN 2

static gpio_num_t i2c_gpio_sda = 21;
static gpio_num_t i2c_gpio_scl = 22;
static uint32_t i2c_frequency = 100000;
static i2c_port_t i2c_port = I2C_NUM_0;

static esp_err_t i2c_master_driver_initialize(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

static esp_err_t spi_master_driver_initialize(void)
{
    printf("Initializing bus SPI...\n");
    spi_bus_config_t conf = {
            .miso_io_num = PIN_SPI_MISO,
            .mosi_io_num = PIN_SPI_MOSI,
            .sclk_io_num = PIN_SPI_CLK,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 32,
    };
    return spi_bus_initialize(MY_SPI_HOST, &conf, DMA_CHAN);
}

static uint8_t crc4(uint16_t n_prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
{
    int cnt; // simple counter
    unsigned int n_rem=0; // crc remainder
    unsigned char n_bit;
    n_prom[0]=((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7]=0; // Subsidiary value, set to 0
    for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
        else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
        for (n_bit = 8; n_bit > 0; n_bit--)
        {
            if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
            else n_rem = (n_rem << 1);
        }
    }
    n_rem= ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    return (n_rem ^ 0x00);
}

static int do_alti_cmd(int argc, char **argv)
{
    esp_err_t ret;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    if ( ESP_OK != (ret = spi_master_driver_initialize()))
    {
        printf("Couldn't initialize spi driver\n");
        return -1;
    }

    spi_device_interface_config_t dev_cfg = {
            .command_bits = 0,
            .address_bits = 0,
            .dummy_bits = 0,
            .mode = 0,
            .duty_cycle_pos = 0,
            .cs_ena_pretrans = 0,
            .cs_ena_posttrans = 0,
            .clock_speed_hz = SPI_MASTER_FREQ_10M,
            .input_delay_ns = 0,
            .spics_io_num = -1,
            .flags = 0,
            .queue_size = 1,
            .pre_cb = 0,
            .post_cb = 0,
    };
    spi_device_handle_t spi_handle;
    esp_err_t err = spi_bus_add_device(MY_SPI_HOST, &dev_cfg, &spi_handle);
    if (ESP_OK != err)
    {
        printf("Couldn't add spi bus device!\n");
    }

    uint8_t gps_address = 0x42;
    uint8_t found_gps = 0;

    uint8_t barometer_address = 0x76;
    uint8_t found_barometer = 0;

    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            i2c_cmd_link_delete(cmd);
            if (ret == ESP_OK) {
                printf("%02x ", address);
                if (address == barometer_address) {
                    found_barometer = 1;
                } else if (address == gps_address) {
                    found_gps = 1;
                }
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    if (found_barometer) {
        printf("Detected barometer at address: %02x\n", barometer_address);

        printf("Resetting device.\n");
        // Reset the device.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0x1e, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c reset cmd: %d\n", ret);
        }
        i2c_cmd_link_delete(cmd);

        printf("Collecting calibration data.\n");
        // There are 7 PROM addresses and each response will be 16-bits, so 112 bits total.
        uint8_t prom_start_address = 0xA0;
        uint16_t prom_vals[7] = {0};
        for (uint8_t i=0; i<7; ++i) {
            // Write
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd, (prom_start_address + i * 2), ACK_CHECK_EN);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            if (ESP_OK != ret) {
                printf("Failed to send i2c cmd: %d\n", ret);
            }
            i2c_cmd_link_delete(cmd);

            // Read
            uint8_t* data = (uint8_t*) &prom_vals[i];
            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (barometer_address << 1) | READ_BIT, ACK_CHECK_EN);
            i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK);
            i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
            ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
            if (ESP_OK != ret) {
                printf("Failed to send i2c PROM[%d] cmd: %d\n", (int)i, ret);
            } else {
                printf("PROM[%d] = %hu\n", (int)i, prom_vals[i]);
            }
            i2c_cmd_link_delete(cmd);
        }

        printf("Checking product type and CRC.\n");
        uint8_t factory_bits = prom_vals[0] & 0x1F; // First five bits.
        uint8_t product_type = (prom_vals[0] >> 5) & 0x7F; // Shift factory bits off and take 7 bits.
        switch (product_type) {
            case 0x00:
                printf("Product is MS5840-02BA21\n");
                break;
            case 0x24:
                printf("Product is MS5840-02BA36\n");
                break;
            default:
                printf("Unknown product type: %x\n", product_type);
        }
        uint8_t crc_bits = prom_vals[0] >> 12;
        uint8_t crc = crc4(prom_vals);
        printf("CRC retrieved: %d, calculated: %d\n", (int)crc_bits, (int)crc);
        // TODO: Don't continue if CRCs don't match

        printf("Starting D1 (pressure) / D2 (temperature) conversions\n");
        uint8_t pressure_cmd = 0x48; // 4K pressure
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, pressure_cmd, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c D1 write cmd: %d\n", ret);
        }
        i2c_cmd_link_delete(cmd);

        // Need to wait for specified time for reading to finish
        vTaskDelay(20 / portTICK_PERIOD_MS);

        // Read pressure
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c ADC read pressure cmd: %d\n", ret);
        }
        i2c_cmd_link_delete(cmd);

        uint32_t d1 = 0;
        uint8_t* press_reading = (uint8_t*) &d1;
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, press_reading + 2, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, press_reading + 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, press_reading, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c D1 read cmd: %d\n", ret);
        } else {
            printf("D1 = %d\n", d1);
        }
        i2c_cmd_link_delete(cmd);

        // D2 - temperature reading
        uint8_t temp_cmd = 0x58; // 4K temperature
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, temp_cmd, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c D2 write cmd: %d\n", ret);
        }
        i2c_cmd_link_delete(cmd);

        // Need to wait for specified time for reading to finish
        vTaskDelay(20 / portTICK_PERIOD_MS);

        // Read
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c D2 read cmd: %d\n", ret);
        }
        i2c_cmd_link_delete(cmd);

        //vTaskDelay(200 / portTICK_PERIOD_MS);

        uint32_t d2 = 0;
        uint8_t* temp_reading = (uint8_t*) &d2;
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (barometer_address << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, temp_reading + 2, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, temp_reading + 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, temp_reading, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        if (ESP_OK != ret) {
            printf("Failed to send i2c D2 read cmd: %d\n", ret);
        } else {
            printf("D2 = %d\n", d2);
        }
        i2c_cmd_link_delete(cmd);

        /*
         * First order calculations
         */
        // Calculate difference between actual and reference temperature.
        /*
        prom_vals[1] = 46372;
        prom_vals[2] = 43981;
        prom_vals[3] = 29059;
        prom_vals[4] = 27842;
        prom_vals[5] = 31553;
        prom_vals[6] = 28165;
        d1 = 6464444;
        d2 = 8077636;
        */
        int32_t deltaTemperature = d2 - prom_vals[5] * pow(2, 8);
        printf("Temperature delta from reference: %d\n", deltaTemperature);

        // Calculate actual temperature.
        int32_t actualTemperatureCelsius = 2000 + deltaTemperature * prom_vals[6] / pow(2, 23);
        printf("Actual temperature celsius: %d\n", actualTemperatureCelsius);

        // Calculate offset pressure at actual temperature.
        int64_t offsetAtActualTemp = prom_vals[2] * pow(2, 17) + deltaTemperature * prom_vals[4] / pow(2, 6);
        printf("Offset pressure at actual temperature: %lld\n", offsetAtActualTemp);

        // Calculate sensitivity at actual temperature.
        int64_t sensAtActualTemp = prom_vals[1] * pow(2, 16) + deltaTemperature * prom_vals[3] / pow(2, 7);
        printf("Sensitivity at actual temperature: %lld\n", sensAtActualTemp);

        // Calculate temperature compensated pressure.
        int32_t tempCompensatedPressure = (d1 * sensAtActualTemp / pow(2, 21) - offsetAtActualTemp) / pow(2, 15);
        printf("Temperature compensated pressure: %d\n", tempCompensatedPressure);

        /*
         * Second order calculations
         */
        int32_t ti;
        int64_t offi, sensi;
        if (actualTemperatureCelsius > 2000) {
            ti = 0;
            offi = 0;
            sensi = 0;
        } else if (actualTemperatureCelsius > 1000) {
            ti = 12 * pow(deltaTemperature, 2) / pow(2, 35);
            offi = 30 * pow((actualTemperatureCelsius - 2000), 2) / pow(2, 8);
            sensi = 0;
        } else {
            ti = 14 * pow(deltaTemperature, 2) / pow(2, 35);
            offi = 35 * pow((actualTemperatureCelsius - 2000), 2) / pow(2, 3);
            sensi = 63 * pow((actualTemperatureCelsius - 2000), 2) / pow(2, 5);
        }

        double temp2 = (actualTemperatureCelsius - ti) / 100.0;
        int64_t off2 = offsetAtActualTemp - offi;
        int64_t sens2 = sensAtActualTemp - sensi;
        int32_t pressure = (d1 * sens2 / pow(2, 21) - off2) / pow(2, 15);
        double pressureMb = pressure / 100.0;
        printf("Final temperature (C): %lf\n", temp2);
        printf("Final temperature (F): %f\n", (temp2 * 1.8) + 32.0);
        printf("Final offset pressure: %lld\n", off2);
        printf("Final sensitivity: %lld\n", sens2);
        printf("Final temperature compensated pressure: %d\n", pressure);
        printf("Final temperature compensated pressure (mb): %lf\n", pressureMb);

        // Convert mb to pressure altitude.
        double pressure_altitude_ft = 145366.45 * (1.0 - pow((pressureMb/1013.25), 0.190284));
        printf("Final temperature compensated altitude (ft): %lf\n", pressure_altitude_ft);
    }

    i2c_driver_delete(i2c_port);
    spi_bus_free(MY_SPI_HOST);

    return 0;
}

static void register_i2cdectect(void)
{
    const esp_console_cmd_t alti_cmd = {
        .command = "alti",
        .help = "Continuously print and log barometer data.",
        .hint = NULL,
        .func = &do_alti_cmd,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&alti_cmd));
}

void register_i2ctools(void)
{
    register_i2cdectect();
}
