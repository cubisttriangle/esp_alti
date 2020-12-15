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
#include "ms5840.h"

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

/*
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
*/

static esp_err_t i2c_dev_available(uint8_t dev_addr)
{
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (dev_addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    i2c_driver_delete(i2c_port);

    return ret;
}

static int scan_i2c_bus(int argc, char **argv)
{
    esp_err_t ret;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

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
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    i2c_driver_delete(i2c_port);

    return 0;
}

static int acquire_alti_data(struct ms5840_data* dev, int reset_device)
{
    esp_err_t ret = i2c_dev_available(MS5840_I2C_ADDR);
    if (ESP_OK != ret)
    {
        printf("Barometer device at %02x not available. Ret: %s\n", MS5840_I2C_ADDR, esp_err_to_name(ret));
        return -1;
    }

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    if (reset_device)
    {
        printf("Resetting device.\n");
        // Reset the device.
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0x1e, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ESP_OK != ret) {
            printf("Failed to send i2c reset cmd: %d\n", ret);
            i2c_driver_delete(i2c_port);
            return -1;
        }
    }

    i2c_cmd_handle_t cmd;

    printf("Collecting calibration data.\n");
    // There are 7 PROM addresses and each response will be 16-bits, so 112 bits total.
    for (uint8_t i=0; i<7; ++i) {
        // Write
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, (MS5840_PROM_START_ADDR + i * 2), ACK_CHECK_EN);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ESP_OK != ret) {
            printf("Failed to send i2c cmd: %d\n", ret);
            i2c_driver_delete(i2c_port);
            return -1;
        }

        // Read
        uint8_t* data = (uint8_t*) &(dev->c)[i];
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data + 1, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, data, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ESP_OK != ret) {
            printf("Failed to send i2c PROM[%d] cmd: %d\n", (int)i, ret);
            i2c_driver_delete(i2c_port);
            return -1;
        } else {
            printf("PROM[%d] = %hu\n", (int)i, dev->c[i]);
        }
    }

    if (!ms5840_crcs_match(dev))
    {
        printf("CRCs don't match\n");
        i2c_driver_delete(i2c_port);
        return -1;
    }

    printf("Starting D1 (pressure) / D2 (temperature) conversions\n");
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MS5840_CMD_D1_4096, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c D1 write cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    }

    // Need to wait for specified time for reading to finish
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // Read pressure
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c ADC read pressure cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    }

    uint8_t* press_reading = (uint8_t*) &(dev->d1);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, press_reading + 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, press_reading + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, press_reading, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c D1 read cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    } else {
        printf("D1 = %d\n", dev->d1);
    }

    // D2 - temperature reading
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, MS5840_CMD_D2_4096, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c D2 write cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    }

    // Need to wait for specified time for reading to finish
    vTaskDelay(20 / portTICK_PERIOD_MS);

    // Read
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c D2 read cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    }

    //vTaskDelay(200 / portTICK_PERIOD_MS);

    uint8_t* temp_reading = (uint8_t*) &(dev->d2);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MS5840_I2C_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, temp_reading + 2, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, temp_reading + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, temp_reading, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send i2c D2 read cmd: %d\n", ret);
        i2c_driver_delete(i2c_port);
        return -1;
    } else {
        printf("D2 = %d\n", dev->d2);
    }

    i2c_driver_delete(i2c_port);

    return 0;
}

static int check_alti(int argc, char **argv)
{
    // Get and init alti struct.
    struct ms5840_data alti = {};

    int ret = acquire_alti_data(&alti, 1);
    //ms5840_load_example_data(&alti);
    if (ret != 0)
    {
        return ret;
    }

    struct ms5840_results results = {};
    ms5840_calc_results(&alti, &results);

    float pressureMb = results.p / 100.0;
    printf("Final temperature (C): %d\n", results.temp2);
    printf("Final temperature (F): %f\n", (results.temp2 * 1.8) + 32.0);
    printf("Final offset pressure: %lld\n", results.off2);
    printf("Final sensitivity: %lld\n", results.sens2);
    printf("Final temperature compensated pressure: %d\n", results.p);
    printf("Final temperature compensated pressure (mb): %lf\n", pressureMb);

    // Convert mb to pressure altitude.
    double pressure_altitude_ft = 145366.45 * (1.0 - pow((pressureMb/1013.25), 0.190284));
    printf("Final temperature compensated altitude (ft): %lf\n", pressure_altitude_ft);

    return 0;
}

static void register_scani2cbus(void)
{
    const esp_console_cmd_t cmd = {
            .command = "scan_i2c",
            .help = "Scan I2C bus for devices.",
            .hint = NULL,
            .func = &scan_i2c_bus,
            .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_altiprint(void)
{
    const esp_console_cmd_t cmd = {
        .command = "alti",
        .help = "Continuously print and log barometer data.",
        .hint = NULL,
        .func = &check_alti,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_tools(void)
{
    register_scani2cbus();
    register_altiprint();
}
