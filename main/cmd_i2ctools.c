/* cmd_i2ctools.c

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "esp_console.h"
#include "esp_log.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "../components/ms5840/include/ms5840.h"
#include "../components/zoe_m8q/include/zoe_m8q.h"
#include "../components/ubx_proto/include/ubx_proto.h"

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
        printf("D1 (pressure) = %d\n", dev->d1);
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
        printf("D2 (temperature) = %d\n", dev->d2);
    }

    i2c_driver_delete(i2c_port);

    return 0;
}

static void print_alti_results(struct ms5840_results* results)
{
    float pressureMb = results->p / 100.0;
    printf("Temperature (C): %f\n", results->temp2);
    printf("Temperature (F): %f\n", (results->temp2 * 1.8) + 32.0);
    printf("Offset pressure: %lld\n", results->off2);
    printf("Sensitivity: %lld\n", results->sens2);
    printf("Temperature compensated pressure: %d\n", results->p);
    printf("Temperature compensated pressure (mb): %lf\n", pressureMb);

    // Convert mb to pressure altitude.
    double pressure_altitude_ft = 145366.45 * (1.0 - pow((pressureMb/1013.25), 0.190284));
    printf("Final temperature compensated altitude (ft): %lf\n", pressure_altitude_ft);
}

static int check_alti(int argc, char **argv)
{
    struct ms5840_data alti = {};
    struct ms5840_results results = {};

    int ret = acquire_alti_data(&alti, 1);
    //ms5840_load_example_data(&alti);
    if (ret != 0)
        return ret;

    ms5840_calc_results(&alti, &results);
    print_alti_results(&results);

    return 0;
}

static void print_buff(uint8_t* buff, uint32_t buff_len)
{
    printf("buff = { " );
    for (int i = 0; i < buff_len; ++i)
    {
        printf("%02x ", buff[i]);
    }
    printf("}\n" );
}

static int gps_get_data_size_registers(struct zoe_m8q* gps, uint8_t* fd, uint8_t* fe)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (gps->addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xFD, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (gps->addr << 1) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, fd, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, fe, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_port, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Error getting data size registers\n");
    }
    printf("Got data size registers: FD: %02x, FE: %02x\n", *fd, *fe);
    return ret;
}

static int gps_read_data(struct zoe_m8q* gps)
{
    uint8_t fd, fe;
    fd = fe = 0;
    int ret = gps_get_data_size_registers(gps, &fd, &fe);
    if (ESP_OK != ret) { return -1; }

    size_t data_size = fd;
    data_size = (data_size << 8 ) | fe;
    printf("Num bytes available: %u\n", data_size);
    if (data_size > 0)
    {
        uint8_t* data = (uint8_t*) malloc(data_size);

        i2c_cmd_handle_t  cmd = i2c_cmd_link_create();
        // Set register to data register
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (gps->addr << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, 0xFF, ACK_CHECK_EN);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (gps->addr << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read(cmd, data, data_size, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        // Send the command.
        ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ESP_OK != ret) {
            printf("Failed to retrieve data: %d\n", ret);
            return -1;
        }

        print_buff(data, data_size);
        free(data);
    }

    return 0;
}

static int gps_request_version_data(struct zoe_m8q* gps)
{
    uint8_t preamble[] = { UBX_PREAMBLE_C1, UBX_PREAMBLE_C2 };
    uint8_t mon_ver_cmd[4] = {};
    memset(mon_ver_cmd, 0, sizeof(mon_ver_cmd));
    mon_ver_cmd[0] = UBX_MON_CLASS;
    mon_ver_cmd[1] = UBX_MON_VER;
    uint8_t checksum[] = { 0, 0 };
    uint32_t cmd_len = sizeof(mon_ver_cmd);
    ubx_checksum(mon_ver_cmd, cmd_len, &checksum[0], &checksum[1]);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (gps->addr << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, preamble, sizeof(preamble), ACK_CHECK_EN);
    i2c_master_write(cmd, mon_ver_cmd, sizeof(mon_ver_cmd), ACK_CHECK_EN);
    i2c_master_write(cmd, checksum, sizeof(checksum), ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ESP_OK != ret) {
        printf("Failed to send UBX_MON_VER cmd: %d\n", ret);
        return -1;
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);

    return 0;
}

static int static_gps_read_data(int argc, char** argv)
{
    struct zoe_m8q gps = {};
    gps.addr = ZOE_M8Q_DEFAULT_I2C_ADDR;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    int ret = gps_read_data(&gps);
    if (ESP_OK != ret) {
        i2c_driver_delete(i2c_port);
        return -1;
    }

    i2c_driver_delete(i2c_port);
    return 0;
}

static int static_gps_request_ver(int argc, char** argv)
{
    struct zoe_m8q gps = {};
    gps.addr = ZOE_M8Q_DEFAULT_I2C_ADDR;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    int ret = gps_request_version_data(&gps);
    if (ESP_OK != ret) {
        i2c_driver_delete(i2c_port);
        return -1;
    }

    i2c_driver_delete(i2c_port);
    return 0;
}

static int check_gps(int argc, char **argv)
{
    struct zoe_m8q gps = {};
    gps.addr = ZOE_M8Q_DEFAULT_I2C_ADDR;

    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();

    int ret = gps_request_version_data(&gps);
    if (ESP_OK != ret) {
        i2c_driver_delete(i2c_port);
        return -1;
    }

    ret = gps_read_data(&gps);
    if (ESP_OK != ret) {
        i2c_driver_delete(i2c_port);
        return -1;
    }

    i2c_driver_delete(i2c_port);

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
        .help = "Print barometer data.",
        .hint = NULL,
        .func = &check_alti,
        .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_gpsprint(void)
{
    const esp_console_cmd_t cmd = {
            .command = "gps",
            .help = "Print gps data.",
            .hint = NULL,
            .func = &check_gps,
            .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_gps_ver(void)
{
    const esp_console_cmd_t cmd = {
            .command = "gps_ver",
            .help = "Submit gps version request.",
            .hint = NULL,
            .func = &static_gps_request_ver,
            .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

static void register_gps_read(void)
{
    const esp_console_cmd_t cmd = {
            .command = "gps_read",
            .help = "Try to read gps data register.",
            .hint = NULL,
            .func = &static_gps_read_data,
            .argtable = NULL
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&cmd));
}

void register_tools(void)
{
    register_scani2cbus();
    register_altiprint();
    register_gpsprint();
    register_gps_ver();
    register_gps_read();
}
