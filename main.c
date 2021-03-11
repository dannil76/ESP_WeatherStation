#include "espressif/esp_common.h"
#include "esp/uart.h"

#include <string.h>

#include <FreeRTOS.h>
#include <task.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <semphr.h>

#include <i2c/i2c.h>
#include <bmp280/bmp280.h>
#include <ds18b20/ds18b20.h>
#include <si7021/si7021.h>

#include "config.h"

// I2C parameters
#define I2C_BUS 0
#define SCL_PIN 14
#define SDA_PIN 2

// DS18B20 parameters
#define DS18B20_GPIO 5
#define MAX_SENSORS 1
#define RESCAN_INTERVAL 8
#define LOOP_DELAY_MS 250

// LED I/O
#define LED_GPIO 4
#define LED_ON 0
#define LED_OFF 1

#define SAMP_INTERVAL 1   // Sampling interval (seconds)
#define AVG_TIME 10       // Total average time (seconds)

// Semaphores
SemaphoreHandle_t wifi_alive;
SemaphoreHandle_t i2c_lock;

// Queues
QueueHandle_t publish_humid_queue;
QueueHandle_t publish_press_queue;
QueueHandle_t publish_temp_queue;

#define PUB_MSG_LEN 16


// Humidity measurment task
static void humid_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];

    bool si7021_found = false;
    bool avg_init = false;

    float si7021_temperature, si7021_humidity;

    float humidity;
    float humid_sum = 0.0;
    float humid_avg[AVG_TIME / SAMP_INTERVAL];

    int avg_index = 0;
    int i;

    printf("%s: started\n", __func__);

    // Get exclusive access to I2C
    xSemaphoreTake(i2c_lock, portMAX_DELAY);

    /* SI7021 (humidity) initlization */
    si7021_t si7021_dev;
    si7021_dev.i2c_dev.bus = I2C_BUS;
    si7021_dev.i2c_dev.addr = SI7021_I2C_ADDRESS;
    si7021_found = si7021_init(&si7021_dev);

    // Release I2C
    xSemaphoreGive(i2c_lock);

    // Stop is sensor is not found
    if (!si7021_found) {
        printf("No SI7021 humidity sensors detected!\n");
        while (1);
    }

    /* Initialize wakeup timer */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Sleep
        vTaskDelayUntil(&xLastWakeTime, HUMID_SAMP_INTERVAL / portTICK_PERIOD_MS);

        // printf("Humidity measure\n");
        /********************************************/
        // Get exclusive access to I2C
        xSemaphoreTake(i2c_lock, portMAX_DELAY);

        /* Get humidity and temperature from SI7021 */
        if (!si7021_read_float(&si7021_dev, &si7021_temperature, &si7021_humidity)) {
            printf("SI7021 reading failed\n");
        }

        // Release I2C
        xSemaphoreGive(i2c_lock);

        /* Initialize moving average buffers the first time */
        if (!avg_init) {
            humid_sum = si7021_humidity * (AVG_TIME/SAMP_INTERVAL);

            for (i = 0; i < AVG_TIME/SAMP_INTERVAL; i++) {
                humid_avg[i] = si7021_humidity;
            }
            avg_init = true;
        }

        /* Calculate moving average */
        humid_sum -= humid_avg[avg_index];
        humid_sum += si7021_humidity;
        humid_avg[avg_index] = si7021_humidity;
        humidity = humid_sum / (AVG_TIME/SAMP_INTERVAL);

        avg_index++;
        avg_index %= AVG_TIME/SAMP_INTERVAL;  // Wrap

        printf("Temperature:     %.1f C     (SI7021)\n", si7021_temperature);
        printf("Humidity:        %.1f %% Rh  (SI7021)\n", si7021_humidity);
        printf("Avg humidity:    %.1f %%\n", humidity);
        printf("\n");

        snprintf(msg, PUB_MSG_LEN, "%.1f", humidity);
        if (xQueueSend(publish_humid_queue, (void *)msg, portMAX_DELAY) == pdFALSE) {
            printf("Humidity publish queue overflow.\n");
        }
    }
}

// Pressure measurment task
static void press_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];

    bool bmp280_found = false;
    bool avg_init = false;

    float bmp280_pressure, bmp280_temperature, bmp280_humidity;

    float pressure;
    float press_sum = 0.0;
    float press_avg[AVG_TIME/SAMP_INTERVAL];

    int avg_index = 0;
    int i;

    printf("%s: started\n", __func__);

    // Get exclusive access to I2C
    xSemaphoreTake(i2c_lock, portMAX_DELAY);

    /* BMP280 (pressure) initialization */
    bmp280_params_t  params;
    bmp280_t bmp280_dev;

    bmp280_init_default_params(&params);

    bmp280_dev.i2c_dev.bus = I2C_BUS;
    bmp280_dev.i2c_dev.addr = BMP280_I2C_ADDRESS_0;
    bmp280_found = bmp280_init(&bmp280_dev, &params);
    bmp280_found = bmp280_found & (bmp280_dev.id == BMP280_CHIP_ID);

    // Release I2C
    xSemaphoreGive(i2c_lock);

    // Stop is sensor is not found
    if (!bmp280_found) {
        printf("No BMP280 pressure sensors detected!\n");
        while (1);
    }

    /* Initialize wakeup timer */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Sleep 2 sec
        vTaskDelayUntil(&xLastWakeTime, PRESS_SAMP_INTERVAL / portTICK_PERIOD_MS);

        // printf("Pressure measure\n");

        /********************************************/
        /* Get pressure and temperature from BMP280 */
        // Get exclusive access to I2C
        xSemaphoreTake(i2c_lock, portMAX_DELAY);

        if (!bmp280_read_float(&bmp280_dev, &bmp280_temperature, &bmp280_pressure, &bmp280_humidity)) {
            printf("BMP280 reading failed\n");
        }

        // Release I2C
        xSemaphoreGive(i2c_lock);

        /* Initialize moving average buffers the first time */
        if (!avg_init) {
            press_sum = bmp280_pressure * (AVG_TIME/SAMP_INTERVAL);

            for (i = 0; i < AVG_TIME/SAMP_INTERVAL; i++) {
                press_avg[i] = bmp280_pressure;
            }
            avg_init = true;
        }

        /* Calculate moving average */
        press_sum -= press_avg[avg_index];
        press_sum += bmp280_pressure;
        press_avg[avg_index] = bmp280_pressure;
        pressure = press_sum / (AVG_TIME/SAMP_INTERVAL);

        avg_index++;
        avg_index %= AVG_TIME/SAMP_INTERVAL;  // Wrap

        printf("Temperature:     %.1f C     (BMP280)\n", bmp280_temperature);
        printf("Pressure:        %.1f hPa  (BMP280)\n", bmp280_pressure/100.0);
        printf("Avg pressure:    %.1f hPa\n", pressure/100.0);
        printf("\n");

        snprintf(msg, PUB_MSG_LEN, "%.1f", pressure/100.0);
        if (xQueueSend(publish_press_queue, (void *)msg, portMAX_DELAY) == pdFALSE) {
            printf("Pressure publish queue overflow.\n");
        }
    }
}

// Temperature measurment task
static void temp_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];

    bool ds18b20_found = false;
    bool avg_init = false;

    float ds18b20_temperature;

    float temperature;
    float temp_sum = 0.0;
    float temp_avg[AVG_TIME/SAMP_INTERVAL];

    int avg_index = 0;
    int i;

    printf("%s: started\n", __func__);

    /* DS18B20 (temperature sensor) initialization */
    ds18b20_addr_t ds18b20_addr;
    int ds18b20_count;

    ds18b20_count = ds18b20_scan_devices(DS18B20_GPIO, &ds18b20_addr, MAX_SENSORS);
    ds18b20_found = (ds18b20_count >= 1) ? true : false;

    // Stop is sensor is not found
    if (!ds18b20_found) {
        printf("No DS18B20 temperature sensors detected!\n");
        while (1);
    }

    /* Initialize wakeup timer */
    xLastWakeTime = xTaskGetTickCount();

    while (1) {
        // Sleep 1 sec
        vTaskDelayUntil(&xLastWakeTime, TEMP_SAMP_INTERVAL / portTICK_PERIOD_MS);

        // printf("Temperature measure\n");

        /********************************************/
        /* Get temperature from DS18B20             */
        ds18b20_measure_and_read_multi(DS18B20_GPIO, &ds18b20_addr, 1, &ds18b20_temperature);

        /* Initialize moving average buffers the first time */
        if (!avg_init) {
            temp_sum = ds18b20_temperature * (AVG_TIME/SAMP_INTERVAL);

            for (i = 0; i < AVG_TIME/SAMP_INTERVAL; i++) {
                temp_avg[i]  = ds18b20_temperature;
            }
            avg_init = true;
        }

        temp_sum  -= temp_avg[avg_index];
        temp_sum  += ds18b20_temperature;
        temp_avg[avg_index] = ds18b20_temperature;
        temperature = temp_sum / (AVG_TIME/SAMP_INTERVAL);

        avg_index++;
        avg_index %= AVG_TIME/SAMP_INTERVAL;  // Wrap

        printf("Temperature:     %.1f C    (DS18B20)\n", ds18b20_temperature);
        printf("Avg temperature: %.1f C\n", temperature);
        printf("\n");

        snprintf(msg, PUB_MSG_LEN, "%.1f", temperature);
        if (xQueueSend(publish_temp_queue, (void *)msg, portMAX_DELAY) == pdFALSE) {
            printf("Tempetarure publish queue overflow.\n");
        }
    }
}

// MQTT message receiver
static void topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    printf("Received: ");
    for( i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[i]);

    printf(" = ");
    for( i = 0; i < (int)message->payloadlen; ++i)
        printf("%c", ((char *)(message->payload))[i]);

    printf("\n");
}


static const char* get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}


static void mqtt_task(void *pvParameters)
{
    int ret = 0;
    struct mqtt_network network;
    mqtt_client_t client = mqtt_client_default;
    char mqtt_client_id[20];
    uint8_t mqtt_buf[100];
    uint8_t mqtt_readbuf[100];

    printf("%s: started\n", __func__);

    /* Initialize LED */
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    gpio_write(LED_GPIO, LED_OFF);

    printf("mqtt_task waiting for wifi_alive\n");
    xSemaphoreTake(wifi_alive, portMAX_DELAY);
    printf("mqtt_task wifi is alive!\n");

    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "DML-");
    strcat(mqtt_client_id, get_my_id());
    printf("mqtt_client_id %s\n", mqtt_client_id);

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__, MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n", ret);
            taskYIELD();
            continue;
        }
        printf("done\n");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, 100, mqtt_readbuf, 100);

        data.willFlag = 0;
        data.MQTTVersion = 3;
        data.clientID.cstring = mqtt_client_id;
        data.username.cstring = MQTT_USER;
        data.password.cstring = MQTT_PASS;
        data.keepAliveInterval = 10;
        data.cleansession = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\n");
        mqtt_subscribe(&client, CTRL_TOPIC, MQTT_QOS1, topic_received);

        char msg[PUB_MSG_LEN - 1] = "\0";
        mqtt_message_t message;

        // Loop forever, or at least as long as the MQTT link is up and running
        while(1){
            if (xQueueReceive(publish_humid_queue, (void *)msg, 0) == pdTRUE) {
                // printf("got humidity message to publish\n");
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, HUMID_TOPIC, &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing humidity message: %d\n", ret );
                }

                // Toggle LED
                gpio_write(LED_GPIO, LED_ON);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_write(LED_GPIO, LED_OFF);
            }

            if (xQueueReceive(publish_press_queue, (void *)msg, 0) == pdTRUE) {
                // printf("got pressure message to publish\n");
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, PRESS_TOPIC, &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing pressure message: %d\n", ret );
                }

                // Toggle LED
                gpio_write(LED_GPIO, LED_ON);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_write(LED_GPIO, LED_OFF);
            }

            if (xQueueReceive(publish_temp_queue, (void *)msg, 0) == pdTRUE) {
                // printf("got temperature message to publish\n");
                message.payload = msg;
                message.payloadlen = strlen(msg);
                message.dup = 0;
                message.qos = MQTT_QOS1;
                message.retained = 0;
                ret = mqtt_publish(&client, TEMP_TOPIC, &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing temperature message: %d\n", ret );
                }

                // Toggle LED
                gpio_write(LED_GPIO, LED_ON);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                gpio_write(LED_GPIO, LED_OFF);
            }

            // Wait 50 ms (for LED blink)
//            vTaskDelay(50 / portTICK_PERIOD_MS);
//            gpio_write(LED_GPIO, LED_OFF);

            ret = mqtt_yield(&client, 1000);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }
}


// WiFi task
static void wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("%s: started\n", __func__);

    printf("WiFi: connecting to WiFi\n");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1) {
        while ((status != STATION_GOT_IP) && (retries)) {
            status = sdk_wifi_station_get_connect_status();
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n");
                break;
            } else if(status == STATION_NO_AP_FOUND) {
                printf("WiFi: AP not found\n");
                break;
            } else if(status == STATION_CONNECT_FAIL) {
                printf("WiFi: connection failed\n");
                break;
            } else if(status == STATION_IDLE) {
                printf("WiFi: station idle\n");
            } else if(status == STATION_CONNECTING) {
                printf("WiFi: station connecting\n");
            } else if(status == STATION_NO_AP_FOUND) {
                printf("WiFi: no AP found\n");
                break;
            } else {
                printf("WiFi: unknown status: %d\n", status);
            }

            // Wait 1 second, then retry
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            --retries;
        }
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n");
            xSemaphoreGive(wifi_alive);
            taskYIELD();
        }

        // Monitor connection
        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive(wifi_alive);
            taskYIELD();
        }

        // Revoke wifi_alive semaphore
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("WiFi: disconnected\n");
        sdk_wifi_station_disconnect();

        // Wait 1 second, then retry
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}


// Main initialization function
void user_init(void) {
    // Initialize UART
    uart_set_baud(0, UART_BAUD_RATE);

    // Initialize I2C Controller
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_400K);

    // Initialize semaphores
    wifi_alive = xSemaphoreCreateBinary();
    i2c_lock = xSemaphoreCreateBinary();
    xSemaphoreGive(i2c_lock);               // Release i2C_lock

    // Initialize queues
    publish_humid_queue = xQueueCreate(2, PUB_MSG_LEN);
    publish_press_queue = xQueueCreate(2, PUB_MSG_LEN);
    publish_temp_queue = xQueueCreate(2, PUB_MSG_LEN);

    // Initialize tasks
    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 1024, NULL, 4, NULL);
    xTaskCreate(&humid_task, "humid_task", 1024, NULL, 3, NULL);
    xTaskCreate(&press_task, "press_task", 1024, NULL, 3, NULL);
    xTaskCreate(&temp_task, "temp_task", 1024, NULL, 3, NULL);
}
