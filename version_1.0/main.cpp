#include <stdio.h>
#include <stdint.h>
#include "stddef.h"
#include "string.h"
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <stdio.h>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "rencoder.hpp"

extern "C"
{
    void app_main(void);
}

// I2C setup
#define PIN_SCL 22
#define PIN_SDA 23

// LED setup
#define PIN_LED 13

// MPU setup
MPU6050 mpu = MPU6050();
// DMP setup
float ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Quaternion q;             // [w, x, y, z]         quaternion container
VectorFloat gravity;      // [x, y, z]            gravity vector
uint16_t packetSize = 42; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;       // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];   // FIFO storage buffer
uint8_t mpuIntStatus;     // holds actual interrupt status byte from MPU

// Motor setup
#define CH0_LEDC_GPIO 19    // Pin labeled MI (MISO)
#define CH1_LEDC_GPIO 16    // Pin labeled RX
#define CH2_LEDC_GPIO 17    // Pin labeled TX
#define CH3_LEDC_GPIO 21    // Pin labeled 21
#define NUM_LEDC_CHANNELS 4 // Total number of LEDC channels

// WiFi setup
#define WIFI_MAXIMUM_RETRY 5
EventGroupHandle_t s_wifi_event_group; // FreeRTOS event group to signal when we are connected
const int WIFI_CONNECTED_BIT = BIT0;   // BIT0 = are we connected to the access point with an IP?
const char *WIFI_TAG = "WiFi";

// PID setup

// Encoder setup
#define ENCODER_A_PIN_1 GPIO_NUM_14
#define ENCODER_A_PIN_2 GPIO_NUM_15
#define ENCODER_B_PIN_1 GPIO_NUM_27
#define ENCODER_B_PIN_2 GPIO_NUM_18

void command_parse(char *message)
{
    char *command = strtok(message, ".");
    if (strcmp(command, "forward") == 0)
    {
        printf("\nFORWARD\n");
        // TODO adjust motor speed offset for 1 second to move the robot forward
        // TODO add some sort of debounce so you cannot call this more than once every second
    }
    else if (strcmp(command, "reverse") == 0)
    {
        printf("REVERSE");
    }
}

esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    const char *MQTT_TAG = "MQTT";
    int msg_id;
    const char *topic = "boris_dylan_ee49/project/commands";
    // your_context_t *context = event->context;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        msg_id = esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, "sent subscribe successful, msg_id=%d", msg_id);
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_CONNECTED");
        msg_id = esp_mqtt_client_publish(client, topic, "Connected.", 0, 0, 0);
        ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        msg_id = esp_mqtt_client_publish(client, topic, "Subscribed.", 0, 0, 0);
        ESP_LOGI(MQTT_TAG, "sent publish successful, msg_id=%d", msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        // Parse command data and adjust balance_setpoint
        // 0 for forward, 1 for reverse
        command_parse(event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(MQTT_TAG, "MQTT_EVENT_ERROR");
        break;
    }
    return ESP_OK;
}

void init_mqtt()
{
    esp_mqtt_client_config_t mqtt_cfg = {};
    mqtt_cfg.uri = "ws://iot.eclipse.org:80/ws";
    mqtt_cfg.event_handle = mqtt_event_handler;
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    int s_retry_num = 0;
    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        ESP_LOGI(WIFI_TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        init_mqtt();
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        if (s_retry_num < WIFI_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
            s_retry_num++;
            ESP_LOGI(WIFI_TAG, "retry to connect to the AP");
        }
        ESP_LOGI(WIFI_TAG, "connect to the AP fail\n");
        break;
    }
    default:
        break;
    }
    return ESP_OK;
}

void init_comm(void *ignore)
{
    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint8_t WIFI_SSID[32] = "Dyldore";          // "EE49-2.4"
    uint8_t WIFI_PASS[64] = "dyldore_password"; // "122Hesse"

    wifi_config_t wifi_config = {};
    memcpy(wifi_config.sta.ssid, WIFI_SSID, 32);
    memcpy(wifi_config.sta.password, WIFI_PASS, 64);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(WIFI_TAG, "WiFi initialization finished.");
    ESP_LOGI(WIFI_TAG, "Connected to SSID: %s.", WIFI_SSID);
    vTaskDelete(NULL);
}

void init_i2c()
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = (gpio_num_t)PIN_SCL;
    conf.sda_io_num = (gpio_num_t)PIN_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
}

void init_mpu()
// ONLY RUN THIS ONCE!
// Then remove it from the program and reflash
{
    // MPU setup
    mpu.initialize();
    mpu.dmpInitialize();
    // This need to be setup individually
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76); // Default: 76
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // Default: 1788
    mpu.setDMPEnabled(true);
}

void init_pwm()
{
    // Motor driver initialization
    int ch;

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,  // timer mode
        .duty_resolution = LEDC_TIMER_8_BIT, // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,           // timer index
        .freq_hz = 30000                     // frequency of PWM signal
    };

    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel[NUM_LEDC_CHANNELS] = {
        {.gpio_num = CH0_LEDC_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_0,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},

        {.gpio_num = CH1_LEDC_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_1,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},

        {.gpio_num = CH2_LEDC_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_2,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0},

        {.gpio_num = CH3_LEDC_GPIO,
         .speed_mode = LEDC_HIGH_SPEED_MODE,
         .channel = LEDC_CHANNEL_3,
         .intr_type = LEDC_INTR_DISABLE,
         .timer_sel = LEDC_TIMER_0,
         .duty = 0,
         .hpoint = 0}};

    // Set LED channels with previously prepared configuration and set all duty cycles to 0
    for (ch = 0; ch < NUM_LEDC_CHANNELS; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, ledc_channel[ch].channel, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, ledc_channel[ch].channel);
    }
}

void get_dmp()
{
    mpuIntStatus = mpu.getIntStatus();
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    }
    else if (mpuIntStatus & 0x02) // otherwise, check for DMP data ready interrupt frequently)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // printf("PITCH: %1.3f\n", ypr[1]);
    }
}

void blink_led(void *ignore)
{
    gpio_pad_select_gpio(PIN_LED);
    // Set the GPIO as a push/pull velocity_output
    gpio_set_direction((gpio_num_t)PIN_LED, GPIO_MODE_OUTPUT);
    while (true)
    {
        // Blink off (output low)
        gpio_set_level((gpio_num_t)PIN_LED, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        // Blink on (output high)
        gpio_set_level((gpio_num_t)PIN_LED, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void balance_control_loop(void *ignore)
{
    // Best result is to match with DMP refresh rate
    // Its last value in components/MPU6050/MPU6050_6Axis_MotionApps20.h file line 312
    // Now its 0x01, which means DMP is refreshed with 100Hz rate
    const float LOOP_PERIOD_MS = 10;                   //ms
    const float LOOP_PERIOD_S = LOOP_PERIOD_MS / 1000; //s
    init_i2c();
    init_pwm();
    bool dmp_test_1 = mpu.getDMPConfig1();
    bool dmp_test_2 = mpu.getDMPConfig2();
    if (!dmp_test_1 && !dmp_test_2) // Check if DMP is init yet
    {
        init_mpu();
    }
    xTaskCreate(&blink_led, "blink_led", 1024, NULL, 5, NULL);
    float p = 0, i = 0, d = 0, output = 0;
    const float kp = 1500, ki = 10000, kd = 60;
    const float setpoint = 0.039;
    float error = 0, last_error = 0;

    while (true)
    {
        get_dmp();
        error = ypr[1] - setpoint;
        p = kp * error;
        i += ki * LOOP_PERIOD_S * error;
        d = kd * (error - last_error) / LOOP_PERIOD_S;
        last_error = error;
        output = p + i + d;

        if (output > 256)
        {
            output = 256;
        }
        else if (output < -256)
        {
            output = -256;
        }

        if (ypr[1] < -0.8 || ypr[1] > 0.8)
        {
            output = 0;
            error = 0;
            p = 0;
            i = 0;
            d = 0;
        }

        if (output >= 0)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, output);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, output);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, 0);
        }
        else if (output < 0)
        {
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, abs(output));
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3, abs(output));
        }

        // Motor A
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        // Motor B
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_3);

        printf("P: %1.3f, D: %1.3f, OUTPUT: %1.3f\n", p, d, output);
        // printf("%1.3f\n", i);

        vTaskDelay(LOOP_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initializations
    nvs_flash_init(); // IDK what this does exactly but it is needed for the LED, WiFi, and MQTT
    /// xTaskCreate(&init_comm, "init_comm", 4096, NULL, 5, NULL);

    // Loops
    xTaskCreate(&balance_control_loop, "balance_control_loop", 20480, NULL, 5, NULL);
}
