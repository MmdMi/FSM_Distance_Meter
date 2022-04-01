#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <time.h>
#include <sys/time.h>
#include "esp_spi_flash.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "ultrasonic.h"

#define MAX_DISTANCE_CM 300
#define TRIGGER_GPIO 4
#define ECHO_GPIO 5
#define BUZZER_GPIO 14

float distance = 0;

bool buzzer_state = false;
long long buzzer_off_millis = 0;
long long buzzer_on_millis = 0;
ultrasonic_sensor_t sensor = {};

enum ReturnCodes
{
    ok,
    fail,
    repeat,
};

enum StateCodes
{
    GpioInitialize,
    UltrasonicInitialize,
    ReadDistance,
    BuzzerCheckState
};

struct TransitionTable
{
    enum StateCodes SourceState;
    enum ReturnCodes ReturnState;
    enum StateCodes DestState;
};

struct TransitionTable stateTransitionsTable[] = {
    {GpioInitialize, ok, UltrasonicInitialize},
    {UltrasonicInitialize, ok, ReadDistance},

    {ReadDistance, ok, BuzzerCheckState},
    {ReadDistance, fail, ReadDistance},
    {ReadDistance, repeat, ReadDistance},

    {BuzzerCheckState, ok, ReadDistance}};

enum ReturnCodes currentReturn;
enum StateCodes currentState = GpioInitialize;

enum ReturnCodes (*StateFunction)();
enum ReturnCodes gpio_initialize();
enum ReturnCodes ultrasonic_initialize();
enum ReturnCodes read_distance();
enum ReturnCodes buzzer_check_state();
enum ReturnCodes (*state[])() = {
    gpio_initialize,
    ultrasonic_initialize,
    read_distance,
    buzzer_check_state};

int64_t millis()
{
    return esp_timer_get_time() / 1000;
}

void app_main()
{
    while (true)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
        StateFunction = state[currentState];
        currentReturn = StateFunction();
        for (int i = 0; i < sizeof(stateTransitionsTable) / sizeof(stateTransitionsTable[0]); i++)
        {
            if (stateTransitionsTable[i].SourceState == currentState && stateTransitionsTable[i].ReturnState == currentReturn)
            {
                currentState = stateTransitionsTable[i].DestState;
                break;
            }
        }
    }
}

enum ReturnCodes gpio_initialize()
{
    static const char *TAG = "gpio_initialize";
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << BUZZER_GPIO);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "GPIO Init ok!");
    return ok;
}

enum ReturnCodes ultrasonic_initialize()
{
    static const char *TAG = "ultrasonic_initialize";
    sensor.trigger_pin = TRIGGER_GPIO;
    sensor.echo_pin = ECHO_GPIO;
    ultrasonic_init(&sensor);
    ESP_LOGI(TAG, "Ultrasonic Init ok!");
    return ok;
}

enum ReturnCodes read_distance()
{
    static const char *TAG = "read_distance";
    esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Error %d: ", res);
        switch (res)
        {
        case ESP_ERR_ULTRASONIC_PING:
            ESP_LOGE(TAG, "Cannot ping (device is in invalid state)\n");
            break;
        case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
            ESP_LOGE(TAG, "Ping timeout (no device found)\n");
            break;
        case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
            ESP_LOGE(TAG, "Echo timeout (i.e. distance too big)\n");
            break;
        default:
            ESP_LOGE(TAG, "%s\n", esp_err_to_name(res));
        }
        return fail;
    }
    ESP_LOGI(TAG, "Distance: %.2f cm", distance);
    if (distance > 10.0)
        return repeat;
    return ok;
}

enum ReturnCodes buzzer_check_state()
{
    static const char *TAG = "buzzer_check_state";

    if (buzzer_on_millis > millis())
    {
        gpio_set_level(BUZZER_GPIO, 1);
        return ok;
    }
    gpio_set_level(BUZZER_GPIO, 0);

    if ((buzzer_off_millis+(long long)(distance*700.0)) < millis())
    {
        buzzer_off_millis = millis();
        buzzer_on_millis = millis() + 50;
    }
    return ok;
}