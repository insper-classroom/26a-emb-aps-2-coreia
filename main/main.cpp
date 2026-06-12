#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "Fusion.h"
#include "audio_jogo.h"

// Edge Impulse classifier used by ai_task while the pause button is held.
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

#define SAMPLE_PERIOD (EI_CLASSIFIER_INTERVAL_MS / 1000.0f)
#define WARMUP_SAMPLES 200
#define SCALE_DEG 45.0f

const int MPU_ADDRESS = 0x68;
const int I2C_SDA_GPIO = 16;
const int I2C_SCL_GPIO = 17;

#define LED_R_PIN 7
#define LED_G_PIN 8
#define LED_B_PIN 9

#define PIN_INSTR_MPU 15
#define PIN_INSTR_FUSION 11
#define PIN_INSTR_PWM 12
#define PIN_INSTR_UART 13
#define PIN_INSTR_BUTTON 18
#define PIN_INSTR_AI 19
#define PIN_INSTR_AUDIO_DEBUG 20

#define BTN_PAUSE_PIN 2 // segurado -> pausa o IMU
#define BTN_ENTER_PIN 3
#define BTN_ROT_R_PIN 4 // gira direita -> seta cima
#define BTN_ESC_PIN 5
#define BTN_DEBOUNCE_US 20000

#define AUDIO_PIN 6
#define AUDIO_SAMPLE_RATE 11000
#define AUDIO_MIDPOINT 121
#define AUDIO_GAIN 2

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define SYNC_BYTE 0xFF
#define VALUE_OFFSET 128
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_CLICK 2
#define AXIS_ROT_R 3
#define AXIS_SPACE 5
#define AXIS_AI_CONFIDENCE 6
#define AXIS_AI_STATUS 7
#define AXIS_AI_PROGRESS 8
#define AXIS_DEBUG_ACCEL_X 9
#define AXIS_DEBUG_ACCEL_Y 10
#define AXIS_DEBUG_ACCEL_Z 11
#define AXIS_AI_IDLE_CONFIDENCE 12
#define AXIS_AI_ANOMALY 13
#define AXIS_AI_DSP_MS 14
#define AXIS_AI_CLASSIFICATION_MS 15
#define AXIS_AI_QUEUE_DEPTH 16
#define AXIS_AI_ERROR_CODE 17
#define AXIS_IMU_READ_ERRORS 18
#define AXIS_ENTER 19
#define AXIS_ESC 20
#define AXIS_AUDIO_STATUS 21
#define AXIS_AUDIO_INDEX_LOW 22
#define AXIS_AUDIO_INDEX_MID 23
#define AXIS_AUDIO_INDEX_HIGH 24
#define MAX_ABS_VALUE 94

#define AUDIO_STATUS_STARTED 1
#define AUDIO_STATUS_PAUSED 2
#define AUDIO_STATUS_RESUMED 3

#define AI_GESTURE_LABEL "tetris"
#define AI_GESTURE_THRESHOLD 0.7f
#define AI_GESTURE_COOLDOWN_MS 600
#define AI_INFERENCE_STRIDE_SAMPLES (EI_CLASSIFIER_RAW_SAMPLE_COUNT / 4)

#define AI_STATUS_TASK_STARTED 1
#define AI_STATUS_PAUSE_ACTIVE 2
#define AI_STATUS_WINDOW_READY 3
#define AI_STATUS_CLASSIFIER_ERROR 4
#define AI_STATUS_TETRIS_DETECTED 5
#define AI_STATUS_PAUSE_RELEASED 6

typedef struct
{
    int16_t accel[3];
    int16_t gyro[3];
} mpu_data_t;
typedef struct
{
    int8_t x, y;
    bool click;
} pos_data_t;
typedef struct
{
    uint8_t r, g, b;
} color_data_t;
typedef enum
{
    AUDIO_COMMAND_START,
    AUDIO_COMMAND_TOGGLE_PAUSE,
    AUDIO_COMMAND_REPORT_INDEX,
} audio_command_t;
typedef struct
{
    uint8_t status;
    uint32_t index;
} audio_event_t;
typedef struct
{
    uint32_t index;
    bool started;
    bool paused;
} audio_state_t;

QueueHandle_t xQueueMPU;
QueueHandle_t xQueueAI;
QueueHandle_t xQueuePos;
QueueHandle_t xQueueColor;
QueueHandle_t xQueueBtn;
QueueHandle_t xQueueAudioCommand;
QueueHandle_t xQueueAudioEvent;
SemaphoreHandle_t xMutexUart;
SemaphoreHandle_t xMutexI2C;
static repeating_timer_t audio_timer;

static void send_packet(uint8_t axis, int8_t value);

static void instrumentation_init(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
}

static void pwm_init_pin(uint pin)
{
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_wrap(slice, 255);
    pwm_set_enabled(slice, true);
}

static void pwm_set_duty(uint pin, uint8_t duty)
{
    pwm_set_chan_level(pwm_gpio_to_slice_num(pin), pwm_gpio_to_channel(pin), duty);
}

static void mpu6050_init()
{
    i2c_init(i2c0, 400 * 1000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);

    // Wake the MPU6050 and make its output rate/ranges explicit. With the
    // default 1 kHz gyro output rate, divider 10 gives 90.9 Hz, matching the
    // model's 91 Hz training frequency.
    const uint8_t config[][2] = {
        {0x6B, 0x00}, // PWR_MGMT_1: wake
        {0x1A, 0x03}, // CONFIG: DLPF enabled, 1 kHz internal sample rate
        {0x19, 0x0A}, // SMPLRT_DIV: 1000 / (1 + 10) = 90.9 Hz
        {0x1B, 0x00}, // GYRO_CONFIG: +/-250 deg/s
        {0x1C, 0x00}, // ACCEL_CONFIG: +/-2g, 16384 LSB/g
    };
    for (size_t ix = 0; ix < sizeof(config) / sizeof(config[0]); ix++)
        i2c_write_blocking(i2c0, MPU_ADDRESS, config[ix], 2, false);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static bool audio_timer_callback(repeating_timer_t *timer)
{
    audio_state_t *state = static_cast<audio_state_t *>(timer->user_data);
    audio_command_t command;
    audio_event_t event = {0, 0};
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (xQueueReceiveFromISR(xQueueAudioCommand, &command, &higher_priority_task_woken) == pdTRUE)
    {
        if (command == AUDIO_COMMAND_START && !state->started)
        {
            state->started = true;
            state->paused = false;
            event.status = AUDIO_STATUS_STARTED;
            xQueueSendFromISR(xQueueAudioEvent, &event, &higher_priority_task_woken);
        }
        else if (command == AUDIO_COMMAND_TOGGLE_PAUSE && state->started)
        {
            state->paused = !state->paused;
            event.status = state->paused ? AUDIO_STATUS_PAUSED : AUDIO_STATUS_RESUMED;
            xQueueSendFromISR(xQueueAudioEvent, &event, &higher_priority_task_woken);
        }
        else if (command == AUDIO_COMMAND_REPORT_INDEX && state->started)
        {
            event.index = state->index;
            xQueueSendFromISR(xQueueAudioEvent, &event, &higher_priority_task_woken);
        }
    }

    portYIELD_FROM_ISR(higher_priority_task_woken);

    if (!state->started || state->paused)
    {
        pwm_set_gpio_level(AUDIO_PIN, AUDIO_MIDPOINT);
        return true;
    }

    int32_t amplified =
        AUDIO_MIDPOINT + ((int32_t)WAV_DATA[state->index] - AUDIO_MIDPOINT) * AUDIO_GAIN;
    if (amplified < 0)
        amplified = 0;
    else if (amplified > 255)
        amplified = 255;
    pwm_set_gpio_level(AUDIO_PIN, (uint16_t)amplified);
    state->index++;
    if (state->index >= WAV_DATA_LENGTH)
        state->index = 0;
    return true;
}

static void audio_init(audio_state_t *state)
{
    pwm_init_pin(AUDIO_PIN);
    gpio_set_drive_strength(AUDIO_PIN, GPIO_DRIVE_STRENGTH_2MA);
    gpio_set_slew_rate(AUDIO_PIN, GPIO_SLEW_RATE_SLOW);
    pwm_set_gpio_level(AUDIO_PIN, AUDIO_MIDPOINT);
    bool started = add_repeating_timer_us(
        -((1000000 + (AUDIO_SAMPLE_RATE / 2)) / AUDIO_SAMPLE_RATE),
        audio_timer_callback,
        state,
        &audio_timer);
    if (!started)
        panic("Audio timer creation failed");
}

static void audio_start()
{
    audio_command_t command = AUDIO_COMMAND_START;
    xQueueSend(xQueueAudioCommand, &command, 0);
}

static void audio_toggle_pause()
{
    audio_command_t command = AUDIO_COMMAND_TOGGLE_PAUSE;
    xQueueSend(xQueueAudioCommand, &command, 0);
}

void audio_debug_task(void *p)
{
    instrumentation_init(PIN_INSTR_AUDIO_DEBUG);
    audio_state_t state = {0, false, false};
    audio_event_t event;
    audio_command_t report_command = AUDIO_COMMAND_REPORT_INDEX;
    TickType_t last_report = xTaskGetTickCount();

    audio_init(&state);

    while (1)
    {
        if (xQueueReceive(xQueueAudioEvent, &event, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            gpio_put(PIN_INSTR_AUDIO_DEBUG, 1);
            if (event.status != 0)
            {
                send_packet(AXIS_AUDIO_STATUS, event.status);
            }
            else
            {
                // Encode the full sample index as three base-95 digits because each
                // serial packet can carry only values from 0 through 94.
                send_packet(AXIS_AUDIO_INDEX_LOW, event.index % 95);
                send_packet(AXIS_AUDIO_INDEX_MID, (event.index / 95) % 95);
                send_packet(AXIS_AUDIO_INDEX_HIGH, (event.index / (95 * 95)) % 95);
            }
            gpio_put(PIN_INSTR_AUDIO_DEBUG, 0);
        }

        if (xTaskGetTickCount() - last_report >= pdMS_TO_TICKS(1000))
        {
            gpio_put(PIN_INSTR_AUDIO_DEBUG, 1);
            xQueueSend(xQueueAudioCommand, &report_command, 0);
            last_report = xTaskGetTickCount();
            gpio_put(PIN_INSTR_AUDIO_DEBUG, 0);
        }
    }
}

static bool mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[14];
    uint8_t val = 0x3B;
    xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    int ret = i2c_write_blocking(i2c0, MPU_ADDRESS, &val, 1, true);
    if (ret < 0)
    {
        xSemaphoreGive(xMutexI2C);
        return false;
    }
    ret = i2c_read_blocking(i2c0, MPU_ADDRESS, buffer, 14, false);
    xSemaphoreGive(xMutexI2C);
    if (ret != 14)
        return false;

    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)((buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1]);
    return true;
}

static int8_t clamp94(float v)
{
    if (v > MAX_ABS_VALUE)
        return MAX_ABS_VALUE;
    if (v < -MAX_ABS_VALUE)
        return -MAX_ABS_VALUE;
    return (int8_t)v;
}

static void send_packet(uint8_t axis, int8_t value)
{
    uint8_t encoded = (uint8_t)(value + VALUE_OFFSET);
    if (encoded == SYNC_BYTE)
        encoded = SYNC_BYTE - 1;
    uint8_t pkt[3] = {SYNC_BYTE, axis, encoded};
    // Serializa a UART: uart_task (X/Y/click) e button_task (rotacao/space)
    // escrevem na mesma serial; sem o mutex os 3 bytes poderiam intercalar.
    xSemaphoreTake(xMutexUart, portMAX_DELAY);
    for (int i = 0; i < 3; i++)
        putchar_raw(pkt[i]);
    xSemaphoreGive(xMutexUart);
}

static int8_t scale_percent(float value)
{
    return clamp94(value * MAX_ABS_VALUE);
}

static void require_handle(const void *handle)
{
    if (handle == NULL)
        panic("FreeRTOS object allocation failed");
}

static void require_task_created(BaseType_t result)
{
    if (result != pdPASS)
        panic("FreeRTOS task creation failed");
}

void mpu6050_task(void *p)
{
    instrumentation_init(PIN_INSTR_MPU);
    mpu6050_init();
    mpu_data_t data;
    int16_t temp;
    memset(&data, 0, sizeof(data));
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t paused_samples = 0;
    uint32_t read_errors = 0;
    while (1)
    {
        gpio_put(PIN_INSTR_MPU, 1);
        if (!mpu6050_read_raw(data.accel, data.gyro, &temp))
        {
            read_errors++;
            if ((read_errors % 10) == 1)
                send_packet(AXIS_IMU_READ_ERRORS, clamp94(read_errors));
            gpio_put(PIN_INSTR_MPU, 0);
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(11));
            continue;
        }
        xQueueSend(xQueueMPU, &data, 0);
        if (!gpio_get(BTN_PAUSE_PIN))
        {
            xQueueSend(xQueueAI, &data, 0);
            paused_samples++;
            if ((paused_samples % AI_INFERENCE_STRIDE_SAMPLES) == 0)
            {
                // Acceleration snapshots are reported in g * 20.
                send_packet(AXIS_DEBUG_ACCEL_X, clamp94(data.accel[0] / 819.2f));
                send_packet(AXIS_DEBUG_ACCEL_Y, clamp94(data.accel[1] / 819.2f));
                send_packet(AXIS_DEBUG_ACCEL_Z, clamp94(data.accel[2] / 819.2f));
                send_packet(AXIS_AI_QUEUE_DEPTH, clamp94(uxQueueMessagesWaiting(xQueueAI)));
            }
        }
        else
        {
            paused_samples = 0;
        }
        gpio_put(PIN_INSTR_MPU, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(11));
    }
}

void fusion_task(void *p)
{
    instrumentation_init(PIN_INSTR_FUSION);
    mpu_data_t raw;
    pos_data_t pos;
    color_data_t color;

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int warmup = 0;
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

    const float CLICK_THRESHOLD = 1.5f;
    const int CLICK_COOLDOWN = 30;
    int cooldown = 0;

    float sr = 0, sg = 0, sb = 0;
    const float ALPHA = 0.06f;

    // Inclinação → velocidade contínua do cursor
    const float ANGLE_DEAD_ZONE = 8.0f; // graus — sensor parado não move o cursor
    const float ANGLE_MAX = 25.0f;      // graus — inclinação máxima (velocidade máxima)

    while (1)
    {
        if (xQueueReceive(xQueueMPU, &raw, portMAX_DELAY) == pdTRUE)
        {

            gpio_put(PIN_INSTR_FUSION, 1);

            FusionVector gyroscope;
            gyroscope.axis.x = (raw.gyro[0] / 131.0f) - gyro_bias[0];
            gyroscope.axis.y = (raw.gyro[1] / 131.0f) - gyro_bias[1];
            gyroscope.axis.z = (raw.gyro[2] / 131.0f) - gyro_bias[2];

            FusionVector accelerometer;
            accelerometer.axis.x = raw.accel[0] / 16384.0f;
            accelerometer.axis.y = raw.accel[1] / 16384.0f;
            accelerometer.axis.z = raw.accel[2] / 16384.0f;

            // Clique: spike no eixo Y
            pos.click = false;
            if (cooldown > 0)
            {
                cooldown--;
                // Ignora aceleracao para nao corromper a matriz AHRS (FUSION zero = ignorar)
                accelerometer.axis.x = 0.0f;
                accelerometer.axis.y = 0.0f;
                accelerometer.axis.z = 0.0f;
            }
            else if (fabsf(accelerometer.axis.y) > CLICK_THRESHOLD)
            {
                pos.click = true;
                cooldown = CLICK_COOLDOWN;

                // Ignora aceleracao do solavanco inicial
                accelerometer.axis.x = 0.0f;
                accelerometer.axis.y = 0.0f;
                accelerometer.axis.z = 0.0f;
            }

            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

            if (warmup < WARMUP_SAMPLES)
            {
                gyro_bias[0] += raw.gyro[0] / 131.0f;
                gyro_bias[1] += raw.gyro[1] / 131.0f;
                gyro_bias[2] += raw.gyro[2] / 131.0f;
                warmup++;
                if (warmup == WARMUP_SAMPLES)
                {
                    gyro_bias[0] /= WARMUP_SAMPLES;
                    gyro_bias[1] /= WARMUP_SAMPLES;
                    gyro_bias[2] /= WARMUP_SAMPLES;
                }
                gpio_put(PIN_INSTR_FUSION, 0);
            }

            // Euler — usado tanto para movimento quanto para o LED
            const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
            float roll = euler.angle.roll;
            float pitch = euler.angle.pitch;

            // Inclinação vira velocidade: inclinado → cursor continua movendo
            float vx = (fabsf(roll) > ANGLE_DEAD_ZONE) ? roll : 0.0f;
            float vy = (fabsf(pitch) > ANGLE_DEAD_ZONE) ? pitch : 0.0f;

            pos.x = clamp94(vx / ANGLE_MAX * MAX_ABS_VALUE);
            pos.y = clamp94(-vy / ANGLE_MAX * MAX_ABS_VALUE);

            // LED: mesmos ângulos
            float tr = (roll > 5.0f) ? fminf(roll / SCALE_DEG, 1.0f) * 255.0f : 0.0f;
            float tb = (roll < -5.0f) ? fminf(-roll / SCALE_DEG, 1.0f) * 255.0f : 0.0f;
            float tg = (pitch < -5.0f) ? fminf(-pitch / SCALE_DEG, 1.0f) * 255.0f : 0.0f;

            sr = sr + ALPHA * (tr - sr);
            sg = sg + ALPHA * (tg - sg);
            sb = sb + ALPHA * (tb - sb);

            color.r = (uint8_t)sr;
            color.g = (uint8_t)sg;
            color.b = (uint8_t)sb;

            xQueueSend(xQueuePos, &pos, 0);
            xQueueSend(xQueueColor, &color, 0);
            gpio_put(PIN_INSTR_FUSION, 0);
        }
    }
}

void pwm_task(void *p)
{
    instrumentation_init(PIN_INSTR_PWM);
    color_data_t color;
    pwm_init_pin(LED_R_PIN);
    pwm_init_pin(LED_G_PIN);
    pwm_init_pin(LED_B_PIN);
    while (1)
    {
        if (xQueueReceive(xQueueColor, &color, portMAX_DELAY) == pdTRUE)
        {
            gpio_put(PIN_INSTR_PWM, 1);
            pwm_set_duty(LED_R_PIN, color.r);
            pwm_set_duty(LED_G_PIN, color.g);
            pwm_set_duty(LED_B_PIN, color.b);
            gpio_put(PIN_INSTR_PWM, 0);
        }
    }
}

// ISR unico dos 4 botoes. Mantido enxuto: filtra por timestamp e enfileira
// o GPIO que gerou a borda. O pause nao usa debounce por timestamp porque
// a task confirma o nivel real do pino depois; bordas extras de bounce so
// geram releituras do mesmo nivel, sem efeito colateral.
void btn_callback(uint gpio, uint32_t events)
{
    uint32_t now = time_us_32();

    if (gpio == BTN_ROT_R_PIN || gpio == BTN_ENTER_PIN || gpio == BTN_ESC_PIN)
    {
        static uint32_t last_us[32];
        if (now - last_us[gpio] < BTN_DEBOUNCE_US)
            return;
        last_us[gpio] = now;
    }
    else if (gpio != BTN_PAUSE_PIN)
    {
        return;
    }

    uint8_t pin = (uint8_t)gpio;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xQueueBtn, &pin, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void button_task(void *p)
{
    instrumentation_init(PIN_INSTR_BUTTON);
    const uint pins[] = {BTN_ROT_R_PIN, BTN_ENTER_PIN, BTN_ESC_PIN};
    for (size_t i = 0; i < sizeof(pins) / sizeof(pins[0]); i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    // Pause (BTN_PAUSE_PIN) so precisa do pull-up: seu estado e lido direto
    // pelo uart_task. So os botoes de evento usam IRQ.
    gpio_set_irq_enabled_with_callback(BTN_ROT_R_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled(BTN_ENTER_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_ESC_PIN, GPIO_IRQ_EDGE_FALL, true);

    uint8_t pin;
    while (1)
    {
        if (xQueueReceive(xQueueBtn, &pin, portMAX_DELAY) == pdTRUE)
        {
            gpio_put(PIN_INSTR_BUTTON, 1);
            // Confirma o nivel apos a borda: bounce ao soltar gera borda de
            // descida falsa, mas 5ms depois o pino ja esta alto e descarta.
            vTaskDelay(pdMS_TO_TICKS(5));
            if (!gpio_get(pin))
            {
                if (pin == BTN_ENTER_PIN)
                    audio_start();
                else if (pin == BTN_ESC_PIN)
                    audio_toggle_pause();

                uint8_t axis = (pin == BTN_ROT_R_PIN)   ? AXIS_ROT_R
                               : (pin == BTN_ENTER_PIN) ? AXIS_ENTER
                                                        : AXIS_ESC;
                send_packet(axis, 0);
            }
            gpio_put(PIN_INSTR_BUTTON, 0);
        }
    }
}

void ai_task(void *p)
{
    instrumentation_init(PIN_INSTR_AI);
    mpu_data_t raw;
    static float ring[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
    static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
    const int frame_values = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    int write_index = 0;
    int collected_samples = 0;
    int samples_since_inference = 0;
    bool pause_was_active = false;

    send_packet(AXIS_AI_STATUS, AI_STATUS_TASK_STARTED);

    while (1)
    {
        if (gpio_get(BTN_PAUSE_PIN))
        {
            if (pause_was_active)
                send_packet(AXIS_AI_STATUS, AI_STATUS_PAUSE_RELEASED);
            xQueueReset(xQueueAI);
            write_index = 0;
            collected_samples = 0;
            samples_since_inference = 0;
            pause_was_active = false;
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (!pause_was_active)
        {
            send_packet(AXIS_AI_STATUS, AI_STATUS_PAUSE_ACTIVE);
            pause_was_active = true;
        }

        if (xQueueReceive(xQueueAI, &raw, pdMS_TO_TICKS(20)) != pdTRUE)
            continue;

        gpio_put(PIN_INSTR_AI, 1);

        // This model was trained with raw MPU6050 accelerometer counts.
        ring[write_index + 0] = raw.accel[0];
        ring[write_index + 1] = raw.accel[1];
        ring[write_index + 2] = raw.accel[2];
        write_index = (write_index + EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME) % frame_values;

        if (collected_samples < EI_CLASSIFIER_RAW_SAMPLE_COUNT)
            collected_samples++;
        samples_since_inference++;

        if ((collected_samples % AI_INFERENCE_STRIDE_SAMPLES) == 0)
            send_packet(AXIS_AI_PROGRESS,
                        scale_percent((float)collected_samples / EI_CLASSIFIER_RAW_SAMPLE_COUNT));

        if (collected_samples < EI_CLASSIFIER_RAW_SAMPLE_COUNT ||
            samples_since_inference < AI_INFERENCE_STRIDE_SAMPLES)
        {
            gpio_put(PIN_INSTR_AI, 0);
            continue;
        }

        samples_since_inference = 0;
        for (int ix = 0; ix < frame_values; ix++)
            inference_buffer[ix] = ring[(write_index + ix) % frame_values];
        send_packet(AXIS_AI_STATUS, AI_STATUS_WINDOW_READY);

        ei::signal_t signal;
        if (ei::numpy::signal_from_buffer(inference_buffer, frame_values, &signal) != 0)
        {
            gpio_put(PIN_INSTR_AI, 0);
            continue;
        }

        ei_impulse_result_t result = {0};
        EI_IMPULSE_ERROR classifier_error = run_classifier(&signal, &result, false);
        if (classifier_error != EI_IMPULSE_OK)
        {
            send_packet(AXIS_AI_STATUS, AI_STATUS_CLASSIFIER_ERROR);
            send_packet(AXIS_AI_ERROR_CODE, clamp94(classifier_error));
            gpio_put(PIN_INSTR_AI, 0);
            continue;
        }

        send_packet(AXIS_AI_DSP_MS, clamp94(result.timing.dsp));
        send_packet(AXIS_AI_CLASSIFICATION_MS, clamp94(result.timing.classification));
        send_packet(AXIS_AI_ANOMALY, clamp94(result.anomaly * 10.0f));

        for (int ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
        {
            if (strcmp(result.classification[ix].label, "idle") == 0)
            {
                send_packet(AXIS_AI_IDLE_CONFIDENCE,
                            scale_percent(result.classification[ix].value));
                continue;
            }

            if (strcmp(result.classification[ix].label, AI_GESTURE_LABEL) != 0)
                continue;

            send_packet(AXIS_AI_CONFIDENCE,
                        scale_percent(result.classification[ix].value));

            if (result.classification[ix].value >= AI_GESTURE_THRESHOLD)
            {
                send_packet(AXIS_AI_STATUS, AI_STATUS_TETRIS_DETECTED);
                send_packet(AXIS_SPACE, 0);
                // Drop samples accumulated during the cooldown. Replaying them
                // quickly would destroy the model's real-time sample spacing.
                gpio_put(PIN_INSTR_AI, 0);
                vTaskDelay(pdMS_TO_TICKS(AI_GESTURE_COOLDOWN_MS));
                xQueueReset(xQueueAI);
                write_index = 0;
                collected_samples = 0;
                samples_since_inference = 0;
                break;
            }
        }
        gpio_put(PIN_INSTR_AI, 0);
    }
}

void uart_task(void *p)
{
    instrumentation_init(PIN_INSTR_UART);
    pos_data_t pos;
    while (1)
    {
        if (xQueueReceive(xQueuePos, &pos, portMAX_DELAY) == pdTRUE)
        {
            // Pause do IMU: le o nivel do pino direto. Pino baixo = segurando
            // = pausado. Sem flag intermediario, sem borda, sem race: o estado
            // do pause E o nivel do GPIO no instante do envio.
            if (!gpio_get(BTN_PAUSE_PIN))
                continue;

            gpio_put(PIN_INSTR_UART, 1);
            send_packet(AXIS_X, pos.x);
            send_packet(AXIS_Y, pos.y);
            if (pos.click)
                send_packet(AXIS_CLICK, 0);
            gpio_put(PIN_INSTR_UART, 0);
        }
    }
}

int main()
{
    stdio_init_all();

    // Pause: input com pull-up, lido direto pelo uart_task. Inicializado aqui
    // (antes do scheduler) para estar pronto antes da primeira leitura.
    gpio_init(BTN_PAUSE_PIN);
    gpio_set_dir(BTN_PAUSE_PIN, GPIO_IN);
    gpio_pull_up(BTN_PAUSE_PIN);
    xQueueMPU = xQueueCreate(32, sizeof(mpu_data_t));
    xQueueAI = xQueueCreate(128, sizeof(mpu_data_t));
    xQueuePos = xQueueCreate(32, sizeof(pos_data_t));
    xQueueColor = xQueueCreate(32, sizeof(color_data_t));
    xQueueBtn = xQueueCreate(8, sizeof(uint8_t));
    xQueueAudioCommand = xQueueCreate(8, sizeof(audio_command_t));
    xQueueAudioEvent = xQueueCreate(8, sizeof(audio_event_t));
    xMutexUart = xSemaphoreCreateMutex();
    xMutexI2C = xSemaphoreCreateMutex();

    require_handle(xQueueMPU);
    require_handle(xQueueAI);
    require_handle(xQueuePos);
    require_handle(xQueueColor);
    require_handle(xQueueBtn);
    require_handle(xQueueAudioCommand);
    require_handle(xQueueAudioEvent);
    require_handle(xMutexUart);
    require_handle(xMutexI2C);

    TaskHandle_t h_mpu = NULL;
    TaskHandle_t h_fusion = NULL;
    TaskHandle_t h_pwm = NULL;
    TaskHandle_t h_uart = NULL;
    TaskHandle_t h_btn = NULL;
    TaskHandle_t h_ai = NULL;
    TaskHandle_t h_audio_debug = NULL;

    require_task_created(xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 3, &h_mpu));
    require_task_created(xTaskCreate(fusion_task, "fusion_Task", 8192, NULL, 3, &h_fusion));
    require_task_created(xTaskCreate(pwm_task, "pwm_Task", 1024, NULL, 1, &h_pwm));
    require_task_created(xTaskCreate(uart_task, "uart_Task", 2048, NULL, 2, &h_uart));
    require_task_created(xTaskCreate(button_task, "button_Task", 2048, NULL, 2, &h_btn));
    require_task_created(xTaskCreate(ai_task, "ai_Task", 16384, NULL, 2, &h_ai));
    require_task_created(xTaskCreate(audio_debug_task, "audio_debug_Task", 1024, NULL, 1, &h_audio_debug));

    vTaskCoreAffinitySet(h_mpu, CORE_0);
    vTaskCoreAffinitySet(h_fusion, CORE_1);
    vTaskCoreAffinitySet(h_pwm, CORE_1);
    vTaskCoreAffinitySet(h_uart, CORE_1);
    vTaskCoreAffinitySet(h_btn, CORE_1);
    vTaskCoreAffinitySet(h_ai, CORE_0);
    vTaskCoreAffinitySet(h_audio_debug, CORE_1);

    vTaskStartScheduler();
    while (true)
        ;
}
