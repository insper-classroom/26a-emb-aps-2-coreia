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

#define SAMPLE_PERIOD (0.01f)
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

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define SYNC_BYTE 0xFF
#define VALUE_OFFSET 128
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_CLICK 2
#define MAX_ABS_VALUE 94

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

QueueHandle_t xQueueMPU;
QueueHandle_t xQueuePos;
QueueHandle_t xQueueColor;
SemaphoreHandle_t xSemaphoreBtn;

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

    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c0, MPU_ADDRESS, buf, 2, false);
    vTaskDelay(pdMS_TO_TICKS(100));
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp)
{
    uint8_t buffer[14];
    uint8_t val = 0x3B;
    int ret = i2c_write_blocking(i2c0, MPU_ADDRESS, &val, 1, true);
    if (ret < 0)
        return;
    i2c_read_blocking(i2c0, MPU_ADDRESS, buffer, 14, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)((buffer[i * 2] << 8) | buffer[i * 2 + 1]);
    *temp = (int16_t)((buffer[6] << 8) | buffer[7]);
    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)((buffer[8 + i * 2] << 8) | buffer[8 + i * 2 + 1]);
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
    for (int i = 0; i < 3; i++)
        putchar_raw(pkt[i]);
}

void mpu6050_task(void *p)
{
    gpio_init(PIN_INSTR_MPU);
    gpio_set_dir(PIN_INSTR_MPU, GPIO_OUT);
    mpu6050_init();
    mpu_data_t data;
    int16_t temp;
    memset(&data, 0, sizeof(data));
    while (1)
    {
        gpio_put(PIN_INSTR_MPU, 1);
        mpu6050_read_raw(data.accel, data.gyro, &temp);
        xQueueSend(xQueueMPU, &data, 0);
        gpio_put(PIN_INSTR_MPU, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void fusion_task(void *p)
{
    gpio_init(PIN_INSTR_FUSION);
    gpio_set_dir(PIN_INSTR_FUSION, GPIO_OUT);
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

            FusionVector gyroscope = {
                .axis.x = (raw.gyro[0] / 131.0f) - gyro_bias[0],
                .axis.y = (raw.gyro[1] / 131.0f) - gyro_bias[1],
                .axis.z = (raw.gyro[2] / 131.0f) - gyro_bias[2],
            };
            FusionVector accelerometer = {
                .axis.x = raw.accel[0] / 16384.0f,
                .axis.y = raw.accel[1] / 16384.0f,
                .axis.z = raw.accel[2] / 16384.0f,
            };

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
                xSemaphoreGive(xSemaphoreBtn);

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
    gpio_init(PIN_INSTR_PWM);
    gpio_set_dir(PIN_INSTR_PWM, GPIO_OUT);
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

// void stack_monitor_task(void *p)
// {
//     static TaskStatus_t tasks[16];
//     while (true)
//     {
//         vTaskDelay(pdMS_TO_TICKS(2000));
//         UBaseType_t n = uxTaskGetSystemState(tasks, 16, NULL);
//         printf("+------------------+-------+\n");
//         printf("| %-16s | %5s |\n", "task", "free");
//         printf("+------------------+-------+\n");
//         for (UBaseType_t i = 0; i < n; i++)
//         {
//             printf("| %-16s | %5u |\n",
//                    tasks[i].pcTaskName,
//                    (unsigned)tasks[i].usStackHighWaterMark);
//         }
//         printf("+------------------+-------+\n");
//         printf("| heap livre min   | %5u |\n",
//                (unsigned)xPortGetMinimumEverFreeHeapSize());
//         printf("+------------------+-------+\n\n");
//     }
// }

void uart_task(void *p)
{
    gpio_init(PIN_INSTR_UART);
    gpio_set_dir(PIN_INSTR_UART, GPIO_OUT);
    pos_data_t pos;
    while (1)
    {
        if (xQueueReceive(xQueuePos, &pos, portMAX_DELAY) == pdTRUE)
        {
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

    xQueueMPU = xQueueCreate(32, sizeof(mpu_data_t));
    xQueuePos = xQueueCreate(32, sizeof(pos_data_t));
    xQueueColor = xQueueCreate(32, sizeof(color_data_t));
    xSemaphoreBtn = xSemaphoreCreateBinary();

    TaskHandle_t h_mpu, h_fusion, h_pwm, h_uart;

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 3, &h_mpu);
    xTaskCreate(fusion_task, "fusion_Task", 8192, NULL, 3, &h_fusion);
    xTaskCreate(pwm_task, "pwm_Task", 1024, NULL, 1, &h_pwm);
    xTaskCreate(uart_task, "uart_Task", 2048, NULL, 2, &h_uart);
    // xTaskCreate(stack_monitor_task, "stack_monitor", 512, NULL, 1, &h_monitor);

    vTaskCoreAffinitySet(h_mpu, CORE_0);
    vTaskCoreAffinitySet(h_fusion, CORE_1);
    vTaskCoreAffinitySet(h_pwm, CORE_1);
    vTaskCoreAffinitySet(h_uart, CORE_1);
    // vTaskCoreAffinitySet(h_monitor, CORE_1);

    vTaskStartScheduler();
    while (true)
        ;
}