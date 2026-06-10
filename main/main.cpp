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

// Edge Impulse: classificador de gestos. Roda no modo gesto (pause segurado).
// ei_run_classifier.h ja define run_classifier() (com corpo) dentro do
// namespace ei, entao basta inclui-lo; nao redeclarar.
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

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

#define BTN_ROT_R_PIN 4 // gira direita -> seta cima
#define BTN_ROT_L_PIN 5 // gira esquerda -> Z
#define BTN_PAUSE_PIN 2 // segurado -> pausa o IMU
#define BTN_SPACE_PIN 3 // tecla space
#define BTN_DEBOUNCE_US 20000

#define CORE_0 (1 << 0)
#define CORE_1 (1 << 1)

#define SYNC_BYTE 0xFF
#define VALUE_OFFSET 128
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_CLICK 2
#define AXIS_ROT_R 3
#define AXIS_ROT_L 4
#define AXIS_SPACE 5
#define MAX_ABS_VALUE 94

// Gesto: confianca minima para aceitar a classe "tetris" e disparar o hard drop.
#define GESTURE_THRESHOLD 0.7f
// Cooldown apos um hard drop por gesto, em ciclos de janela, para nao repetir.
#define GESTURE_COOLDOWN_MS 600

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
QueueHandle_t xQueueBtn;
SemaphoreHandle_t xMutexUart;
// Serializa o acesso I2C ao MPU6050 entre o leitor do cursor e o leitor de
// gestos, que nunca rodam juntos (pause alterna entre eles) mas compartilham
// o barramento.
SemaphoreHandle_t xMutexI2C;

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
    xSemaphoreTake(xMutexI2C, portMAX_DELAY);
    int ret = i2c_write_blocking(i2c0, MPU_ADDRESS, &val, 1, true);
    if (ret < 0)
    {
        xSemaphoreGive(xMutexI2C);
        return;
    }
    i2c_read_blocking(i2c0, MPU_ADDRESS, buffer, 14, false);
    xSemaphoreGive(xMutexI2C);
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
    // Serializa a UART: uart_task (X/Y/click) e button_task (rotacao/space)
    // escrevem na mesma serial; sem o mutex os 3 bytes poderiam intercalar.
    xSemaphoreTake(xMutexUart, portMAX_DELAY);
    for (int i = 0; i < 3; i++)
        putchar_raw(pkt[i]);
    xSemaphoreGive(xMutexUart);
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
        // Modo gesto (pause segurado): a gesture_task assume o sensor. O cursor
        // ja esta congelado, entao parar de alimentar a fusion nao tem efeito
        // visivel e evita intercalar leituras na janela do classificador.
        if (!gpio_get(BTN_PAUSE_PIN))
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
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

            // Inicializacao campo a campo: designated initializers aninhados
            // (.axis.x) nao compilam de forma portavel em C++.
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

// ISR unico dos 4 botoes. Mantido enxuto: filtra por timestamp e enfileira
// o GPIO que gerou a borda. O pause nao usa debounce por timestamp porque
// a task confirma o nivel real do pino depois; bordas extras de bounce so
// geram releituras do mesmo nivel, sem efeito colateral.
void btn_callback(uint gpio, uint32_t events)
{
    static uint32_t last_us[32];
    uint32_t now = time_us_32();

    if (gpio == BTN_ROT_R_PIN || gpio == BTN_ROT_L_PIN || gpio == BTN_SPACE_PIN)
    {
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
    const uint pins[] = {BTN_ROT_R_PIN, BTN_ROT_L_PIN, BTN_SPACE_PIN};
    for (int i = 0; i < 3; i++)
    {
        gpio_init(pins[i]);
        gpio_set_dir(pins[i], GPIO_IN);
        gpio_pull_up(pins[i]);
    }

    // Pause (BTN_PAUSE_PIN) so precisa do pull-up: seu estado e lido direto
    // pelo uart_task. So os botoes de evento usam IRQ.
    gpio_set_irq_enabled_with_callback(BTN_ROT_R_PIN, GPIO_IRQ_EDGE_FALL, true, &btn_callback);
    gpio_set_irq_enabled(BTN_ROT_L_PIN, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(BTN_SPACE_PIN, GPIO_IRQ_EDGE_FALL, true);

    uint8_t pin;
    while (1)
    {
        if (xQueueReceive(xQueueBtn, &pin, portMAX_DELAY) == pdTRUE)
        {
            // Confirma o nivel apos a borda: bounce ao soltar gera borda de
            // descida falsa, mas 5ms depois o pino ja esta alto e descarta.
            vTaskDelay(pdMS_TO_TICKS(5));
            if (!gpio_get(pin))
            {
                uint8_t axis = (pin == BTN_ROT_R_PIN)   ? AXIS_ROT_R
                               : (pin == BTN_ROT_L_PIN) ? AXIS_ROT_L
                                                        : AXIS_SPACE;
                send_packet(axis, 0);
            }
        }
    }
}

// Modo gesto: ativo enquanto BTN_PAUSE_PIN esta segurado (pino baixo). Nesse
// estado o cursor ja esta congelado (uart_task nao envia), entao o sensor fica
// livre para alimentar o classificador. Acumula uma janela de acelerometro,
// roda a inferencia e, se a classe "tetris" passar do limiar, dispara o hard
// drop (mesma acao do botao de space). Fora do modo gesto, dorme.
void gesture_task(void *p)
{
    int16_t accel[3], gyro[3], temp;
    const int frame = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; // 91 * 3
    static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

    while (1)
    {
        // So coleta quando o pause esta segurado. Fora disso, espera.
        if (gpio_get(BTN_PAUSE_PIN))
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // Acumula a janela: 91 amostras de 3 eixos do acelerometro, ~11ms cada.
        for (int ix = 0; ix < frame; ix += 3)
        {
            mpu6050_read_raw(accel, gyro, &temp);
            buffer[ix + 0] = accel[0];
            buffer[ix + 1] = accel[1];
            buffer[ix + 2] = accel[2];
            vTaskDelay(pdMS_TO_TICKS(10));

            // Pause soltou no meio da coleta: aborta a janela.
            if (gpio_get(BTN_PAUSE_PIN))
                break;
        }
        if (gpio_get(BTN_PAUSE_PIN))
            continue;

        ei::signal_t signal;
        if (ei::numpy::signal_from_buffer(buffer, frame, &signal) != 0)
            continue;

        ei_impulse_result_t result = {0};
        // run_classifier() vive no namespace anonimo do header (extern "C"),
        // nao em ei::. Chamada qualificada com ei:: liga a uma declaracao
        // externa sem definicao -> "undefined reference". Chamar sem ei::.
        if (run_classifier(&signal, &result, false) != EI_IMPULSE_OK)
            continue;

        // Escolhe a classe de maior probabilidade.
        int best = 0;
        for (int ix = 1; ix < EI_CLASSIFIER_LABEL_COUNT; ix++)
            if (result.classification[ix].value > result.classification[best].value)
                best = ix;

        const char *label = result.classification[best].label;
        float conf = result.classification[best].value;

        if (strcmp(label, "tetris") == 0 && conf >= GESTURE_THRESHOLD)
        {
            send_packet(AXIS_SPACE, 0);
            // Cooldown: evita repetir o hard drop com o mesmo gesto.
            vTaskDelay(pdMS_TO_TICKS(GESTURE_COOLDOWN_MS));
        }
    }
}

void uart_task(void *p)
{
    gpio_init(PIN_INSTR_UART);
    gpio_set_dir(PIN_INSTR_UART, GPIO_OUT);
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
    xQueuePos = xQueueCreate(32, sizeof(pos_data_t));
    xQueueColor = xQueueCreate(32, sizeof(color_data_t));
    xQueueBtn = xQueueCreate(8, sizeof(uint8_t));
    xMutexUart = xSemaphoreCreateMutex();
    xMutexI2C = xSemaphoreCreateMutex();

    TaskHandle_t h_mpu, h_fusion, h_pwm, h_uart, h_btn, h_gesture;

    xTaskCreate(mpu6050_task, "mpu6050_Task", 8192, NULL, 3, &h_mpu);
    xTaskCreate(fusion_task, "fusion_Task", 8192, NULL, 3, &h_fusion);
    xTaskCreate(pwm_task, "pwm_Task", 1024, NULL, 1, &h_pwm);
    xTaskCreate(uart_task, "uart_Task", 2048, NULL, 2, &h_uart);
    xTaskCreate(button_task, "button_Task", 2048, NULL, 2, &h_btn);
    // Inferencia: stack grande (o classificador consome bastante). No core 0,
    // junto do mpu_task, que dorme durante o modo gesto.
    xTaskCreate(gesture_task, "gesture_Task", 16384, NULL, 2, &h_gesture);

    vTaskCoreAffinitySet(h_mpu, CORE_0);
    vTaskCoreAffinitySet(h_fusion, CORE_1);
    vTaskCoreAffinitySet(h_pwm, CORE_1);
    vTaskCoreAffinitySet(h_uart, CORE_1);
    vTaskCoreAffinitySet(h_btn, CORE_1);
    vTaskCoreAffinitySet(h_gesture, CORE_0);

    vTaskStartScheduler();
    while (true)
        ;
}