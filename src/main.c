#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// ======================
// Definições de Pinos
// ======================

// Sensores digitais
#define SENSOR_D_DIR   GPIO_NUM_32
#define SENSOR_D_ESQ   GPIO_NUM_17

// Motores (Ponte H)
#define MOTOR_A_IN1    GPIO_NUM_25
#define MOTOR_A_IN2    GPIO_NUM_26

#define MOTOR_B_IN3    GPIO_NUM_33
#define MOTOR_B_IN4    GPIO_NUM_27

#define MOTOR_A_ENA    GPIO_NUM_14 // PWM
#define MOTOR_B_ENB    GPIO_NUM_13 // PWM

// ======================
// Configuração PWM
// ======================
#define LEDC_MODE          LEDC_HIGH_SPEED_MODE
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_FREQ_HZ       5000
#define LEDC_RESOLUTION    LEDC_TIMER_8_BIT

#define MOTOR_A_CHANNEL    LEDC_CHANNEL_0
#define MOTOR_B_CHANNEL    LEDC_CHANNEL_1

// ======================
// Setup GPIO
// ======================
void setup_gpio() {
    // Sensores como entrada
    gpio_set_direction(SENSOR_D_ESQ, GPIO_MODE_INPUT);
    gpio_set_direction(SENSOR_D_DIR, GPIO_MODE_INPUT);

    // Motores como saída
    gpio_set_direction(MOTOR_A_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_A_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOTOR_B_IN4, GPIO_MODE_OUTPUT);
}

// ======================
// Setup PWM
// ======================
void setup_pwm() {
    ledc_timer_config_t timer_conf = {
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t motorA = {
        .channel    = MOTOR_A_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_A_ENA,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&motorA);

    ledc_channel_config_t motorB = {
        .channel    = MOTOR_B_CHANNEL,
        .duty       = 0,
        .gpio_num   = MOTOR_B_ENB,
        .speed_mode = LEDC_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER
    };
    ledc_channel_config(&motorB);
}

// ======================
// Funções auxiliares
// ======================
void set_motor(int motor, int forward, int speed) {
    if (motor == 0) {
        // Motor A - invertido para corrigir a direção
        gpio_set_level(MOTOR_A_IN1, forward ? 0 : 1);
        gpio_set_level(MOTOR_A_IN2, forward ? 1 : 0);
        ledc_set_duty(LEDC_MODE, MOTOR_A_CHANNEL, speed);
        ledc_update_duty(LEDC_MODE, MOTOR_A_CHANNEL);
    } else {
        // Motor B - invertido para corrigir a direção
        gpio_set_level(MOTOR_B_IN3, forward ? 0 : 1);
        gpio_set_level(MOTOR_B_IN4, forward ? 1 : 0);
        ledc_set_duty(LEDC_MODE, MOTOR_B_CHANNEL, speed);
        ledc_update_duty(LEDC_MODE, MOTOR_B_CHANNEL);
    }
}

void andar_frente(int vel) {
    set_motor(0, 1, vel);
    set_motor(1, 1, vel);
}

void virar_esquerda(int vel) {
    set_motor(0, 0, vel); // Motor A ré
    set_motor(1, 1, vel); // Motor B frente
}

void virar_direita(int vel) {
    set_motor(0, 1, vel); // Motor A frente
    set_motor(1, 0, vel); // Motor B ré
}

void parar() {
    set_motor(0, 1, 0);
    set_motor(1, 1, 0);
}

static bool recovering = false;       // indica que estamos executando a manobra de recuperação
// static bool last_both_1 = false;      // para detectar borda de entrada em (1,1)

void trotada_recuperacao(int vel) {
    // Pequena manobra para “sair” da condição 1/1
    // Ajuste as durações conforme seu robô
    parar();
    vTaskDelay(pdMS_TO_TICKS(50));

    // andar_frente(vel*1.5);
    // vTaskDelay(pdMS_TO_TICKS(150));

    // 1) Dar uma “trotada” curta para trás e girar
    set_motor(0, 0, vel); // A ré
    set_motor(1, 0, vel); // B ré
    vTaskDelay(pdMS_TO_TICKS(300));   // recua um pouquinho

    // 2) Girar à esquerda um pouco
    set_motor(0, 0, vel); // A ré
    set_motor(1, 1, vel); // B frente
    vTaskDelay(pdMS_TO_TICKS(100));

    // 3) Parar
    parar();
    vTaskDelay(pdMS_TO_TICKS(20));
}

// ======================
// Loop principal
// ======================
void app_main(void) {
    setup_gpio();
    setup_pwm();

    int velocidade = 110; // Velocidade

    while (1) {
        int esq = gpio_get_level(SENSOR_D_ESQ);
        int dir = gpio_get_level(SENSOR_D_DIR);

        bool both_1 = (esq == 1 && dir == 1);
        // if (both_1 && !last_both_1 && !recovering) {
        if (both_1 && !recovering) {
            // Entrou em (1,1) agora → executa a manobra uma única vez
            recovering = true;
            trotada_recuperacao(velocidade);
            recovering = false;
        }
        // last_both_1 = both_1;

        if (!recovering) {
            if (esq == 0 && dir == 0) {
                andar_frente(velocidade);
            } else if (esq == 1 && dir == 0) {
                // Esquerdo fora da linha → vira esquerda
                virar_esquerda(velocidade);
            } else if (esq == 0 && dir == 1) {
                // Direito fora da linha → vira direita
                virar_direita(velocidade);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
