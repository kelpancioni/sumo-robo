#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"

// Definições dos pinos - CONSISTENTE
#define SENSOR_LUZ_FRONTAL_PIN    ADC_CHANNEL_4  // GPIO32
#define SENSOR_LUZ_TRASEIRO_PIN   ADC_CHANNEL_6  // GPIO34

#define ULTRASONIC_TRIG_PIN       GPIO_NUM_5
#define ULTRASONIC_ECHO_PIN       GPIO_NUM_18
                                                                                                                                                                                                                                                                                                                                                            
// Motores - NOMES CONSISTENTES
#define MOTOR_ESQ_PWM_PIN         GPIO_NUM_14    // PWM motor esquerdo
#define MOTOR_ESQ_DIR1_PIN        GPIO_NUM_25    // DIR1 motor esquerdo
#define MOTOR_ESQ_DIR2_PIN        GPIO_NUM_26    // DIR2 motor esquerdo

#define MOTOR_DIR_PWM_PIN         GPIO_NUM_13    // PWM motor direito
#define MOTOR_DIR_DIR1_PIN        GPIO_NUM_33    // DIR1 motor direito
#define MOTOR_DIR_DIR2_PIN        GPIO_NUM_27    // DIR2 motor direito

// Constantes
#define LIMITE_LINHA_BRANCA       2000
#define DISTANCIA_ATAQUE          30
#define VELOCIDADE_MAXIMA         180
#define VELOCIDADE_BUSCA          100
#define VELOCIDADE_RECUO          140

// Configuração PWM
#define LEDC_TIMER                LEDC_TIMER_0
#define LEDC_MODE                 LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_ESQ          LEDC_CHANNEL_0  // Canal motor esquerdo
#define LEDC_CHANNEL_DIR          LEDC_CHANNEL_1  // Canal motor direito
#define LEDC_DUTY_RES             LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY            1000

static const char *TAG = "ROBO_SUMO";

// Handle do ADC
adc_oneshot_unit_handle_t adc1_handle;

// Estados do robô
typedef enum {
    ESTADO_BUSCA,
    ESTADO_ATAQUE,
    ESTADO_RECUO,
    ESTADO_GIRO
} estado_robo_t;

estado_robo_t estado_atual = ESTADO_BUSCA;

// Função para configurar GPIO
void configurar_gpio() {
    // Configurar pinos do ultrassônico
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRASONIC_TRIG_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ULTRASONIC_ECHO_PIN);
    gpio_config(&io_conf);

    // Configurar pinos dos motores - CORRIGIDO
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << MOTOR_ESQ_DIR1_PIN) | 
                           (1ULL << MOTOR_ESQ_DIR2_PIN) |
                           (1ULL << MOTOR_DIR_DIR1_PIN) | 
                           (1ULL << MOTOR_DIR_DIR2_PIN);
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
}

// Função para configurar ADC
void configurar_adc() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_LUZ_FRONTAL_PIN, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SENSOR_LUZ_TRASEIRO_PIN, &config));
}

// Função para configurar PWM
void configurar_pwm() {
    // Configurar timer
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    // Configurar canal motor esquerdo
    ledc_channel_config_t ledc_channel_esq = {
        .channel = LEDC_CHANNEL_ESQ,
        .duty = 0,
        .gpio_num = MOTOR_ESQ_PWM_PIN,
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&ledc_channel_esq);

    // Configurar canal motor direito
    ledc_channel_config_t ledc_channel_dir = {
        .channel = LEDC_CHANNEL_DIR,
        .duty = 0,
        .gpio_num = MOTOR_DIR_PWM_PIN,
        .speed_mode = LEDC_MODE,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE
    };
    ledc_channel_config(&ledc_channel_dir);
}

// Função para controlar motor esquerdo
void motor_esquerdo(int velocidade, int direcao) {
    if (direcao > 0) { // Frente
        gpio_set_level(MOTOR_ESQ_DIR1_PIN, 1);
        gpio_set_level(MOTOR_ESQ_DIR2_PIN, 0);
    } else if (direcao < 0) { // Trás
        gpio_set_level(MOTOR_ESQ_DIR1_PIN, 0);
        gpio_set_level(MOTOR_ESQ_DIR2_PIN, 1);
    } else { // Parar
        gpio_set_level(MOTOR_ESQ_DIR1_PIN, 0);
        gpio_set_level(MOTOR_ESQ_DIR2_PIN, 0);
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_ESQ, abs(velocidade));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_ESQ);
}

// Função para controlar motor direito
void motor_direito(int velocidade, int direcao) {
    if (direcao > 0) { // Frente
        gpio_set_level(MOTOR_DIR_DIR1_PIN, 1);
        gpio_set_level(MOTOR_DIR_DIR2_PIN, 0);
    } else if (direcao < 0) { // Trás
        gpio_set_level(MOTOR_DIR_DIR1_PIN, 0);
        gpio_set_level(MOTOR_DIR_DIR2_PIN, 1);
    } else { // Parar
        gpio_set_level(MOTOR_DIR_DIR1_PIN, 0);
        gpio_set_level(MOTOR_DIR_DIR2_PIN, 0);
    }
    
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_DIR, abs(velocidade));
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_DIR);
}

// Função para mover o robô
void mover_robo(int vel_esq, int vel_dir) {
    motor_esquerdo(abs(vel_esq), vel_esq >= 0 ? 1 : -1);
    motor_direito(abs(vel_dir), vel_dir >= 0 ? 1 : -1);
}

// Função para parar o robô
void parar_robo() {
    motor_esquerdo(0, 0);
    motor_direito(0, 0);
}

// Função para ler sensor de luz
int ler_sensor_luz(adc_channel_t canal) {
    int adc_raw;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, canal, &adc_raw));
    return adc_raw;
}

// Função para medir distância com ultrassônico
float medir_distancia() {
    // Enviar pulso trigger
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
    gpio_set_level(ULTRASONIC_TRIG_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(ULTRASONIC_TRIG_PIN, 0);

    // Medir tempo do echo
    int64_t tempo_inicio = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 0) {
        if ((esp_timer_get_time() - tempo_inicio) > 30000) return -1; // Timeout
    }
    
    tempo_inicio = esp_timer_get_time();
    while (gpio_get_level(ULTRASONIC_ECHO_PIN) == 1) {
        if ((esp_timer_get_time() - tempo_inicio) > 30000) return -1; // Timeout
    }
    
    int64_t duracao = esp_timer_get_time() - tempo_inicio;
    float distancia = (duracao * 0.034) / 2; // Converter para cm
    
    return distancia;
}

// Função principal de controle do robô
void controlar_robo() {
    int sensor_frontal = ler_sensor_luz(SENSOR_LUZ_FRONTAL_PIN);
    int sensor_traseiro = ler_sensor_luz(SENSOR_LUZ_TRASEIRO_PIN);
    float distancia = medir_distancia();
    
    ESP_LOGI(TAG, "Sensor frontal: %d, Sensor traseiro: %d, Distância: %.2f cm", 
             sensor_frontal, sensor_traseiro, distancia);

    // Verificar se detectou linha branca (prioridade máxima)
    if (sensor_frontal > LIMITE_LINHA_BRANCA) {
        ESP_LOGI(TAG, "Linha branca detectada na frente - RECUANDO");
        estado_atual = ESTADO_RECUO;
        mover_robo(-VELOCIDADE_RECUO, -VELOCIDADE_RECUO); // Recuar
        vTaskDelay(pdMS_TO_TICKS(300));
        mover_robo(-VELOCIDADE_BUSCA, VELOCIDADE_BUSCA); // Girar para direita
        vTaskDelay(pdMS_TO_TICKS(200));
        return;
    }
    
    if (sensor_traseiro > LIMITE_LINHA_BRANCA) {
        ESP_LOGI(TAG, "Linha branca detectada atrás - AVANÇANDO");
        estado_atual = ESTADO_ATAQUE;
        mover_robo(VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA); // Avançar
        vTaskDelay(pdMS_TO_TICKS(300));
        return;
    }

    // Lógica principal baseada no estado atual
    switch (estado_atual) {
        case ESTADO_BUSCA:
            if (distancia > 0 && distancia <= DISTANCIA_ATAQUE) {
                ESP_LOGI(TAG, "Inimigo detectado - ATACANDO");
                estado_atual = ESTADO_ATAQUE;
                mover_robo(VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);
            } else {
                ESP_LOGI(TAG, "Procurando inimigo - GIRANDO");
                mover_robo(VELOCIDADE_BUSCA, -VELOCIDADE_BUSCA); // Girar para esquerda
            }
            break;
            
        case ESTADO_ATAQUE:
            if (distancia > 0 && distancia <= DISTANCIA_ATAQUE) {
                ESP_LOGI(TAG, "Mantendo ataque");
                mover_robo(VELOCIDADE_MAXIMA, VELOCIDADE_MAXIMA);
            } else {
                ESP_LOGI(TAG, "Perdeu o alvo - BUSCANDO");
                estado_atual = ESTADO_BUSCA;
            }
            break;
            
        case ESTADO_RECUO:
            ESP_LOGI(TAG, "Voltando para busca");
            estado_atual = ESTADO_BUSCA;
            break;
            
        case ESTADO_GIRO:
            ESP_LOGI(TAG, "Girando para reposicionar");
            mover_robo(-VELOCIDADE_BUSCA, VELOCIDADE_BUSCA);
            vTaskDelay(pdMS_TO_TICKS(500));
            estado_atual = ESTADO_BUSCA;
            break;
    }
}

// Task principal
void task_robo_sumo(void *pvParameters) {
    ESP_LOGI(TAG, "Iniciando robô de sumô...");
    
    // Aguardar 5 segundos antes de começar (regra do sumô)
    for (int i = 5; i > 0; i--) {
        ESP_LOGI(TAG, "Iniciando em %d segundos...", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    ESP_LOGI(TAG, "LUTA INICIADA!");
    
    while (1) {
        controlar_robo();
        vTaskDelay(pdMS_TO_TICKS(50)); // Loop a cada 50ms
    }
}

void app_main() {
    ESP_LOGI(TAG, "Configurando robô de sumô...");
    
    // Configurar periféricos
    configurar_gpio();
    configurar_adc();
    configurar_pwm();
    
    ESP_LOGI(TAG, "Configuração concluída!");
    
    // Criar task principal
    xTaskCreate(task_robo_sumo, "robo_sumo", 4096, NULL, 5, NULL);
}