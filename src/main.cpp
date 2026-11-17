/**
 * =========================================================================================
 * Finite-Control-Set Model Predictive Control (FCS-MPC) for PMSM Power Electronics Drive
 * Department of Electrical Engineering, Veermata Jijabai Technological Institute (VJTI)
 * Author: Arpan Kailash Gupta (Roll No: 241030009)
 * Target Hardware: Espressif ESP32 Dev Module (Xtensa Dual-Core 240 MHz)
 * =========================================================================================
 */

#ifdef ARDUINO
#include <Arduino.h>
#include "esp_timer.h"
#include "esp_cpu.h"
#include "mpc_api.h"
#include "motor_params.h"

/* Inverter Gate Driver Pin Definitions (Direct GPIO Triggering, No PWM Modulator) */
#define PIN_GATE_AH     16  /* Phase A High-Side Gate */
#define PIN_GATE_AL     17  /* Phase A Low-Side Gate (Inverted with dead-time) */
#define PIN_GATE_BH     18  /* Phase B High-Side Gate */
#define PIN_GATE_BL     19  /* Phase B Low-Side Gate */
#define PIN_GATE_CH     21  /* Phase C High-Side Gate */
#define PIN_GATE_CL     22  /* Phase C Low-Side Gate */

/* Current Shunt ADC Pins */
#define PIN_ADC_IA      36  /* ADC1_CH0 (Phase A Current) */
#define PIN_ADC_IB      39  /* ADC1_CH3 (Phase B Current) */
#define PIN_ADC_IC      34  /* ADC1_CH6 (Phase C Current) */
#define PIN_ADC_VDC     35  /* ADC1_CH7 (DC-Link Bus Voltage) */

/* Optical/Magnetic Quadrature Encoder Pins */
#define PIN_ENC_A       25
#define PIN_ENC_B       26
#define PIN_ENC_Z       27

static MPCTelemetry_t g_telemetry;
static hw_timer_t* g_timer = NULL;
static portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

/* Real-Time Execution Profiling */
static volatile uint32_t g_last_cycles = 0;
static volatile uint32_t g_max_cycles = 0;
static volatile bool g_step_ready = false;

static float g_target_speed_rpm = 1500.0f;
static float g_measured_speed_rpm = 0.0f;
static float g_theta_e = 0.0f;

/* Apply discrete switching state directly to gate driver pins */
static inline void IRAM_ATTR apply_inverter_gates(uint8_t sa, uint8_t sb, uint8_t sc) {
    /* Fast direct GPIO register write on ESP32 */
    if (sa) {
        GPIO.out_w1tc = (1 << PIN_GATE_AL);
        GPIO.out_w1ts = (1 << PIN_GATE_AH);
    } else {
        GPIO.out_w1tc = (1 << PIN_GATE_AH);
        GPIO.out_w1ts = (1 << PIN_GATE_AL);
    }
    
    if (sb) {
        GPIO.out_w1tc = (1 << PIN_GATE_BL);
        GPIO.out_w1ts = (1 << PIN_GATE_BH);
    } else {
        GPIO.out_w1tc = (1 << PIN_GATE_BH);
        GPIO.out_w1ts = (1 << PIN_GATE_BL);
    }
    
    if (sc) {
        GPIO.out_w1tc = (1 << PIN_GATE_CL);
        GPIO.out_w1ts = (1 << PIN_GATE_CH);
    } else {
        GPIO.out_w1tc = (1 << PIN_GATE_CH);
        GPIO.out_w1ts = (1 << PIN_GATE_CL);
    }
}

/* 20 kHz Hardware Timer Interrupt Service Routine (50 us cycle) */
void IRAM_ATTR on_mpc_timer(void) {
    portENTER_CRITICAL_ISR(&g_timer_mux);
    uint32_t start_cycles = esp_cpu_get_ccount();
    
    /* 1. Sample ADC phase currents and DC bus voltage */
    int raw_ia = analogRead(PIN_ADC_IA);
    int raw_ib = analogRead(PIN_ADC_IB);
    int raw_ic = analogRead(PIN_ADC_IC);
    int raw_vdc = analogRead(PIN_ADC_VDC);
    
    /* Convert raw ADC counts to physical floating-point quantities */
    float ia = ((float)raw_ia - 2048.0f) * (20.0f / 2048.0f);
    float ib = ((float)raw_ib - 2048.0f) * (20.0f / 2048.0f);
    float ic = ((float)raw_ic - 2048.0f) * (20.0f / 2048.0f);
    float vdc = (float)raw_vdc * (36.0f / 4095.0f);
    if (vdc < 12.0f) vdc = VSI_VBUS_NOMINAL;
    
    /* 2. Execute Finite-Control-Set Model Predictive Control Optimization */
    mpc_system_step(ia, ib, ic, g_theta_e, g_measured_speed_rpm, g_target_speed_rpm, vdc, &g_telemetry);
    
    /* 3. Apply the optimal switching vector directly to the power inverter gates */
    apply_inverter_gates(g_telemetry.sa, g_telemetry.sb, g_telemetry.sc);
    
    uint32_t end_cycles = esp_cpu_get_ccount();
    g_last_cycles = end_cycles - start_cycles;
    if (g_last_cycles > g_max_cycles) {
        g_max_cycles = g_last_cycles;
    }
    g_step_ready = true;
    portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    
    Serial.println("==================================================================");
    Serial.println("Finite-Control-Set Model Predictive Control (FCS-MPC) PMSM Drive");
    Serial.println("Department of Electrical Engineering, VJTI Mumbai");
    Serial.println("Author: Arpan Kailash Gupta (Roll No: 241030009)");
    Serial.println("==================================================================");
    
    /* Initialize Inverter Gate Output Pins */
    pinMode(PIN_GATE_AH, OUTPUT);
    pinMode(PIN_GATE_AL, OUTPUT);
    pinMode(PIN_GATE_BH, OUTPUT);
    pinMode(PIN_GATE_BL, OUTPUT);
    pinMode(PIN_GATE_CH, OUTPUT);
    pinMode(PIN_GATE_CL, OUTPUT);
    
    /* Ensure all switches are turned off initially */
    apply_inverter_gates(0, 0, 0);
    
    /* Configure ADC Resolution */
    analogReadResolution(12);
    
    /* Initialize MPC Core with nominal DC bus voltage */
    mpc_system_init(VSI_VBUS_NOMINAL);
    mpc_system_set_weights(MPC_WEIGHT_ID, MPC_WEIGHT_IQ, MPC_WEIGHT_SW, MPC_WEIGHT_CMV);
    
    /* Set up 20 kHz Hardware Timer (50 us period) on Timer 0 */
    g_timer = timerBegin(0, 80, true); /* 80 MHz / 80 = 1 MHz (1 us tick) */
    timerAttachInterrupt(g_timer, &on_mpc_timer, true);
    timerAlarmWrite(g_timer, 50, true); /* 50 us alarm */
    timerAlarmEnable(g_timer);
    
    Serial.println("[INFO] FCS-MPC 20 kHz Hardware Timer Active (Period: 50 us).");
}

void loop() {
    static uint32_t last_print = 0;
    if (millis() - last_print >= 250) {
        last_print = millis();
        
        float wcet_us = (float)g_max_cycles / 240.0f; /* 240 MHz clock */
        float last_us = (float)g_last_cycles / 240.0f;
        
        Serial.printf("[FCS-MPC] Speed: %6.1f / %6.1f RPM | Id/Iq: %5.2f / %5.2f A | Vec: V%d (%d%d%d) | Loss: %4.2f W | Eff: %4.1f%% | f_sw: %5.1f kHz | WCET: %4.1f us\n",
                      g_telemetry.speed_rpm, g_telemetry.speed_ref_rpm,
                      g_telemetry.i_d, g_telemetry.i_q,
                      g_telemetry.opt_vector, g_telemetry.sa, g_telemetry.sb, g_telemetry.sc,
                      g_telemetry.p_total_loss, g_telemetry.efficiency,
                      g_telemetry.switching_freq_hz / 1000.0f,
                      last_us);
    }
    delay(10);
}

#else
/* Standard C entry point for non-Arduino desktop validation */
int main(void) {
    return 0;
}
#endif
