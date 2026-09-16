#include <Arduino.h>

// Pin definitions for communication with Tang Nano 20K
const int PWM_OUT_PIN = 4;   // Sends control effort to FPGA
const int ENC_IN_PIN = 5;    // Receives simulated feedback pulses (or use SPI)

// PID Controller Variables
double setpoint = 1500.0;    // Target position
double input = 0.0, output = 0.0;
double err = 0.0, last_err = 0.0, integral = 0.0;

// PID Gains
double Kp = 1.2;
double Ki = 0.05;
double Kd = 0.1;

unsigned long last_time = 0;
const long interval = 1; // 1ms control loop (1 kHz)

void setup() {
  Serial.begin(115200);
  pinMode(PWM_OUT_PIN, OUTPUT);
  pinMode(ENC_IN_PIN, INPUT);
  
  // Configure LEDC PWM for ESP32-S3
  ledcAttach(PWM_OUT_PIN, 10000, 8); // 10kHz frequency, 8-bit resolution
}

void loop() {
  unsigned long current_time = millis();
  
  if (current_time - last_time >= interval) {
    double dt = (current_time - last_time) / 1000.0;
    last_time = current_time;

    // 1. Read simulated plant feedback (mocked here as reading a pin/register)
    // In a full build, you'd read multi-bit data via SPI or parallel GPIOs
    input = (double)pulseIn(ENC_IN_PIN, HIGH, 1000); 

    // 2. Compute PID Error
    err = setpoint - input;
    integral += err * dt;
    double derivative = (err - last_err) / dt;
    
    output = (Kp * err) + (Ki * integral) + (Kd * derivative);
    last_err = err;

    // 3. Constrain output to 8-bit PWM limits (0 - 255)
    int pwm_val = constrain((int)output, 0, 255);
    
    // 4. Send control signal to FPGA plant simulator
    ledcWrite(PWM_OUT_PIN, pwm_val);

    // 5. Debug Telemetry
    Serial.printf("Setpoint: %.1f | Input: %.1f | PWM: %d\n", setpoint, input, pwm_val);
  }
}
