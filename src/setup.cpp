#include <setup.h>

HardwareTimer pwmtimer(3);

void setup_timer(){
  pwmtimer.setPrescaleFactor(1);
  pwmtimer.setPeriod(12160); //82Hz
}

void setup_pin_modes() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_YAW_PWM, PWM);
  pinMode(SERVO_PITCH_PWM, PWM);
}

void setup_serial_baud_rate() {
  SERIAL_DEBUG.begin(BAUD_RATE_DEBUG);
}