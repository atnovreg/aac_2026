// Arduino Mega firmware for arduino_mega_control_node pwm_line protocol.
//
// Wiring:
//   Right in-wheel motor IN1 -> D13, IN2 -> D12
//   Left  in-wheel motor IN1 -> D11, IN2 -> D10
//   Steering DC motor  IN1 -> D9,  IN2 -> D8
//
// Serial command:
//   <S:steering_pwm,L:left_pwm,R:right_pwm>
//
// Values are signed PWM commands in the range -255..255.

const int RIGHT_IN1 = 13;
const int RIGHT_IN2 = 12;

const int LEFT_IN1 = 11;
const int LEFT_IN2 = 10;

const int STEER_IN1 = 9;
const int STEER_IN2 = 8;

const long BAUD_RATE = 9600;
const unsigned long COMMAND_TIMEOUT_MS = 300;
const int PWM_LIMIT = 255;

char rx_buffer[48];
size_t rx_index = 0;
bool receiving = false;
unsigned long last_command_ms = 0;

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);

  pinMode(STEER_IN1, OUTPUT);
  pinMode(STEER_IN2, OUTPUT);

  stopAll();
  last_command_ms = millis();
}

void loop() {
  readSerialCommands();

  if (millis() - last_command_ms > COMMAND_TIMEOUT_MS) {
    stopAll();
  }
}

void readSerialCommands() {
  while (Serial.available() > 0) {
    const char c = Serial.read();

    if (c == '<') {
      receiving = true;
      rx_index = 0;
      rx_buffer[rx_index++] = c;
      continue;
    }

    if (!receiving) {
      continue;
    }

    if (rx_index >= sizeof(rx_buffer) - 1) {
      receiving = false;
      rx_index = 0;
      stopAll();
      continue;
    }

    rx_buffer[rx_index++] = c;

    if (c == '>') {
      rx_buffer[rx_index] = '\0';
      int steer_pwm = 0;
      int left_pwm = 0;
      int right_pwm = 0;

      if (parsePwmCommand(rx_buffer, steer_pwm, left_pwm, right_pwm)) {
        applySignedPwm(STEER_IN1, STEER_IN2, steer_pwm);
        applySignedPwm(LEFT_IN1, LEFT_IN2, left_pwm);
        applySignedPwm(RIGHT_IN1, RIGHT_IN2, right_pwm);
        last_command_ms = millis();
      } else {
        stopAll();
      }

      receiving = false;
      rx_index = 0;
    }
  }
}

bool parsePwmCommand(const char * command, int & steer_pwm, int & left_pwm, int & right_pwm) {
  if (command[0] != '<') {
    return false;
  }

  const char * s_ptr = strstr(command, "S:");
  const char * l_ptr = strstr(command, "L:");
  const char * r_ptr = strstr(command, "R:");

  if (s_ptr == nullptr || l_ptr == nullptr || r_ptr == nullptr) {
    return false;
  }

  steer_pwm = constrain(atoi(s_ptr + 2), -PWM_LIMIT, PWM_LIMIT);
  left_pwm = constrain(atoi(l_ptr + 2), -PWM_LIMIT, PWM_LIMIT);
  right_pwm = constrain(atoi(r_ptr + 2), -PWM_LIMIT, PWM_LIMIT);
  return true;
}

void applySignedPwm(const int in1, const int in2, const int pwm) {
  const int value = constrain(pwm, -PWM_LIMIT, PWM_LIMIT);
  const int magnitude = abs(value);

  if (magnitude == 0) {
    analogWrite(in1, 0);
    analogWrite(in2, 0);
  } else if (value > 0) {
    analogWrite(in1, magnitude);
    analogWrite(in2, 0);
  } else {
    analogWrite(in1, 0);
    analogWrite(in2, magnitude);
  }
}

void stopAll() {
  analogWrite(STEER_IN1, 0);
  analogWrite(STEER_IN2, 0);
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}
