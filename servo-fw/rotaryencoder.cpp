#include <Adafruit_PWMServoDriver.h>
#include "rotaryencoder.h"

static const int LEFT_PIN = 3; // Pin of left rotary encoder (digital)
static const int RIGHT_PIN = 4; // Pin of right rotary encder
static const double WARNING_TIME = 40l * 1000l; // Number of microseconds early to send stop message to Pi

static CommandError test_func(int commandId, String argument) {
  String arg1 = pop_option(argument);
  serialWrite(commandId, '>', arg1);
  return OK;
}

static CommandError count_rotary_encoder(int CommandId, String argument) {
  int re_mode = pop_option(argument).toInt(); // Which wheel to track
  int final_count = pop_option(argument).toInt(); // Number of switches to run
  long timeout = pop_option(argument).toInt(); // Function timeout in microseconds

  int count[] = {0, 0}; // How many times the signal has switched
  int state[] = {0, 0}; // To store the current state

  long t0 = micros(); // Microseconds since arduino boot

  long last_switch[] = {t0, t0}; // Time of last switch
  while (true) {
    int last_state[] = {state[0], state[1]};
    state[0] = digitalRead(LEFT_PIN);
    state[1] = digitalRead(RIGHT_PIN);

    auto t1 = micros();

    // We treat both LO->HI and HI->LO as a switch
    if (state[0] != last_state[0]) {
      count[0]++;
      long switch_time = t1 - last_switch[0];
      double left_velocity = 1 / switch_time; // switches per microsecond
      if (re_mode == 0) {
        int counts_remaining = final_count - count[0];
        if (counts_remaining * switch_time <= WARNING_TIME) {
          // warn()
          break;
        }
      }
    }
    if (state[1] != last_state[1]) {
      count[1]++;
      long switch_time = t1 - last_switch[1];
      double left_velocity = 1 / switch_time;
      if (re_mode == 1) {
        int counts_remaining = final_count - count[1];
        if (counts_remaining * switch_time <= WARNING_TIME) {
          // warn()
          break;
        }
      }

      // send_message()
      if (t1 - t0 > timeout) {
        break;
      }
    }
  }
}
