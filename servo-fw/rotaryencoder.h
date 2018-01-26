#include <Adafruit_PWMServoDriver.h>

typedef String CommandError;
static const CommandError OK = "";

static CommandError count_rotary_encoder(int commandId, String argument);
static CommandError test_func(int commandId, String argument);

static String pop_option(String& argument);
static void serialWrite(int commandId, char lineType, const String& str);
