#include <Adafruit_PWMServoDriver.h>

// Multiplying by this converts round-trip duration in microseconds to distance to object in millimetres.
static const float ULTRASOUND_COEFFICIENT = 1e-6 * 343.0 * 0.5 * 1e3;

// Horrible hack because I can't include <climits> for some reason.
static const unsigned int UINT_MAX = (unsigned int) -1;

static const String FIRMWARE_VERSION = "SourceBots PWM/GPIO v0.0.1";

typedef String CommandError;

static const CommandError OK = "";

#define COMMAND_ERROR(x) ((x))

static Adafruit_PWMServoDriver SERVOS = Adafruit_PWMServoDriver();

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  for (int pin = 2; pin <= 12; pin++) {
    pinMode(pin, INPUT);
  }

  Serial.begin(9600);
  Serial.setTimeout(5);

  SERVOS.begin();
  SERVOS.setPWMFreq(50);

  Serial.write("# booted\n");
}

class CommandHandler {
public:
  String command;
  CommandError (*run)(int, String argument);
  String helpMessage;

  CommandHandler(String cmd, CommandError (*runner)(int, String), String help);
};

CommandHandler::CommandHandler(String cmd, CommandError (*runner)(int, String), String help)
: command(cmd), run(runner), helpMessage(help)
{
}

static void serialWrite(int commandId, char lineType, const String& str);

static String pop_option(String& argument) {
  int separatorIndex = argument.indexOf(' ');
  if (separatorIndex == -1) {
    String copy(argument);
    argument = "";
    return copy;
  } else {
    String first_argument(argument.substring(0, separatorIndex));
    argument = argument.substring(separatorIndex + 1);
    return first_argument;
  }
}

static CommandError run_help(int commandId, String argument);

static CommandError led(int commandId, String argument) {
  if (argument == "on") {
    digitalWrite(LED_BUILTIN, HIGH);
  } else if (argument == "off") {
    digitalWrite(LED_BUILTIN, LOW);
  } else {
    return COMMAND_ERROR("unknown argument");
  }
  return OK;
}

static CommandError servo(int commandId, String argument) {
  String servoArg = pop_option(argument);
  String widthArg = pop_option(argument);

  if (argument.length() || !servoArg.length() || !widthArg.length()) {
    return COMMAND_ERROR("servo takes exactly two arguments");
  }

  auto width = widthArg.toInt();
  auto servo = servoArg.toInt();
  if (servo < 0 || servo > 15) {
    return COMMAND_ERROR("servo index out of range");
  }
  if (width != 0 && (width < 150 || width > 550)) {
    return COMMAND_ERROR("width must be 0 or between 150 and 550");
  }
  SERVOS.setPWM(servo, 0, width);
  return OK;
}

static CommandError write_pin(int commandId, String argument) {
  String pinIDArg = pop_option(argument);
  String pinStateArg = pop_option(argument);

  if (argument.length() || !pinIDArg.length() || !pinStateArg.length()) {
    return COMMAND_ERROR("need exactly two arguments: <pin> <high/low/hi-z/pullup>");
  }

  int pin = pinIDArg.toInt();

  if (pin < 2 || pin > 12) {
    return COMMAND_ERROR("pin must be between 2 and 12");
  }

  if (pinStateArg == "high") {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
  } else if (pinStateArg == "low") {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  } else if (pinStateArg == "hi-z") {
    pinMode(pin, INPUT);
  } else if (pinStateArg == "pullup") {
    pinMode(pin, INPUT_PULLUP);
  } else {
    return COMMAND_ERROR("unknown drive state");
  }
  return OK;
}

static CommandError read_pin(int commandId, String argument) {
  String pinIDArg = pop_option(argument);

  if (argument.length() || !pinIDArg.length()) {
    return COMMAND_ERROR("need exactly one argument: <pin>");
  }

  int pin = pinIDArg.toInt();

  if (pin < 2 || pin > 12) {
    return COMMAND_ERROR("pin must be between 2 and 12");
  }

  auto state = digitalRead(pin);

  if (state == HIGH) {
    serialWrite(commandId, '>', "high");
  } else {
    serialWrite(commandId, '>', "low");
  }

  return OK;
}

static void read_analogue_pin_to_serial(int commandId, const String& name, int pin) {
  int reading = analogRead(pin);
  double mungedReading = (double)reading * (5.0 / 1024.0);
  serialWrite(commandId, '>', name + " " + String(mungedReading));
}

static CommandError analogue_read(int commandId, String argument) {
  read_analogue_pin_to_serial(commandId, "a0", A0);
  read_analogue_pin_to_serial(commandId, "a1", A1);
  read_analogue_pin_to_serial(commandId, "a2", A2);
  read_analogue_pin_to_serial(commandId, "a3", A3);
  return OK;
}

static CommandError ultrasound_read(int commandId, String argument) {
  String triggerPinStr = pop_option(argument);
  String echoPinStr = pop_option(argument);

  if (argument.length() || !triggerPinStr.length() || !echoPinStr.length()) {
    return COMMAND_ERROR("need exactly two arguments: <trigger-pin> <echo-pin>");
  }

  int triggerPin = triggerPinStr.toInt();
  int echoPin = echoPinStr.toInt();

  // Reset trigger pin.
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);

  // Pulse trigger pin.
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Set echo pin to input now (we don't do it earlier, since it's allowable
  // for triggerPin and echoPin to be the same pin).
  pinMode(echoPin, INPUT);

  // Read return pulse.
  float duration = (float) pulseIn(echoPin, HIGH);       // In microseconds.
  float distance = duration * ULTRASOUND_COEFFICIENT;    // In millimetres.
  distance = constrain(distance, 0.0, (float) UINT_MAX); // Ensure that the next line won't overflow.
  unsigned int distanceInt = (unsigned int) distance;

  serialWrite(commandId, '>', String(distanceInt));

  return OK;
}

static CommandError get_version(int commandId, String argument) {
  serialWrite(commandId, '>', FIRMWARE_VERSION);
  return OK;
}

static const CommandHandler commands[] = {
  CommandHandler("help", &run_help, "show information"),
  CommandHandler("led", &led, "control the debug LED (on/off)"),
  CommandHandler("servo", &servo, "control a servo <num> <width>"),
  CommandHandler("version", &get_version, "get firmware version"),
  CommandHandler("gpio-write", &write_pin, "set output from GPIO pin"),
  CommandHandler("gpio-read", &read_pin, "get digital input from GPIO pin"),
  CommandHandler("analogue-read", &analogue_read, "get all analogue inputs"),
  CommandHandler("ultrasound-read", &ultrasound_read, "read an ultrasound sensor <trigger-pin> <echo-pin>"),
};

static void serialWrite(int commandId, char lineType, const String& str) {
    if (commandId != 0) {
        Serial.write('@');
        Serial.print(commandId, DEC);
        Serial.write(' ');
    }

    Serial.write(lineType);
    Serial.write(' ');

    Serial.println(str);
}

static void dispatch_command(int commandId, const class CommandHandler& handler, const String& argument) {
  auto err = handler.run(commandId, argument);
  if (err == OK) {
    serialWrite(commandId, '+', "OK");
  } else {
    serialWrite(commandId, '-', String("Error: ") + err);
  }
}

static void handle_actual_command(int commandId, const String& cmd) {
    for (int i = 0; i < sizeof(commands) / sizeof(CommandHandler); ++i) {
      const CommandHandler& handler = commands[i];

      if (handler.command == cmd) {
        dispatch_command(commandId, handler, "");
        return;
      } else if (cmd.startsWith(handler.command + " ")) {
        dispatch_command(
          commandId,
          handler,
          cmd.substring(handler.command.length() + 1)
        );
        return;
      }
    }

    serialWrite(commandId, '-', String("Error, unknown command: ") + cmd);
}

static void handle_command(const String& cmd) {
  if (cmd.startsWith("@")) {
    auto spaceIndex = cmd.indexOf(' ');
    auto commandId = cmd.substring(1, spaceIndex).toInt();
    handle_actual_command(commandId, cmd.substring(spaceIndex + 1));
  } else {
    handle_actual_command(0, cmd);
  }
}

static CommandError run_help(int commandId, String argument) {
  if (argument == "") {
    serialWrite(commandId, '#', "commands: ");
    for (int i = 0; i < sizeof(commands) / sizeof(CommandHandler); ++i) {
      const CommandHandler& handler = commands[i];

      String s("   ");
      s += handler.command;

      for (int i = handler.command.length(); i < 30; ++i) {
        s += " ";
      }

      s += handler.helpMessage;
      serialWrite(commandId, '#', s);
    }
    return OK;
  } else {
    for (int i = 0; i < sizeof(commands) / sizeof(CommandHandler); ++i) {
      const CommandHandler& handler = commands[i];
      if (handler.command == argument) {
        serialWrite(commandId, '#', handler.command);
        serialWrite(commandId, '#', handler.helpMessage);
        return OK;
      }
    }
  }
  return COMMAND_ERROR("I do not know anything about that topic");
}

static String serialBuffer;
static boolean skipWS = false;

static void process_serial() {
  auto serialInput = Serial.read();

  // Allow resetting the buffer by sending a NULL. This allows for recovery
  // from partial commands being sent or received.
  if (serialInput == 0) {
    serialBuffer = "";
    return;
  }

  if (serialInput == -1) {
    return;
  }

  if (serialInput == '\r') {
    return; // ignore CR, just take the LF
  }

  if (serialInput == '\t') {
    serialInput = ' '; // treat tabs as equivalent to spaces
  }

  if (serialInput == '\n') {
    serialBuffer.trim();
    Serial.write("# ");
    Serial.write(serialBuffer.c_str());
    Serial.write('\n');
    handle_command(serialBuffer);
    serialBuffer = "";
    Serial.flush();
    return;
  }

  if (serialInput == ' ' && skipWS) {
    return; // ignore junk whitespace
  } else {
    skipWS = (serialInput == ' '); // ignore any successive whitespace
  }

  serialBuffer += (char)serialInput;
}

void loop() {
  while (Serial.available()) {
    process_serial();
  }
}
