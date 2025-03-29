#include <Arduino.h>
#include <Wire.h>

bool const STRAIGHT = true;
bool const TURN = false;

using Direction = bool;
using Position = int; // Position is synonymous for "track"
using SignedIndex = int;
using UnsignedIndex = size_t;
SignedIndex const IDX_INVALID = -1;
Position const POSITION_INVALID = -1;

class OutputModule {
public:
  OutputModule(uint8_t addr) : addr{addr} {}
  inline void reset() { pin_states = 0; }
  inline void set_pin_on(uint8_t pin) { pin_states |= (0x01 << pin); }
  inline void commit() {
    // first, set all pin states according to their values
    Wire.beginTransmission(addr);
    Wire.write(pin_states);
    Wire.endTransmission();

    delay(200);

    // then, reset all pin values so that coils are no more supplied with
    // current
    Wire.beginTransmission(addr);
    Wire.write(0x00); // TODO might need inversion
    Wire.endTransmission();
  }

private:
  uint8_t const addr;
  uint8_t pin_states = 0;
};

struct OutputModulePin {
  uint8_t module_idx;
  uint8_t pin;
};

OutputModule output_modules[]{
    0x20, 0x22, 0x24, 0x26, 0x28,
};

struct Switch {
  Switch(OutputModulePin const &pin_gerade, OutputModulePin const &pin_abbiegen)
      : pin_gerade{pin_gerade}, pin_abbiegen{pin_abbiegen} {}
  OutputModulePin const pin_gerade;
  OutputModulePin const pin_abbiegen;
};

struct SwitchAction {
  Switch const *weiche;
  Direction richtung; // STRAIGHT or TURN

  inline void init() const {}

  void perform() const {
    OutputModulePin const *module_pin =
        richtung == STRAIGHT ? &weiche->pin_gerade : &weiche->pin_abbiegen;

    Serial.print("  --> WeichenAction: ");
    Serial.print(module_pin->module_idx);
    Serial.print(":");
    Serial.print(module_pin->pin);
    Serial.println(richtung == STRAIGHT ? " | GERADE" : " | ABBIEGEN");

    output_modules[module_pin->module_idx].set_pin_on(module_pin->pin);
  }
};

struct InstructionList /* IL */ {
  UnsignedIndex length;    // number of SwitchActions in the InstructionList
  SwitchAction actions[4]; // Array of 4 SwitchActions max.

  inline bool is_valid() const { return length > 0; }

  void init() const {
    for (UnsignedIndex idx_il = 0; idx_il < length; idx_il++) {
      actions[idx_il].init();
    }
  }

  void perform() const {
    for (UnsignedIndex idx_il = 0; idx_il < length; idx_il++) {
      actions[idx_il].perform();
    }

    for (OutputModule &output_module : output_modules) {
      output_module.commit();
    }
  }
};

InstructionList const IL_INVALID = {0, {}};

struct Route {
  Position src; // starting point
  Position dst; // destination point

  void print() const {
    Serial.print("Route: ");
    Serial.print(src);
    Serial.print(" => ");
    Serial.print(dst);
  }
};

bool operator==(Route const &first, Route const &second) {
  return first.src == second.src && first.dst == second.dst;
}

struct Line {
  Route route;
  InstructionList il;
};

// PinMapping specifies association between pin number on Arduion Board and
// position in the map
struct PinMapping {
  uint8_t pin;
  Position position;
};

// Numbers of switches correspond to digital pins of the Arduino
// the relays of the switches are attached to
Switch const SWITCH_1{{1, 0}, {2, 1}};
Switch const SWITCH_2{{3, 0}, {4, 1}};
Switch const SWITCH_3{{3, 5}, {3, 6}};

Line const TABLE[] = {
    // First Line
    {
        {1, 3}, // Route
        {
            // InstructionList
            2, // Number of SwitchActions in InstructionList
            // il Array
            {
                {&SWITCH_1, TURN}, // <-- First SwitchAction in 'il'
                {&SWITCH_2, TURN}, // <-- Second SwitchAction in 'il'
            },
        },
    },
    // Second Line
    {
        {1, 5}, // Route
        {
            // InstructionList
            1, // Number of SwitchActions in InstructionList
            // il Array
            {
                {&SWITCH_3,
                 STRAIGHT}, // <-- First (and only) SwitchAction in 'il'
            },
        },
    },
    // ..
};

PinMapping const PIN_MAPPINGS[] = {
    {
        2, // digital pin
        1, // position (= track)
    },
    {
        3, // digital pin
        2, // position (= track)
    },
    {
        4, // digital pin
        4, // position (= track)
    },
    {
        5, // digital pin
        7, // position (= track)
    },
    // ...
};

struct Pins {
  static size_t const NUM_PIN_MAPPINGS =
      sizeof(PIN_MAPPINGS) / sizeof(PIN_MAPPINGS[0]);

  bool states[NUM_PIN_MAPPINGS];

  Pins() {
    for (UnsignedIndex index = 0; index < NUM_PIN_MAPPINGS; index++) {
      states[index] = digitalRead(PIN_MAPPINGS[index].pin);
    }
  }

  SignedIndex diff(Pins const &other) const {
    for (UnsignedIndex index = 0; index < NUM_PIN_MAPPINGS; index++) {
      if (!states[index] && other.states[index]) {
        return index;
      }
    }
    return IDX_INVALID;
  }
};

Position determineButtonPosition() {
  Pins pin_states_prev{};
  SignedIndex pin_idx_pressed = IDX_INVALID;

  while (pin_idx_pressed == IDX_INVALID) {
    Pins pin_states_cur{};
    pin_idx_pressed = pin_states_prev.diff(pin_states_cur);
    pin_states_prev = pin_states_cur;
  }

  return PIN_MAPPINGS[pin_idx_pressed].position;
}

InstructionList determineInstructionList(Route const &strecke) {
  for (Line const &row : TABLE) {
    if (row.route == strecke) {
      return row.il;
    }
  }

  return IL_INVALID;
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  for (PinMapping const &pin_mapping : PIN_MAPPINGS) {
    pinMode(pin_mapping.pin, INPUT_PULLUP);
  }

  for (Line const &row : TABLE) {
    row.il.init(); // TODO is this still needed?
  }
}

void loop() {
  // see: https://github.com/RalphBacon/PCF8574-Pin-Extender-I2C/tree/master
  int src_position = determineButtonPosition();
  int dst_position = determineButtonPosition();

  Route route{src_position, dst_position}; // two positions make a route
  route.print();

  InstructionList il = determineInstructionList(
      route); // determine Instructionlist for given route from table

  if (il.is_valid()) {
    // Route was pre-defined
    il.perform();
  } else {
    Serial.println("  unknown route.");
  }
}

#if PROVIDE_MAIN
auto main() -> int {
  setup();
  for (;;) {
    loop();
    delay(5 * 1000);
  }
  return 0; // never reached
}
#endif
