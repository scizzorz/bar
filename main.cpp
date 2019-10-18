//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HX711.h>
//#include <EEPROM.h>
//#include <SPI.h>
//#include <Wire.h>

#define DEBOUNCE 30
#define NUM_PUMPS 6

class Knob {
private:
  int pin;

public:
  Knob(int pin) : pin(pin) {}

  void init() { pinMode(this->pin, INPUT); }

  int read() { return analogRead(pin); }
};

class Beeper {
private:
  int pin;

public:
  Beeper(int pin) : pin(pin) {}

  void init() { pinMode(this->pin, OUTPUT); }

  void beep(int len) {
    digitalWrite(this->pin, HIGH);
    delay(len);
    digitalWrite(this->pin, LOW);
  }
};

class Button {
private:
  int pin;
  unsigned long lastDebounce;
  int buttonState;
  int lastButtonState;

public:
  Button(int pin)
      : pin(pin), lastDebounce(0), buttonState(LOW), lastButtonState(HIGH) {}

  void init() { pinMode(this->pin, INPUT); }

  /// this does *not* do any debouncing!
  bool read() { return digitalRead(this->pin); }

  /// This returns true if the button state CHANGED. It does *not* return the
  /// button state! If you want to react to presses, use
  /// `.checkPress() && .isPressed()`
  bool checkPress() {
    bool change = false;
    int currentReading = this->read();
    if (currentReading != this->lastButtonState) {
      this->lastDebounce = millis();
    }

    if ((millis() - this->lastDebounce) > DEBOUNCE) {
      if (currentReading != this->buttonState) {
        this->buttonState = currentReading;
        change = true;
      }
    }

    this->lastButtonState = currentReading;
    return change;
  }

  bool isPressed() { return this->buttonState; }
};

class Pump {
private:
  int pin;
  bool on;

public:
  Pump(int pin) : pin(pin), on(false) {}

  void init() {
    pinMode(this->pin, OUTPUT);
    this->turnOff();
  }

  void set(bool to) {
    this->on = to;
    digitalWrite(this->pin, to ? LOW : HIGH);
  }
  void turnOn() { this->set(true); }
  void turnOff() { this->set(false); }
  bool isOn() { return this->on; }
};

class Scale {
private:
  HX711 sensor;
  int dataPin;
  int clockPin;
  float calibrationFactor;

public:
  Scale(int dataPin, int clockPin, float calibrationFactor)
      : dataPin(dataPin), clockPin(clockPin),
        calibrationFactor(calibrationFactor){};

  void init() {
    this->sensor.begin(this->dataPin, this->clockPin);
    this->sensor.set_scale(this->calibrationFactor);
    this->sensor.tare();
  }

  void tare() { this->sensor.tare(); }

  float read() { return this->sensor.get_units(1); }
};

Pump pumps[NUM_PUMPS] = {
    Pump(4), Pump(5), Pump(6), Pump(7), Pump(8), Pump(9),
};

Scale scale(3, 2, -7000);

void press(int i) {}

void setup() {
  Serial.begin(9600);
  Serial.println("booting...");

  scale.init();

  for (int i = 0; i < NUM_PUMPS; i++) {
    pumps[i].init();
    pumps[i].turnOn();
    delay(100);
    pumps[i].turnOff();
    Serial.print("pump ");
    Serial.print(i, DEC);
    Serial.println(" ready.");
  }

  Serial.println("booted.");
}

void loop() {
  Serial.print("reading = ");
  Serial.println(scale.read(), 1);
  delay(500);
}

int main(void) {
  init();
  setup();
  while (true) {
    loop();
  }
}
