//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
#include <Arduino.h>
#include <HX711.h>
//#include <EEPROM.h>
//#include <SPI.h>
//#include <Wire.h>

#define DEBOUNCE 30
#define NUM_PUMPS 6
#define NUM_BUTTS 4
#define MIN_CUP_WEIGHT 5

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
      : pin(pin), lastDebounce(0), buttonState(HIGH), lastButtonState(HIGH) {}

  void init() { pinMode(this->pin, INPUT_PULLUP); }

  /// this does *not* do any debouncing!
  bool read() { return !digitalRead(this->pin); }

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

  float read() { return this->sensor.get_units(2); }
};

Pump pumps[NUM_PUMPS] = {
    Pump(4), Pump(5), Pump(6), Pump(7), Pump(8), Pump(9),
};

Button butts[NUM_BUTTS] = {
  Button(A2), Button(A3), Button(A4), Button(A5),
};

Beeper beeper(10);

Scale scale(3, 2, -13500);

void beep_err() {
  beeper.beep(400);
}

void beep_succ() {
  beeper.beep(100);
  delay(200);
  beeper.beep(100);
}

void press(int i) {
  float cupWeight = scale.read();
  Serial.print("cupWeight = ");
  Serial.println(cupWeight);

  if(cupWeight < MIN_CUP_WEIGHT) {
    Serial.println("NO CUP");
    beep_err();
    return;
  }

  beep_succ();

  int startTime = millis();

  if(i == 0) {
    pumps[0].turnOn();
    delay(1000);
    pumps[0].turnOff();
  }
  else if(i == 1) {
    pumps[0].turnOn();
    delay(5000);
    pumps[0].turnOff();
  }
  else if(i == 2) {
    pumps[5].turnOn();
    delay(1000);
    pumps[5].turnOff();
  }
  else if(i == 3) {
    pumps[5].turnOn();
    delay(5000);
    pumps[5].turnOff();
  }

  int endTime = millis();

  delay(5000);
  beep_succ();

  float finalWeight = scale.read();
  Serial.print("finalWeight = ");
  Serial.println(finalWeight);

  float drinkWeight = finalWeight - cupWeight;
  Serial.print("drinkWeight = ");
  Serial.println(drinkWeight);

  float pourTime = (endTime - startTime) / 1000.0;
  Serial.print("pourTime = ");
  Serial.print(pourTime);
  Serial.println(" sec");

  float pourSpeed = drinkWeight / pourTime;
  Serial.print("pourSpeed = ");
  Serial.print(pourSpeed);
  Serial.println(" per sec");

  // dispense(0, 2);
  // dispense(5, 6);
  // dispense(2, 6);
}

void dispense(int pump, float amt) {
  int delayFactor = 800;
  int quit = 200;
  int leadTime = 200;
  int lagTime = 3000;
  float dropRate = 0.9;

  float cur = scale.read();
  float target = cur + amt;

  while(true) {
    Serial.print("@ ");
    Serial.print(cur);
    Serial.print(" / ");
    Serial.println(target);

    int delayTime = (target - cur) * delayFactor;

    if(delayTime <= quit || cur > target) {
      Serial.println("pump decay is too low");
      break;
    }

    pumps[pump].turnOn();
    delay(leadTime);
    delay(delayTime);
    pumps[pump].turnOff();
    delay(lagTime);

    cur = scale.read();
    delayFactor *= dropRate;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("booting...");

  scale.init();
  beeper.init();

  for (int i = 0; i < NUM_PUMPS; i++) {
    pumps[i].init();
    Serial.print("pump ");
    Serial.print(i, DEC);
    Serial.println(" ready.");
  }

  for (int i=0; i<NUM_BUTTS; i++) {
    butts[i].init();
  }

  delay(3000);

  scale.tare();

  Serial.println("booted.");

  beep_succ();
}

void loop() {
  for(int i=0; i<NUM_BUTTS; i++) {
    if(butts[i].checkPress() && butts[i].isPressed()) {
      press(i);
    }
  }
  delay(17);
}

int main(void) {
  init();
  setup();
  while (true) {
    loop();
  }
}
