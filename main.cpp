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
#define PUMP_HISTORY 4

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

  // a circular buffer for storing the recent pump durations + amounts dispensed
  float pumpTimes[PUMP_HISTORY];
  float pumpAmounts[PUMP_HISTORY];
  int historyIndex;

public:
  Pump(int pin) : pin(pin), on(false), historyIndex(0) {
    for(int i=0; i<PUMP_HISTORY; i++) {
      this->pumpTimes[i] = 1000.0;
      this->pumpAmounts[i] = 2.0;
    }
  }

  void init() {
    pinMode(this->pin, OUTPUT);
    this->turnOff();
  }

  // record a duration + amount dispensed and move the history index forwards
  void recordMeasure(float time, float amount) {
    Serial.print("recording measure: ");
    Serial.print(amount);
    Serial.print(" in ");
    Serial.print(time);
    Serial.print("ms; current average pump rate = ");
    Serial.println(this->pumpRate());

    this->pumpTimes[this->historyIndex] = time;
    this->pumpAmounts[this->historyIndex] = amount;
    this->historyIndex = (this->historyIndex + 1) % PUMP_HISTORY;
  }

  // compute the average pump rate as (ms per oz) for the recent history
  float pumpRate() {
    float totalAmount = 0.0;
    float totalTime = 0.0;
    for(int i=0; i<PUMP_HISTORY; i++) {
      totalAmount += this->pumpAmounts[i];
      totalTime += this->pumpTimes[i];
    }

    return totalTime / totalAmount;
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

void dispense(int pump, float amt) {
  float cur = scale.read();
  float target = cur + amt;
  int lagTime = 5000;

  Serial.print("start dispense: ");
  Serial.print(cur);
  Serial.print(" / ");
  Serial.println(target);

  while(true) {
    int startTime = millis();
    float remain = (target - cur);

    Serial.print("need to pour ");
    Serial.println(remain);

    // it's close enough, quit out
    if(remain < 0.2) {
      break;
    }

    // go for broke
    else if(remain <= 3) {
      int delayTime = (int)(pumps[pump].pumpRate() * remain);
      pumps[pump].turnOn();
      delay(delayTime);
      pumps[pump].turnOff();
    }

    // try to pour until 2oz away
    else {
      int delayTime = (int)(pumps[pump].pumpRate() * (remain - 2.0));
      pumps[pump].turnOn();
      delay(delayTime);
      pumps[pump].turnOff();
    }

    int endTime = millis();
    delay(lagTime);
    float end = scale.read();
    pumps[pump].recordMeasure(endTime - startTime, end - cur);
    cur = end;

    Serial.print("  mid dispense: ");
    Serial.print(cur);
    Serial.print(" / ");
    Serial.println(target);
  }

  Serial.print("final dispense: ");
  Serial.print(cur);
  Serial.print(" / ");
  Serial.println(target);
}

#define CRANBERRY 0
#define VODKA 1
#define CIDER 2
#define BOURBON 3
#define GINGER 4
#define LIME 5

void dispenseCranberryVodka() {
  Serial.println("Cranberry vodka");
  dispense(VODKA, 2);
  dispense(CRANBERRY, 6);
}

void dispenseMule() {
  Serial.println("Mule");
  dispense(VODKA, 2);
  dispense(GINGER, 3);
  dispense(LIME, 0.5);
  dispense(GINGER, 3);
}

void dispenseBourbonCider() {
  Serial.println("Bourbon cider");
  dispense(BOURBON, 2);
  dispense(CIDER, 6);
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

  if(i == 3) {
    i = random(3);
    Serial.print("Dealer's choice: ");
    Serial.println(i, DEC);
  }

  if(i == 0) {
    dispenseMule();
  }
  else if(i == 1) {
    dispenseCranberryVodka();
  }
  else if(i == 2) {
    dispenseBourbonCider();
  }

  beep_succ();

  float finalWeight = scale.read();
  Serial.print("finalWeight = ");
  Serial.println(finalWeight);

  float drinkWeight = finalWeight - cupWeight;
  Serial.print("drinkWeight = ");
  Serial.println(drinkWeight);
}

void setup() {
  Serial.begin(9600);
  Serial.println("booting...");

  scale.init();
  Serial.println("scale ready.");

  beeper.init();
  Serial.println("beeper ready.");

  for (int i = 0; i < NUM_PUMPS; i++) {
    pumps[i].init();
  }
  Serial.println("pumps ready.");

  for (int i=0; i<NUM_BUTTS; i++) {
    butts[i].init();
  }
  Serial.println("buttons ready.");

  Serial.println("waiting for scale to stabilize....");
  delay(3000);
  scale.tare();
  Serial.println("scale tared.");

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
