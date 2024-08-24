#include <Arduino.h>

#define PIN1A	2
#define PIN1B	3

#define PIN2A	18
#define PIN2B	19
volatile long lastC1, lastC2, counter1 = 0, counter2 = 0; // This variable will increase or decrease depending on the rotation of encoder
// See https://electricdiylab.com/how-to-connect-optical-rotary-encoder-with-arduino/

unsigned int pushButtonPins[] = {4, 5, 6, 7, 8};
int pushButtonStates[] = {0, 0, 0, 0, 0};
// unsigned long lastDebounceTime[] = {0, 0, 0, 0, 0};
// unsigned int debounceDelay = 50;
const int nButtons = 5;

void setup() {
  
  Serial.begin(115200);

  for (int i = 0; i < nButtons; i++) {
    pinMode(pushButtonPins[i], INPUT_PULLUP);
  }
  
  pinMode(PIN1A, INPUT_PULLUP);
  pinMode(PIN1B, INPUT_PULLUP);

  pinMode(PIN2A, INPUT_PULLUP);
  pinMode(PIN2B, INPUT_PULLUP);
  
  // A Rising pulse from encoder activates ia0()
  attachInterrupt(digitalPinToInterrupt(PIN1A), i1A, RISING);
  // B Rising pulse from encoder activates ia1()
  attachInterrupt(digitalPinToInterrupt(PIN1B), i1B, RISING);

  attachInterrupt(digitalPinToInterrupt(PIN2A), i2A, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN2B), i2B, RISING);
}

// int sendButtonState(int i) {
//   int currentBtnState = digitalRead(pushButtonPins[i]);
  
//   if (currentBtnState != pushButtonStates[i]) {
//     lastDebounceTime[i] = millis();
//   } 
//   if(lastDebounceTime[i] > 0 && (millis() - lastDebounceTime[i]) > debounceDelay) {
//     pushButtonStates[i] = currentBtnState;
//     Serial.println("Button" + String(i+1) + ": " + String(currentBtnState));
//     lastDebounceTime[i] = 0;
//   }
// }

void loop() {
  if (counter1 != lastC1 || counter2 != lastC2)
  {
    Serial.println("Encoders: " + String(counter1) + "," + String(counter2));
    lastC1 = counter1;
    lastC2 = counter2;
  }

  for (int i = 0; i < nButtons; i++) {
    // sendButtonState(i);
    int currentBtnState = digitalRead(pushButtonPins[i]);
    if(pushButtonStates[i] != currentBtnState) {
      Serial.println("Button" + String(i+1) + ": " + String(pushButtonStates[i])); // Send the previous state because buttons are 1 when released and 0 when pressed
      pushButtonStates[i] = currentBtnState;
    }
  }
}

// i1A is activated if DigitalPin PIN1A is going from LOW to HIGH
// Check PIN1B to determine the direction
void i1A() {
  counter1 += digitalRead(PIN1B) == LOW ? 1 : -1;
}

void i1B() {
  counter1 += digitalRead(PIN1A) == LOW ? -1 : +1;
}

void i2A() {
  counter2 += digitalRead(PIN2B) == LOW ? 1 : -1;
}

void i2B() {
  counter2 += digitalRead(PIN2A) == LOW ? -1 : +1;
}
