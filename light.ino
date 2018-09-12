// Bicycle LED ligth with low power consumption when off
// ATINY 13A

#include <avr/sleep.h>
#include <avr/power.h>

#define UINT8_MAX 255

enum StateType {
  STATE_OFF = 0,
  STATE_FADE_OFF,
  STATE_ON,
  STATE_FADE_ON,
  STATE_BLINK,
};

// only pin 1 support external interupt
const uint8_t buttonPin = 1;
// pins 0,1 supports PWM, so only 0 is free
const uint8_t ledPin = 0;

uint8_t state = STATE_OFF;
uint8_t brightness = 0;
uint8_t maxBrightness = UINT8_MAX;
uint8_t blinkSpeed = 0;
uint8_t holdCount = 0;

bool buttonPressed = 0;

void wakeUpFromSleep()
{
  detachInterrupt(digitalPinToInterrupt(buttonPin));
  buttonChanged();
}

void sleepPwrDown(uint8_t ledState)
{
  // wait until button is released because there is ~1 second delay
  // when waking from interrupt while button is pressed
  if (buttonPressed && state != STATE_BLINK)
    return;
  // disable PWM because clocks are not active during sleep
  digitalWrite(ledPin, ledState);
  if (ledState == LOW)
    // turn off BOD when during sleep when led is off to save more power
    sleep_bod_disable();
  // go to the deepest of sleeps
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  // attach pull up interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), wakeUpFromSleep, buttonPressed);
  // sleep now
  sleep_mode();
}

void sleepIdle()
{
  // wait untill button is released
  if (buttonPressed)
    return;
  // PWM is not working during PWR_DOWN sleep mode
  set_sleep_mode(SLEEP_MODE_IDLE);
  // attach pull up interrupt
  attachInterrupt(digitalPinToInterrupt(buttonPin), wakeUpFromSleep, buttonPressed);
  // sleep now
  sleep_mode();
}

void setWatchdogTimer(uint8_t value)
{
  noInterrupts();
  WDTCR |= (1 << WDTIE) | (1 << WDCE);
  WDTCR = value;
  interrupts();
}

void startHoldTimer()
{
  if (state == STATE_BLINK) {
    switch (blinkSpeed) {
      case 0: holdCount = 64; break;
      case 1: holdCount = 32; break;
      case 2: holdCount = 16; break;
      case 3: holdCount = 8; break;
      case 4: holdCount = 4; break;
      case 5: holdCount = 2; break;
      default: holdCount = 1; break;
    }
  } else {
    // start timer for hold event for 1 second
    setWatchdogTimer((1 << WDTIE) | (1 << WDP2) | (1 << WDP1));
  }
}

void stopHoldTimer()
{
  if (state == STATE_BLINK) {
    holdCount = 0;
  } else {
    setWatchdogTimer(0);
  }
}

void blinkOnOff()
{
  bool ledOn = digitalRead(ledPin);

  if (!ledOn)
    // start blink timer for 0.032 seconds
    setWatchdogTimer((1 << WDTIE) | (1 << WDP0));
  else
    // start off timer up to 8 seconds
    setWatchdogTimer((1 << WDTIE) | (blinkSpeed & 7) | ((blinkSpeed & 8) << 2));

  sleepPwrDown(!ledOn);
}

ISR(WDT_vect)
{
  if (!buttonPressed)
    return;

  // filter out blink watchdog timeouts
  if (state == STATE_BLINK) {
    if (holdCount)
      --holdCount;
    else
      return;
    if (holdCount)
      return;
  }
  holdEvent();
  // continue monitor hold event
  startHoldTimer();
}

void enterState(StateType newState)
{
  switch (newState) {
    case STATE_OFF:
      blinkSpeed = 0;
      break;
    case STATE_FADE_OFF:
      maxBrightness = 0;
      break;
    case STATE_ON:
      if (state == STATE_FADE_OFF)
        maxBrightness = brightness;
      break;
    case STATE_FADE_ON:
      maxBrightness = UINT8_MAX;
      break;
    case STATE_BLINK:
      brightness = 0;
      if (state == STATE_BLINK)
        ++blinkSpeed;
      else
        blinkSpeed = 0;
      if (blinkSpeed > 9)
        blinkSpeed = 9;
      break;
  }
  state = newState;
}

void pressEvent()
{
  switch (state) {
    case STATE_OFF:
      enterState(STATE_FADE_ON);
      break;
    case STATE_BLINK:
      enterState(STATE_OFF);
      break;
  }
}

void holdEvent()
{
  switch (state) {
    case STATE_ON:
      enterState(STATE_FADE_OFF);
      break;
    case STATE_FADE_ON:
    case STATE_BLINK:
      enterState(STATE_BLINK);
      break;
  }
}

void releaseEvent()
{
  switch (state) {
    case STATE_ON:
      enterState(STATE_OFF);
      break;
    case STATE_FADE_OFF:
    case STATE_FADE_ON:
      enterState(STATE_ON);
      break;
  }
}

void buttonChanged()
{
  bool newButtonPressed = !digitalRead(buttonPin);
  if (buttonPressed != newButtonPressed) {
    buttonPressed = newButtonPressed;
    if (buttonPressed) {
      startHoldTimer();
      pressEvent();
    } else {
      stopHoldTimer();
      releaseEvent();
    }
  }
}

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  power_adc_disable();
}

void loop()
{
  buttonChanged();

  if (state == STATE_BLINK) {
    blinkOnOff();

  } else if (state && brightness < maxBrightness) {
    // increase brightness
    analogWrite(ledPin, ++brightness);

  } else if ((!state && brightness > 0) || (brightness > maxBrightness)) {
    // decrease brightness
    analogWrite(ledPin, --brightness);

  } else if (brightness == 0) {
    enterState(STATE_OFF);
    sleepPwrDown(LOW);

  } else if (brightness == UINT8_MAX) {
    sleepPwrDown(HIGH);

  } else if (brightness == maxBrightness) {
    sleepIdle();
  }

  // increase brightness exponencionally
  for (uint8_t n = UINT8_MAX; n > brightness; --n) {
    if (state == STATE_FADE_OFF)
      delayMicroseconds(80);
    else
      delayMicroseconds(20);
    buttonChanged();
  }
}

