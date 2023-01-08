#include <Arduino.h>
#include "BluetoothSerial.h"

//zakomentuj dla diody ze wspolna katoda
#define COMMON_ANODE

#define BLUE_BUTTON 19
#define YELLOW_BUTTON 32
#define LED_PIN 18
// #define RED_LED_PIN 5
// #define GREEN_LED_PIN 18
// #define BLUE_LED_PIN 23

BluetoothSerial SerialBT;

uint8_t lastStateBlueButton = HIGH;
uint8_t lastStateYellowButton = HIGH;
bool isLedOn = false;
uint8_t ledBrightness = 255;

int delayTime = 20;
char cmd[100];
uint8_t cmdIndex;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;

bool isButtonChangedState(int, uint8_t &);
// <= -1 - change LED state, 0 - set LED OFF, >= 1 - set LED ON
void changeLedState(short int = -1);
void setLedBrightness(uint8_t, bool = true);
bool ledStatusMessage();
void readSerialData();
bool exeCmd();

void setup()
{
  pinMode(BLUE_BUTTON, INPUT_PULLUP);
  pinMode(YELLOW_BUTTON, INPUT_PULLUP);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(LED_PIN, ledChannel);
  ledcWrite(ledChannel, 0);

  Serial.begin(115200);
  SerialBT.begin("Lucid Dreaming Device");
  Serial.println("Urządzenie uruchomione");
}

void loop()
{
  readSerialData();

  if (isButtonChangedState(BLUE_BUTTON, lastStateBlueButton) && lastStateBlueButton == LOW)
    changeLedState();

  isButtonChangedState(YELLOW_BUTTON, lastStateYellowButton);
  if (lastStateYellowButton == LOW && isLedOn)
  {
    setLedBrightness(ledBrightness - 1);
    delay(10);
  }
}

bool isButtonChangedState(int button, uint8_t &lastState)
{
  if (digitalRead(button) != lastState)
  {
    delay(delayTime);
    if (digitalRead(button) != lastState)
    {
      lastState = !lastState;
      return true;
    }
  }
  return false;
}

void changeLedState(short int setLedState)
{
  if ((isLedOn && setLedState <= -1) || setLedState == 0)
  {
    ledcWrite(ledChannel, 0);
    isLedOn = false;
  }
  else
  {
    if (ledBrightness == 0)
      ledBrightness = 255;
    ledcWrite(ledChannel, 255 - ledBrightness);
    isLedOn = true;
  }

  ledStatusMessage();
}

void setLedBrightness(uint8_t value, bool showResult)
{
  #ifdef COMMON_ANODE
    value = 255 - value;
  #endif

  ledBrightness = value;

  if (ledBrightness)
  {
    if (showResult)
      changeLedState(1);
  }
  else
    changeLedState(0);
}

bool ledStatusMessage()
{
  if (isLedOn)
  {
    int percentOfBrightness = ((float)ledBrightness / 255) * 100;
    char text[30];
    sprintf(text, "Diody swiecą na %i%%\n", percentOfBrightness);
    Serial.write(text);
    return true;
  }
  else
  {
    Serial.write("Diody zostały zgaszone\n");
    return false;
  }
}

void readSerialData()
{
  while (SerialBT.available() > 0)
  {
    cmd[cmdIndex] = (char)SerialBT.read();
    Serial.write(cmd[cmdIndex]);
    if (cmd[cmdIndex] == '\n')
    {
      if (cmdIndex == 10)
        exeCmd();

      cmdIndex = 0;
    }
    else if (cmdIndex < 99)
      cmdIndex++;
  }
}

bool exeCmd()
{
  if (cmd[0] == 'x' && cmd[8] == 'x')
  {
    char query[3];
    char charQueryValue[3];
    int intQueryValue;

    for (int i = 0; i < 3; ++i)
    {
      query[i] = cmd[i + 1];
      charQueryValue[i] = cmd[i + 5];
    }
    intQueryValue = atoi(charQueryValue);

    if (strncmp(query, "LED", 3) == 0)
    {
      if (intQueryValue == 0)
        changeLedState(0);
      else if (intQueryValue == 256)
        changeLedState(1);
      else if (intQueryValue > 256)
        changeLedState(-1);
      else if (intQueryValue > 0)
        setLedBrightness(intQueryValue);
      else
        Serial.write("Nieprawidłowa wartość komendy 'LED'.\n");
    }
    else
    {
      Serial.write("Nieprawidłowa komenda.\n");
      return false;
    }

    return true;
  }

  return false;
}