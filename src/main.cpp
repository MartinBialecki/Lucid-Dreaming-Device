#include <Arduino.h>
#include <cmath>
#include <Wire.h>
#include "BluetoothSerial.h"
#include <Adafruit_MPU6050.h>

// comment for a diode with a common cathode
// #define COMMON_ANODE

#define BUTTON_PIN 39
#define RED_LED_PIN 23
#define GREEN_LED_PIN 19
#define BLUE_LED_PIN 22
#define LEFT_EYE_SENSOR 34
#define RIGHT_EYE_SENSOR 35
#define SENSORS_POWER 5
#define IR_DIOD_AND_ACCELEROMETER_POWER 17
#define BUZZER 18
#define VIBRATION_MOTOR 16
#define BATTERY_LEVEL_PIN 36
#define I2C_SDA 32
#define I2C_SCL 33

int leftEyeSensorValue = 0;
int rightEyeSensorValue = 0;
int lastLeftEyeSensorValue = 0;
int lastRightEyeSensorValue = 0;
int maxDeviation = 0;                            // największe chwilowe odchylenie z 2 czujników odbiciowych od poprzednich ich wartości
int detectionSensitive = 45;                     // od tej wartości zależy przy jakim odchyleniu od ostatnio zapisanego pomiaru będzie zapisywać się następny dodając też ilość ruchów oczu
int numberOfEyeMovementsToActivatePerMinute = 5; // ilosc wykrytych ruchów gałek ocznych na minutę, które będą miały oznaczać wykrycie fazy snu REM
int numberOfEyeMovementsLeftSensor = 0;
int numberOfEyeMovementsRightSensor = 0;
int lastTimeReduction = 0;

BluetoothSerial SerialBT;
// MPU6050 object
Adafruit_MPU6050 mpu;
// Constant value to convert accelerometer values to gravitational forces
const float GRAVITY_SCALE_FACTOR = 9.81;

uint8_t lastButtonState = HIGH;
unsigned long pressStartTime = 0;
unsigned long pressDuration = 0;
// unsigned long shortPressTime = 0;
unsigned long longPressTime = 2000;
bool shortPressExecuted = false;
bool longPressExecuted = false;
bool taskExecuted = true;

bool REMdetected = false;                      // flaga wykrycia fazy REM, jeśli wartość to "true", to nadawane są sygnały mające na celu pomóc uświadomić się we śnie
unsigned long REMdetectionStartTime = 0;       // czas startu wykrywania fazy REM
float initialDelayTimeInMinutes = 0.2;         // początkowe opóźnienie wykrywania fazy REM w minutach
float additionalDelayTimeInMinutes = 0.1;      // dodatkowe opóźnienie wykrywania fazy REM w minutach
float delayTimeAfterREMdetectionInMinutes = 5; // opóźnienie wykrywania fazy REM w minutach po jej wykryciu
int numberOfSignals = 20;                      // ilość nadawanych sygnałów po wykryciu fazy REM
float signalsCounter = 0;                      // zlicza nadane już sygnały w obecnym cyklu nadawania
int signalLength = 500;                        // długość pojedynczego sygnału w milisekundach
int signalTimer = 0;                           // przechowuje czas do następnego sygnału, pozwala uruchamiać sygnał w określonych odstępach czasu bez wstrzymywania działania urządzenia

bool isLedOn = false;
uint8_t ledBrightness = 20;
bool ledBrightnessWithAccelerometer = true;

int delayTime = 20;
char cmd[100];
uint8_t cmdIndex;

const int freq = 5000;
const int ledChannel[3] = {0, 1, 2};
const int resolution = 8;

int functionNumber = 0;

int batteryValue = 1;

void buttonPressTaskExecutor();
bool ButtonChangedStateMonitor(int, uint8_t &);
// <= -1 - change LED state, 0 - set LED OFF, >= 1 - set LED ON
void changeLedState(short int = -1);
void setLedBrightness(uint8_t, bool = true);
void executLedBrightnes(uint8_t ledBrightness);
bool ledStatusMessage();
void readSerialData();
bool exeCmd();
int batteryLevel(uint8_t batteryLevelPin);
void buttonFunctions(int number);
int batteryLevel(uint8_t batteryLevelPin);
bool devicePositionAndTemperature();
void numberOfEyeMovementsRedution();
void numberOfEyeMovementsAddition();
void sendingSignals();
void shortPressButtonTask();
void longPressButtonTask();

void setup()
{
  // reduce CPU frequency to 80 MHz form default 240 MHz (reduce power consumption from 63 mA to 30 mA)
  setCpuFrequencyMhz(80);

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LEFT_EYE_SENSOR, INPUT);
  pinMode(RIGHT_EYE_SENSOR, INPUT);
  pinMode(BATTERY_LEVEL_PIN, INPUT);

  pinMode(SENSORS_POWER, OUTPUT);
  digitalWrite(SENSORS_POWER, HIGH);
  pinMode(IR_DIOD_AND_ACCELEROMETER_POWER, OUTPUT);
  digitalWrite(IR_DIOD_AND_ACCELEROMETER_POWER, HIGH);
  pinMode(VIBRATION_MOTOR, OUTPUT);
  digitalWrite(VIBRATION_MOTOR, LOW);

  ledcSetup(ledChannel[0], freq, resolution);
  ledcAttachPin(RED_LED_PIN, ledChannel[0]);
  ledcSetup(ledChannel[1], freq, resolution);
  ledcAttachPin(GREEN_LED_PIN, ledChannel[1]);
  ledcSetup(ledChannel[2], freq, resolution);
  ledcAttachPin(BLUE_LED_PIN, ledChannel[2]);

  Serial.begin(9600);
  SerialBT.begin("Lucid Dreaming Device");
  SerialBT.println("Urządzenie uruchomione");
  Serial.println("Urządzenie uruchomione");

  batteryValue = batteryLevel(BATTERY_LEVEL_PIN);
  Serial.print(batteryValue);
  Serial.println("%");
  SerialBT.print(batteryValue);
  SerialBT.println("%");

  if (batteryValue < 2)
  {
    changeLedState(1);
    delay(300);
    changeLedState(0);
    delay(300);
    changeLedState(1);
    delay(300);
    Serial.println("Niski poziom baterii. Uruchomienie trybu głębokiego snu.");
    SerialBT.println("Niski poziom baterii. Uruchomienie trybu głębokiego snu.");
    esp_deep_sleep_start();
  }

  changeLedState(1);
  digitalWrite(VIBRATION_MOTOR, HIGH);
  digitalWrite(BUZZER, HIGH);
  delay(20);
  digitalWrite(BUZZER, LOW);
  delay(300);
  changeLedState(0);
  digitalWrite(VIBRATION_MOTOR, LOW);
  delay(300);
  changeLedState(1);
  digitalWrite(VIBRATION_MOTOR, HIGH);
  delay(300);
  changeLedState(0);
  digitalWrite(VIBRATION_MOTOR, LOW);

  Wire.begin(I2C_SDA, I2C_SCL);

  // Inicjalizacja MPU6050
  if (!mpu.begin())
  {
    Serial.println("Nie znaleziono układu MPU6050. Sprawdź połączenia!");
  }

  REMdetectionStartTime = millis() + (initialDelayTimeInMinutes * 60 * 1000);

  // devicePositionAndTemperature();
}

void loop()
{
  readSerialData();
  buttonPressTaskExecutor();

  numberOfEyeMovementsRedution();
  numberOfEyeMovementsAddition();

  sendingSignals();

  // if (isButtonChangedState(BUTTON_PIN, lastButtonState) && lastButtonState == LOW)
  // {
  //   if(functionNumber >= 4)
  //     functionNumber = 0;
  //   else
  //     functionNumber++;

  //   buttonFunctions(functionNumber);
  // }

  // if(isLedOn && ledBrightnessWithAccelerometer)
  //   executLedBrightnes(ledBrightness);
}

void buttonPressTaskExecutor()
{
  ButtonChangedStateMonitor(BUTTON_PIN, lastButtonState);

  if (longPressExecuted == true)
  {
    taskExecuted = true;
    longPressButtonTask();
  }

  if (shortPressExecuted == true && taskExecuted == false)
  {
    taskExecuted = true;
    shortPressButtonTask();
  }
}

bool ButtonChangedStateMonitor(int button, uint8_t &lastState)
{
  if (lastState == LOW)
  {
    if (pressStartTime == 0)
    {
      pressStartTime = millis();
      taskExecuted = false;
      longPressExecuted = false;
      shortPressExecuted = false;
    }
    else
      pressDuration = millis() - pressStartTime;
  }
  else
  {
    if (taskExecuted == false)
    {
      if (pressDuration >= longPressTime)
        longPressExecuted = true;
      else
        shortPressExecuted = true;
    }
    // Zresetuj licznik czasu jeśli przycisk nie jest wciśnięty
    pressStartTime = 0;
    pressDuration = 0;
  }

  if (longPressExecuted == false && pressDuration >= longPressTime)
  {
    longPressExecuted = true;
  }

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
    executLedBrightnes(0);
    isLedOn = false;
  }
  else
  {
    if (ledBrightness == 0)
      ledBrightness = 255;
    executLedBrightnes(ledBrightness);
    isLedOn = true;
  }

  ledStatusMessage();
}

void setLedBrightness(uint8_t value, bool showResult)
{
  ledBrightness = value;

  if (ledBrightness)
  {
    if (showResult)
      changeLedState(1);
  }
  else
    changeLedState(0);
}

void executLedBrightnes(uint8_t ledBrightness)
{
#ifdef COMMON_ANODE
  ledBrightness = 255 - ledBrightness;
#endif

  if (ledBrightnessWithAccelerometer)
  {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    ledcWrite(ledChannel[0], ledBrightness * (std::abs(a.acceleration.x) / 11));
    ledcWrite(ledChannel[1], ledBrightness * (std::abs(a.acceleration.y) / 11));
    ledcWrite(ledChannel[2], ledBrightness * (std::abs(a.acceleration.z) / 11));
  }
  else
  {
    for (int i = 0; i <= 2; i++)
      ledcWrite(ledChannel[i], ledBrightness);
  }
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
      {
        SerialBT.println("Nieprawidłowa wartość komendy 'LED'.");
        return false;
      }
    }
    else if (strncmp(query, "VIB", 3) == 0)
    {
      if (intQueryValue == 0)
        digitalWrite(VIBRATION_MOTOR, LOW);
      else if (intQueryValue == 256)
        digitalWrite(VIBRATION_MOTOR, HIGH);
      else if (intQueryValue > 256)
        digitalWrite(VIBRATION_MOTOR, !digitalRead(VIBRATION_MOTOR));
      else
      {
        SerialBT.println("Nieprawidłowa wartość komendy 'VIB'.");
        return false;
      }
    }
    else if (strncmp(query, "BUZ", 3) == 0)
    {
      if (intQueryValue == 0)
        digitalWrite(BUZZER, LOW);
      else if (intQueryValue == 256)
        digitalWrite(BUZZER, HIGH);
      else if (intQueryValue > 256)
        digitalWrite(BUZZER, !digitalRead(BUZZER));
      else
      {
        SerialBT.println("Nieprawidłowa wartość komendy 'BUZ'.");
        return false;
      }
    }
    else if (strncmp(query, "GYR", 3) == 0)
    {
      if (intQueryValue == 0)
        ledBrightnessWithAccelerometer = false;
      else if (intQueryValue == 256)
        ledBrightnessWithAccelerometer = true;
      else if (intQueryValue > 256)
        ledBrightnessWithAccelerometer = !ledBrightnessWithAccelerometer;
      else
      {
        SerialBT.println("Nieprawidłowa wartość komendy 'GYR'.");
        return false;
      }
    }
    else if (strncmp(query, "ALL", 3) == 0)
    {
      if (intQueryValue > 256)
      {
        if (isLedOn || digitalRead(BUZZER) || digitalRead(VIBRATION_MOTOR))
        {
          changeLedState(0);
          digitalWrite(VIBRATION_MOTOR, LOW);
          digitalWrite(BUZZER, LOW);
        }
        else
        {
          changeLedState(1);
          digitalWrite(VIBRATION_MOTOR, HIGH);
          digitalWrite(BUZZER, HIGH);
        }
      }
      else
      {
        SerialBT.println("Nieprawidłowa wartość komendy 'ALL'.");
        return false;
      }
    }
    else
    {
      SerialBT.println("Nieprawidłowa komenda.");
      return false;
    }

    batteryValue = batteryLevel(BATTERY_LEVEL_PIN);
    Serial.print(batteryValue);
    Serial.println("%");
    SerialBT.print(batteryValue);
    SerialBT.println("%");
    devicePositionAndTemperature();

    return true;
  }

  return false;
}

void buttonFunctions(int number)
{
  changeLedState(0);
  digitalWrite(BUZZER, LOW);
  digitalWrite(VIBRATION_MOTOR, LOW);

  switch (number)
  {
  case 1:
    digitalWrite(VIBRATION_MOTOR, HIGH);
    delay(100);
    tone(BUZZER, 1000, 500);
    break;
  case 2:
    changeLedState(1);
    break;
  case 3:
    digitalWrite(BUZZER, HIGH);
    break;
  case 4:
    digitalWrite(VIBRATION_MOTOR, HIGH);
    break;
  default:
    break;
  }

  batteryValue = batteryLevel(BATTERY_LEVEL_PIN);
  Serial.print(batteryValue);
  Serial.println("%");
  SerialBT.print(batteryValue);
  SerialBT.println("%");
  // devicePositionAndTemperature();
}

int batteryLevel(uint8_t batteryLevelPin)
{
  // int calibration = 217; //(before the changes) ((0.35 * (4096 / 3.3)) / 2); calibration due to differences in readings when measuring with a multimeter, the difference was about 0.36 volts
  int calibration = 87;  //((0.14 * (4096 / 3.3)) / 2); calibration due to differences in readings when measuring with a multimeter, the difference was about 0.36 volts
  int batteryMax = 2606; //((4.2 * 0.5)*(4096 / 3.3))
  int batteryMin = 2172; //((3.5 * 0.5)*(4096 / 3.3))
  float onePercentOfBattery = (batteryMax - batteryMin) / 100.0;
  int sum = 0;
  int batteryLevelPinValue;

  for (int i = 0; i < 20; i++)
  {
    sum += analogRead(batteryLevelPin);
    delay(5);
  }

  batteryLevelPinValue = (sum / 20) + calibration;
  // Serial.println(batteryLevelPinValue);
  Serial.print((batteryLevelPinValue / (4096 / 3.3)) * 2);
  Serial.print("V - ");
  SerialBT.print((batteryLevelPinValue / (4096 / 3.3)) * 2);
  SerialBT.print("V - ");
  if (batteryLevelPinValue < batteryMin + onePercentOfBattery)
    return 1;
  else if (batteryLevelPinValue > batteryMax - onePercentOfBattery)
    return 100;
  else
    return (batteryLevelPinValue - batteryMin) / onePercentOfBattery;
}

bool devicePositionAndTemperature()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Wypisanie wartości na port szeregowy
  Serial.print("Akcelerometr (m/s^2): x = ");
  Serial.print(a.acceleration.x);
  Serial.print(", y = ");
  Serial.print(a.acceleration.y);
  Serial.print(", z = ");
  Serial.print(a.acceleration.z);
  Serial.print(".\nTemperatura: ");
  Serial.print(temp.temperature);
  Serial.println("C.");

  SerialBT.print("Akcelerometr (m/s^2): x = ");
  SerialBT.print(a.acceleration.x);
  SerialBT.print(", y = ");
  SerialBT.print(a.acceleration.y);
  SerialBT.print(", z = ");
  SerialBT.print(a.acceleration.z);
  SerialBT.print(".\nTemperatura: ");
  SerialBT.print(temp.temperature);
  SerialBT.println("C.");

  return true;
}

void numberOfEyeMovementsRedution()
{
  if ((lastTimeReduction + (60000 / numberOfEyeMovementsToActivatePerMinute)) <= millis())
  {
    if (numberOfEyeMovementsLeftSensor > 0)
      numberOfEyeMovementsLeftSensor -= 1;
    if (numberOfEyeMovementsRightSensor > 0)
      numberOfEyeMovementsRightSensor -= 1;

    if ((lastTimeReduction + (2 * (60000 / numberOfEyeMovementsToActivatePerMinute))) < millis())
      lastTimeReduction = millis();
    else
      lastTimeReduction += (60000 / numberOfEyeMovementsToActivatePerMinute);
  }
}

void numberOfEyeMovementsAddition()
{
  if (REMdetectionStartTime <= millis())
  {
    leftEyeSensorValue = analogRead(LEFT_EYE_SENSOR);
    rightEyeSensorValue = analogRead(RIGHT_EYE_SENSOR);

    if (abs(leftEyeSensorValue - lastLeftEyeSensorValue) >= detectionSensitive)
    {
      numberOfEyeMovementsLeftSensor += 1;
      lastLeftEyeSensorValue = leftEyeSensorValue;
    }
    if (abs(rightEyeSensorValue - lastRightEyeSensorValue) >= detectionSensitive)
    {
      numberOfEyeMovementsRightSensor += 1;
      lastRightEyeSensorValue = rightEyeSensorValue;
    }

    if (numberOfEyeMovementsLeftSensor >= numberOfEyeMovementsToActivatePerMinute || numberOfEyeMovementsRightSensor >= numberOfEyeMovementsToActivatePerMinute)
    {
      REMdetectionStartTime = millis() + (delayTimeAfterREMdetectionInMinutes * 60000);
      REMdetected = true;
    }
    else
      REMdetectionStartTime = millis() + 100;
  }
}

void sendingSignals()
{
  if (REMdetected)
  {
    if (signalTimer <= millis())
    {
      changeLedState(-1);
      signalsCounter += 0.5;
      signalTimer = millis() + signalLength;
    }

    if (signalsCounter >= numberOfSignals)
    {
      changeLedState(0);
      REMdetected = false;
    }
  }
  else
    signalsCounter = 0;
}

void shortPressButtonTask()
{
  REMdetected = false;
  if (REMdetectionStartTime > millis())
    REMdetectionStartTime += (additionalDelayTimeInMinutes * 60 * 1000);
  else
    REMdetectionStartTime = millis() + (additionalDelayTimeInMinutes * 60 * 1000);

  digitalWrite(BUZZER, HIGH);
  digitalWrite(VIBRATION_MOTOR, HIGH);
  changeLedState(1);
  delay(20);
  digitalWrite(BUZZER, LOW);
  delay(30);
  digitalWrite(VIBRATION_MOTOR, LOW);
  delay(100);
  changeLedState(0);

  batteryValue = batteryLevel(BATTERY_LEVEL_PIN);
  Serial.print(batteryValue);
  Serial.println("%");
  SerialBT.print(batteryValue);
  SerialBT.println("%");

  Serial.print("Czas do rozpoczęcia monitorowania ruchów gałek ocznych w minutach: ");
  Serial.println((REMdetectionStartTime - millis()) / 60000);
  SerialBT.print("Czas do rozpoczęcia monitorowania ruchów gałek ocznych w minutach: ");
  SerialBT.println((REMdetectionStartTime - millis()) / 60000);
}

void longPressButtonTask()
{
  REMdetected = false;
  changeLedState(0);

  REMdetectionStartTime = millis() + (initialDelayTimeInMinutes * 60 * 1000);

  leftEyeSensorValue = analogRead(LEFT_EYE_SENSOR);
  rightEyeSensorValue = analogRead(RIGHT_EYE_SENSOR);

  if (abs(leftEyeSensorValue - lastLeftEyeSensorValue) >= abs(rightEyeSensorValue - lastRightEyeSensorValue))
    maxDeviation = abs(leftEyeSensorValue - lastLeftEyeSensorValue);
  else
    maxDeviation = abs(rightEyeSensorValue - lastRightEyeSensorValue);

  if (maxDeviation >= (detectionSensitive / 3))
  {
    lastLeftEyeSensorValue = leftEyeSensorValue;
    lastRightEyeSensorValue = rightEyeSensorValue;

    Serial.print("Odnotowane odchylenie: ");
    Serial.println(maxDeviation);
    SerialBT.print("Odnotowane odchylenie: ");
    SerialBT.println(maxDeviation);

    digitalWrite(BUZZER, HIGH);
    if (maxDeviation < detectionSensitive)
      delay(100 * (maxDeviation / detectionSensitive));
    else
      delay(120);
    digitalWrite(BUZZER, LOW);
  }

  delay(250);
}
