#include <Arduino_FreeRTOS.h> 
#include "SoftwareSerial.h"
#include "DFRobotDFPlayerMini.h"
#include <semphr.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#include "RTClib.h"

/**** Constants ****/
// Car
//#define motorPinsLength = 4;
//const int[motorPinsLength] motorPins = {};
const int enableA = 2;
const int inputA1 = 38;
const int inputA2 = 36;
const int enableB = 3;
const int inputB3 = 47;
const int inputB4 = 45;
const int buttonPin = 32;

// AEB
const int buzzerPin = 25;
const int ultrasonicPinsLength = 1;
const int trigPins[ultrasonicPinsLength] = {29};
const int echoPins[ultrasonicPinsLength] = {28};
const int distanceToBrake = 20;
const int distanceToBuzz = 40;

// GPD
const int lcdRSPin = 10;
const int lcdEnablePin = 11;
const int lcdBitMode = 4;
const int lcdRegistersPins[lcdBitMode] = {4,5,6,7};
LiquidCrystal lcd(lcdRSPin,lcdEnablePin,lcdRegistersPins[0],lcdRegistersPins[1],lcdRegistersPins[2],lcdRegistersPins[3]);
RTC_DS1307 rtc;
const int temperatureSensorPin = A0;
const int joystickXPin = A3;
const int joystickYPin = A2;
const int photoresistorPin = A1;
const int ledPinsLength = 1;
const int ledPins[ledPinsLength] = {33};

// SS
const int playPauseButtonPin = 23;
const int nextButtonPin = 22;
const int previousButtonPin = 24;
const int RXPin = 9;
const int TXPin = 8;
SemaphoreHandle_t playSemaphore;
SoftwareSerial mySoftwareSerial(RXPin,TXPin);
DFRobotDFPlayerMini myDFPlayer;

/**** Global Variables ****/
char currentGear = 'P';
float temperature = 0;
int distanceFront = 999;
int distanceBack = 999;

/**** Method Definitions ****/
// Tasks
void AutoEmergencyBraking( void *pvParameters);
void GeneralPurposeDisplay( void *pvParameters);
void SoundSystem( void *pvParameters);
void WarningSystem( void *pvParameters);
void SoundControl( void *pvParameters);

// CAR Helpers
void moveMotors(int analogValue);
// AEB Helpers
int UltrasonicSensor(int TRIG, int ECHO);
// GPD Helpers
float TemperatureSensor();
boolean Photoresistor(int lightCal);
void LCD();
void Joystick();

// SS Helpers
void playFirstTime();
void play();
void pause();
void playNext();
void playPrevious();


void setup() {
  // Car
  pinMode(enableA, OUTPUT);
  pinMode(inputA1, OUTPUT);
  pinMode(inputA2, OUTPUT);
  pinMode(enableB, OUTPUT);
  pinMode(inputB3, OUTPUT);
  pinMode(inputB4, OUTPUT);
  pinMode(buttonPin, INPUT);

  // AEB
  pinMode(buzzerPin, OUTPUT);
  for(int i=0;i<ultrasonicPinsLength;i++){
    pinMode(trigPins[i],OUTPUT);
  }
  // GPD
  lcd.begin(16,2);
  rtc.begin();
//  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  for(int i=0;i<ledPinsLength;i++){
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  // SS
  pinMode(TXPin, OUTPUT);
  digitalWrite(playPauseButtonPin,HIGH);
  digitalWrite(nextButtonPin,HIGH);
  digitalWrite(previousButtonPin,HIGH);
  playSemaphore= xSemaphoreCreateBinary();
  mySoftwareSerial.begin(9600);
  myDFPlayer.begin(mySoftwareSerial);
  
  Serial.begin(9600);
    
  xTaskCreate (AutoEmergencyBraking, "AEB", 128, NULL, 3, NULL);
  xTaskCreate (GeneralPurposeDisplay, "GPD", 128, NULL, 2, NULL);
  xTaskCreate (SoundSystem, "SS", 128, NULL, 1, NULL);
  xTaskCreate (SoundControl, "SC", 128, NULL, 1, NULL);
  xTaskCreate (WarningSystem, "WS", 128, NULL, 4, NULL);
}

void loop() {
  
}

void AutoEmergencyBraking (void *pvParameters)
{
  TickType_t previousWakeTime;
  previousWakeTime = xTaskGetTickCount();
  int buttonState = HIGH;
  int state = 0;
  while (1) {
    int distanceFront1 = UltrasonicSensor(trigPins[0],echoPins[0]);
    int distanceFront2 = 999;//UltrasonicSensor(trigPins[1],echoPins[1]);
    int distanceFront3 = 999;//UltrasonicSensor(trigPins[2],echoPins[2]);
    distanceFront = min(distanceFront1,min(distanceFront2,distanceFront3));
    distanceBack = 999;//UltrasonicSensor(trigPins[3],echoPins[3]);
    
    if((distanceFront < distanceToBrake && currentGear == 'D') || (distanceBack < distanceToBrake && currentGear == 'R')){
      moveMotors(0);
    }
    else{
      if(currentGear == 'D')
        moveMotors(255);
    }
    if(currentGear != 'D'){
      moveMotors(0);
    }
//    if(digitalRead(buttonPin) != buttonState) {
//        buttonState = digitalRead(buttonPin);
//        state = (state + 1) % 2;
//        delay(50);
//    }
//    if(state == 1)
//        moveMotors(0);
//    else
//        moveMotors(255);
    vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS (150));
  }
}
void GeneralPurposeDisplay (void *pvParameters)
{
  TickType_t previousWakeTime;
  previousWakeTime = xTaskGetTickCount();
  int lightCal = analogRead(photoresistorPin);
  boolean isDark;
  while (1) {
    isDark = Photoresistor(lightCal);
    temperature = TemperatureSensor();
    Joystick();
    if(isDark){
      digitalWrite(ledPins[0], HIGH);
    }
    else{
      digitalWrite(ledPins[0], LOW);
    }
    LCD();
    vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS (1000));
  }
}
void SoundSystem (void *pvParameters)
{
  int playPauseButtonState = HIGH;
  int playPrevState = HIGH;
  bool isPlaying = false;
  bool initialPlay = true;
  while (1) {
    playPauseButtonState = digitalRead(playPauseButtonPin);
    if(playPauseButtonState == !playPrevState && playPauseButtonState == LOW){
      if(initialPlay){
        xSemaphoreGive(playSemaphore);
        playFirstTime();
        isPlaying = true;
        initialPlay = false;
      }
      else{
        if(isPlaying){
          xSemaphoreTake(playSemaphore, portMAX_DELAY);
          pause();
          isPlaying = false;
        }
        else{
          xSemaphoreGive(playSemaphore);
          play();
          isPlaying = true;
        }
      }
    }
    playPrevState = playPauseButtonState;
//    vTaskDelay(pdMS_TO_TICKS (10));
    delay(10);
  }
}
void SoundControl (void *pvParameters)
{
  int nextButtonState = HIGH;
  int previousButtonState = HIGH;
  int nextPrevState = HIGH;
  int prevPrevState = HIGH;
  while(1){
    nextButtonState = digitalRead(nextButtonPin);
    previousButtonState = digitalRead(previousButtonPin);
    xSemaphoreTake(playSemaphore, portMAX_DELAY);
    if(nextButtonState == !nextPrevState && nextButtonState == LOW){
      playNext();
    }
    else if(previousButtonState == !prevPrevState && previousButtonState == LOW){
      playPrevious();
    }
    xSemaphoreGive(playSemaphore);
    nextPrevState = nextButtonState;
    prevPrevState = previousButtonState;
//    vTaskDelay(pdMS_TO_TICKS (10));
    delay(10);
  }
}
void WarningSystem (void *pvParameters)
{
  TickType_t previousWakeTime;
  previousWakeTime = xTaskGetTickCount();
  while (1) {
    if(distanceFront <= distanceToBuzz && currentGear == 'D' && distanceFront > distanceToBrake){
      tone(buzzerPin,880,distanceFront*10);
      vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS (distanceFront*10));
      tone(buzzerPin,0,distanceFront*10);
      vTaskDelayUntil(&previousWakeTime, pdMS_TO_TICKS (distanceFront*10));
    }
    else if(distanceFront <= distanceToBrake && currentGear == 'D'){
      tone(buzzerPin,880);
      vTaskDelay(pdMS_TO_TICKS (distanceFront*10));
    }
    else{
      tone(buzzerPin, 0,1);
      vTaskDelay(pdMS_TO_TICKS (distanceToBuzz*10));
    }
  }
}

// CAR
void moveMotors(int analogValue) {
    digitalWrite(inputA1, HIGH);
    digitalWrite(inputA2, LOW);
    analogWrite(enableA, analogValue);
    digitalWrite(inputB3, HIGH);
    digitalWrite(inputB4, LOW);
    analogWrite(enableB, analogValue);
}

// AEB
int UltrasonicSensor(int TRIG, int ECHO)
{
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG,HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG,LOW);
  delayMicroseconds(2);
  return pulseIn(ECHO,HIGH) * 340 / 20000;
}

// GPD
bool Photoresistor(int lightCal) {
  const int SENSITIVITY = 10;
  int lightVal = analogRead(photoresistorPin);
  if (lightVal < lightCal - SENSITIVITY)
//   if(lightVal < 100)
    return true;
  return false;
}
float TemperatureSensor(){
  float reading = analogRead(temperatureSensorPin) * 0.48828125;
  return reading;
}
void Joystick(){
  int xPinValue = analogRead(joystickXPin);
  int yPinValue = analogRead(joystickYPin);
  if(xPinValue < 100){
    currentGear = 'D';
  }
  else if(xPinValue > 923){
    currentGear = 'R';
  }
  else if(yPinValue < 100){
    currentGear = 'P';
  }
  else if(yPinValue > 923){
    currentGear = 'N';
  }
}
void LCD(){
  lcd.clear();
  DateTime now = rtc.now();
  lcd.print(now.year());
  lcd.print('/');
  lcd.print(now.month());
  lcd.print('/');
  lcd.print(now.day());
  lcd.setCursor(15,0);
  lcd.print(currentGear);
  lcd.setCursor(0,1);
  lcd.print(now.hour());
  lcd.print(':');
  lcd.print(now.minute());
  lcd.print(':');
  lcd.print(now.second());
  lcd.setCursor(12,1);
  lcd.print(temperature);
}

// SS
void playFirstTime(){
  myDFPlayer.volume(15);
  myDFPlayer.play(1);
}
void pause()
{
  myDFPlayer.pause();
}
void play()
{
  myDFPlayer.start();
}
void playNext()
{
  myDFPlayer.next();
}
void playPrevious()
{
  myDFPlayer.previous(); 
}
