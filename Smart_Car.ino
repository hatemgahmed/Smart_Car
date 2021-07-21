#include <Arduino_FreeRTOS.h> 
#include <semphr.h> // add to be able to use semaphores
#include <Wire.h>
#include <DS3231.h>
#include "dht.h" 
#define dht_apin A0 
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 
DS3231  rtc(SDA, SCL);
dht DHT;
#define REMOTEXY_MODE__HARDSERIAL
#include <LiquidCrystal.h> // includes the LiquidCrystal Library
#include <RemoteXY.h>
#define dht_apin A0 // Analog Pin sensor is connected to A0
// RemoteXY connection settings 
#define REMOTEXY_SERIAL Serial3
#define REMOTEXY_SERIAL_SPEED 9600

const int contrast=75;
byte PWMA=0;
byte PWMB=0;
 
LiquidCrystal lcd(1, 2, 4, 5, 6, 7); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7) 
String gear="";
// RemoteXY configurate  
#pragma pack(push, 1)
uint8_t RemoteXY_CONF[] =
  { 255,4,0,0,0,32,0,8,1,0,
  5,0,34,9,30,30,2,26,31,1,
  0,5,44,12,12,2,31,60,0,1,
  0,76,43,12,12,2,31,62,0 };
  
// this structure defines all the variables of your control interface 
struct {

    // input variable
  int8_t joystick_1_x; // =-100..100 x-coordinate joystick position 
  int8_t joystick_1_y; // =-100..100 y-coordinate joystick position 
  uint8_t button_Left; // =1 if button pressed, else =0 
  uint8_t button_Right; // =1 if button pressed, else =0 

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0 

} RemoteXY;
#pragma pack(pop)

int ledPin = 25;
int lightSensorPin = A1;
double analogValue = 0;

#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         13         // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above
#define buttonPin 27
int prevState=0;
int buttonState = 0;
int onState=0;
int keyEntered=0;
const int enableBluetooth=49;

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

#define enA 12 //PWM
#define inA1 10
#define inA2 11
#define enB 3 //PWM
#define inB1 26
#define inB2 8
#define trigPin 23
#define echoPin 22
#define buzzerPin 24

int rotDirection = 0;
long duration;
int distance;

SemaphoreHandle_t Sem0;
SemaphoreHandle_t Sem1;
SemaphoreHandle_t Sem2;
SemaphoreHandle_t Sem3;

// define two Tasks 
void TaskDisplay( void *pvParameters); 
void TaskBluetooth( void *pvParameters);
void TaskLight( void *pvParameters);
void TaskStart( void *pvParameters);
void TaskMotor( void *pvParameters);

void setup() {
  Sem0=xSemaphoreCreateCounting(1,0);
  Sem1=xSemaphoreCreateCounting(20,0);
  Sem2=xSemaphoreCreateCounting(10,0);
  Sem3=xSemaphoreCreateCounting(20,0);
  digitalWrite(25,HIGH);
  analogWrite(9,contrast);
  rtc.begin(); // Initialize the rtc object
//  rtc.setDOW(MONDAY);     // Set Day-of-Week to SUNDAY
//  rtc.setTime(16, 11, 0);     // Set the time to 12:00:00 (24hr format)
//  rtc.setDate(2, 12, 2019);   // Set the date to January 1st, 2014
  RemoteXY_Init (); 
  lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 
  pinMode(ledPin, OUTPUT);
  pinMode(enableBluetooth,OUTPUT);
  pinMode(buttonPin,INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  delay(500);
  SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  // Set initial rotation direction
  digitalWrite(inA1, LOW);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, LOW);
  digitalWrite(inB2, HIGH);
  digitalWrite(enableBluetooth,LOW);
//  Serial.begin(9600);
  xTaskCreate (TaskDisplay, "Display", 128, NULL, 1, NULL);
  xTaskCreate (TaskBluetooth, "Bluetooth", 128, NULL, 3, NULL);
  xTaskCreate (TaskLight, "Light", 128, NULL, 1, NULL);
  xTaskCreate (TaskStart, "Keyless", 128, NULL, 5, NULL);
  xTaskCreate (TaskMotor, "Motor", 128, NULL, 4, NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
void TaskDisplay (void *pvParameters) // This is a Task. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(1000);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    lcd.clear();
    xSemaphoreTake(Sem0,portMAX_DELAY);
//    Serial.println("Display");
     //lcd.noDisplay();
     lcd.setCursor(0,0);
     lcd.print(rtc.getTimeStr());
     lcd.setCursor(0,1);
     lcd.print(rtc.getDateStr());
      lcd.setCursor(10,0);
      DHT.read11(dht_apin);
      lcd.print(DHT.temperature);
      lcd.print('C'); 
      lcd.setCursor(15, 1);
      lcd.print(gear);
//      lcd.setCursor(13, 1);
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void TaskBluetooth (void *pvParameters) // This is a Task. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(100);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    xSemaphoreTake(Sem1,portMAX_DELAY);
//    Serial.println("Bluetooth");
     RemoteXY_Handler ();
     if( RemoteXY.joystick_1_x == -100){
        gear="N";
        PWMA=0;
        PWMB=0;
      } else if( RemoteXY.joystick_1_y == 100){
        digitalWrite(inA1, LOW);
        digitalWrite(inA2, HIGH);
        digitalWrite(inB1, LOW);
        digitalWrite(inB2, HIGH);
        gear="D";
        PWMA=255;
        PWMB=255;
        if(RemoteXY.button_Left==HIGH)
          PWMA=100;
        if(RemoteXY.button_Right==HIGH)
          PWMB=100;
      } else if( RemoteXY.joystick_1_y == -100){
        gear="R";
        PWMA=255;
        PWMB=255;
        digitalWrite(inA1, HIGH);
        digitalWrite(inA2, LOW);
        digitalWrite(inB1, HIGH);
        digitalWrite(inB2, LOW);
        if(RemoteXY.button_Left==HIGH)
          PWMA=100;
        if(RemoteXY.button_Right==HIGH)
          PWMB=100;
      } else if( RemoteXY.joystick_1_x == 100 || (RemoteXY.joystick_1_x == 0 && RemoteXY.joystick_1_y == 0)){
        gear="P";
        PWMA=0;
        PWMB=0;
      }
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void TaskLight (void *pvParameters) // This is a Task. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(1500);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
    digitalWrite(ledPin, LOW);
    xSemaphoreTake(Sem2,portMAX_DELAY);
//    Serial.println("Light");
    analogValue = analogRead(lightSensorPin);
  if(analogValue < 500){            
    digitalWrite(ledPin, HIGH);
  }
  else{
    digitalWrite(ledPin, LOW);
  }
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void TaskStart (void *pvParameters) // This is a Task. 
{
  TickType_t xLastWakeTime;
//  const
  TickType_t xDelay
  ;xDelay
  = pdMS_TO_TICKS(400);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {

//    Serial.println("Start");
  if(onState==HIGH){
    for(int i=0;i<20;i++)
        xSemaphoreGive(Sem1);
    for(int i=0;i<10;i++)
        xSemaphoreGive(Sem2);
    for(int i=0;i<20;i++)
        xSemaphoreGive(Sem3);
//    for(int i=0;i<2;i++)
        xSemaphoreGive(Sem0);
  }
    if(onState==LOW){
      lcd.clear();
      lcd.setCursor(7,1);
      lcd.print("OFF");
      lcd.setCursor(4,0);
      if(keyEntered==0)
        lcd.print(" Locked ");
      else
         lcd.print("Unlocked");
    }
  
    
    if(onState==LOW){
    if ( mfrc522.PICC_IsNewCardPresent()) {
    if ( mfrc522.PICC_ReadCardSerial()) {
    // Dump debug info about the card; PICC_HaltA() is automatically called
    if (mfrc522.uid.uidByte[0] == 0x99 && 
     mfrc522.uid.uidByte[1] == 0x91 &&
     mfrc522.uid.uidByte[2] == 0xA2 &&
     mfrc522.uid.uidByte[3] == 0x6D) {
      if(keyEntered==0){
        keyEntered=1;
//        Serial.println("Unlocked");
      }
       else{
        keyEntered=0;
       }
    }
    }
  }
}

if(keyEntered==1){
  
    buttonState=digitalRead(buttonPin);
  if(prevState!=buttonState){
    prevState=buttonState;
    if(buttonState==HIGH){
      if(onState==LOW){
//        Serial.println("ON");
        lcd.setCursor(7,1);
        lcd.print("ON ");
        onState=HIGH;
        digitalWrite(enableBluetooth,HIGH);
        xDelay = pdMS_TO_TICKS(500);
      }
       else
       if(gear[0]=='P'){
        digitalWrite(enableBluetooth,LOW);
        xDelay = pdMS_TO_TICKS(400);
        onState=LOW;
       }
     }
  }
  }
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
void TaskMotor (void *pvParameters) // This is a Task. 
{
  TickType_t xLastWakeTime;
  const TickType_t xDelay = pdMS_TO_TICKS(200);
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
   digitalWrite(buzzerPin,LOW);
    xSemaphoreTake(Sem3,portMAX_DELAY);
//    Serial.println("Motor");
distance=100;
if(gear[0]=='D'){
  //Ultrasonic
   // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance= duration*0.034/2;
}
  if(distance>30){
   digitalWrite(buzzerPin,LOW);
    analogWrite(enA, PWMA); // Send PWM signal to L298N Enable pin
    analogWrite(enB, PWMB); // Send PWM signal to L298N Enable pin
  }
  else{
   digitalWrite(buzzerPin,HIGH);
    analogWrite(enA, 0); // Send PWM signal to L298N Enable pin
    analogWrite(enB, 0); // Send PWM signal to L298N Enable pin
  }
  vTaskDelayUntil(&xLastWakeTime, xDelay);
  }
}
