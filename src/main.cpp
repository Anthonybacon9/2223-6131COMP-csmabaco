#include <Arduino.h>
#include <DHT.h>
#include <RotaryEncoder.h>
#include <LiquidCrystal.h>
#include <Wire.h>
#define DHT11PIN 4

#define DHTTYPE DHT11

DHT dht(DHT11PIN, DHTTYPE);
float h,tc,tf;
int toggle = 1;
int switchPin = 2;
int switchState = HIGH;

int pinA = 16;
int pinB = 15;
int pinAstateCurrent = LOW;
int pinAStateLast = pinAstateCurrent;
int prevSwitchState = HIGH;

const int redPin = 5;
const int greenPin = 18;
const int bluePin = 12;

//temperature
  float deftemp = 17.5;
  float mintemp = 5;
  float maxtemp = 30;

  //humidity
  float defhumidity = 50;
  float minhumidity = 25;
  float maxhumidity = 60;



void setup()
{
  delay(200);
  Serial.begin(9600);
  dht.begin();
  delay(1000);
  Serial.println("DHT11 Temperature and Humidity ");

  pinMode(switchPin, INPUT_PULLUP);
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

void loop()
{
    // Read temperature and humidity data from DHT11 sensor
  h = dht.readHumidity();
  tc = dht.readTemperature();
  tf = dht.readTemperature(true);

  // Check if temperature and humidity values are out of range


  boolean isTemperatureOutOfRange;
  boolean isHumidityOutOfRange;


  if (tc > maxtemp || tc < mintemp) {
    isTemperatureOutOfRange = true;
  } else {
    isTemperatureOutOfRange = false;
  }

  if (h > maxhumidity || h < minhumidity) {
    isHumidityOutOfRange = true;
  } else {
    isHumidityOutOfRange = false;
  }


  if (isTemperatureOutOfRange && isHumidityOutOfRange) {
    // Flash between red and blue
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);
    delay(500);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
    analogWrite(redPin, 255);
    delay(500);
  } else if (isTemperatureOutOfRange) {
    // Turn on red LED
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 0);
  } else if (isHumidityOutOfRange) {
    // Turn on blue LED
    analogWrite(redPin, 0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin, 255);
  } else if (!isTemperatureOutOfRange && !isHumidityOutOfRange){
    // Turn on green LED
    analogWrite(redPin, 0);
    analogWrite(greenPin, 255);
    analogWrite(bluePin, 0);
  }

  
  //BUTTON
  switchState = digitalRead(switchPin);

  if(switchState == LOW && prevSwitchState == HIGH) {
    Serial.println("Switch Pressed");
    prevSwitchState = LOW;
    delay(1000);
    if(toggle == 4) {
      toggle = 1;
      Serial.println("Minimum Temperature: ");
    } else if (toggle == 1) {
      toggle++;
      Serial.println("Maximum Temperature: ");
    } else if (toggle == 2) {
      toggle++;
      Serial.println("Minimum Humidity: ");
    } else if (toggle == 3) {
      toggle++;
      Serial.println("Maximum Humidity: ");
    } 
    
  } else {
    prevSwitchState = switchState;
  }
  pinAstateCurrent = digitalRead(pinA);

  if((pinAStateLast == LOW) && (pinAstateCurrent==HIGH)) {
    if(digitalRead(pinB) == HIGH) {
      if(toggle == 4) {
        mintemp = mintemp - 0.1;
        Serial.println("Minimum Temperature: ");
        Serial.println(mintemp);
      } else if (toggle == 1 && maxtemp>mintemp) {
        maxtemp = maxtemp - 0.1;
        Serial.println("Maximum Temperature: ");
        Serial.println(maxtemp);
      } else if (toggle == 2) {
        minhumidity = minhumidity - 0.1;
        Serial.println("Minimum Humidity: ");
        Serial.println(minhumidity);
      } else if (toggle == 3 && maxhumidity>minhumidity) {
        maxhumidity = maxhumidity - 0.1;
        Serial.println("Maximum Humidity: ");
        Serial.println(maxhumidity);
      } 
    } else {
      if(toggle == 4 && maxtemp>mintemp) {
        mintemp = mintemp + 0.1;
        Serial.println("Minimum Temperature: ");
        Serial.println(mintemp);
      } else if (toggle == 1) {
        maxtemp = maxtemp + 0.1;
        Serial.println("Maximum Temperature: ");
        Serial.println(maxtemp);
      } else if (toggle == 2 && maxhumidity>minhumidity) {
        minhumidity = minhumidity + 0.1;
        Serial.println("Minimum Humidity: ");
        Serial.println(minhumidity);
      } else if (toggle == 3) {
        maxhumidity = maxhumidity + 0.1;
        Serial.println("Maximum Humidity: ");
        Serial.println(maxhumidity);
      } 
    }
  }

  pinAStateLast = pinAstateCurrent;

  static unsigned long lastSampleTime = 0;
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = millis();
  const unsigned long sampleInterval = 1000; // Sample every 1 second
  const unsigned long debugInterval = 5000; // Log debug data every 5 seconds
  
  if (currentTime - lastSampleTime >= sampleInterval)
  {
    lastSampleTime = currentTime;
    
    h = dht.readHumidity();
    tc = dht.readTemperature();
    tf = dht.readTemperature(true);
    
    Serial.print('\n');
    Serial.print("Humidity = ");
    Serial.print(h);
    Serial.print("%,  ");
    Serial.print("Temperature = ");
    Serial.print(tc);
    Serial.print("°C, ");
    Serial.print(tf);
    Serial.println("°F");
  }
  
  if (currentTime - lastDebugTime >= debugInterval)
  {
    lastDebugTime = currentTime;
    
    Serial.print("Debug Data: ");
    Serial.print("Humidity = ");
    Serial.print(h);
    Serial.print("%,  ");
    Serial.print("Temperature = ");
    Serial.print(tc);
    Serial.print("°C, ");
    Serial.print(tf);
    Serial.println("°F");
  }
}