// LCD
#include<Wire.h>
#include<LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// BME680
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#define MIN(a,b) (((a)<(b))?(a):(b))
#define SEALEVELPRESSURE_HPA (1013.25)
#define GAS_CEIL 90000
Adafruit_BME680 bme;
float CalculateIAQ(float temperature, float humidity, float gas_resistance);

// LEDs
#define LED1 4
#define LED2 5
#define LED3 6
#define LED4 7
int LEDS[] = {LED1, LED2, LED3, LED4};

// Button
#define BUTTON 2

// Boundaries
float bound_temp_high = 30;
float bound_temp_low = 0;
float bound_humid_high = 60;
float bound_humid_low = 40;
float bound_press_high = 1050;
float bound_press_low = 950;
float bound_iaq = 100;

// Flags
bool flag_temp = 0;
bool flag_humid = 0;
bool flag_press = 0;
bool flag_iaq = 0;

void setup() {
  // Serial
  Serial.begin(9600);
  while(!Serial);
  Serial.println(F("RSSV Projekt"));

  help();
  
  // LCD
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.home();
  lcd.print("RSSV Projekt");
  delay(1000);
  lcd.clear();
  lcd.home();

  // BME680
  if (!bme.begin(0x76)) {
    Serial.println(F("Could not find a valid BME680 sensor, check wiring!"));
    while(1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_16X);
  bme.setHumidityOversampling(BME680_OS_16X);
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  // LEDs
  for (int i = 0; i < 4; i++)
  {
    pinMode(LEDS[i], OUTPUT);
    digitalWrite(LEDS[i], LOW);
  }

  // Button
  pinMode(BUTTON, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON), reset, RISING);
}

void loop() {
  // Begin measurement.
  unsigned long endTime = bme.beginReading();
  if (endTime == 0) {
    Serial.println(F("Failed to begin reading :("));
    return;
  }

  // PARALLEL CODE HERE
  // Serial input
  if(Serial.available()){
    String str = Serial.readString();
    str.trim();
    if(str == "temperature_high"){
      Serial.println(F("Unesi novu gornju granicu temperature:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_temp_high = value.toFloat();
      Serial.print(F("Nova gornja granica temperature: "));
      Serial.print(bound_temp_high);
      Serial.print(F("\n"));
    }
    else if(str == "temperature_low"){
      Serial.println(F("Unesi novu donju granicu temperature:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_temp_low = value.toFloat();
      Serial.print(F("Nova donja granica temperature: "));
      Serial.print(bound_temp_low);
      Serial.print(F("\n"));
    }
    else if(str == "pressure_high"){
      Serial.println(F("Unesi novu gornju granicu tlaka:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_press_high = value.toFloat();
      Serial.print(F("Nova gornja granica tlaka: "));
      Serial.print(bound_press_high);
      Serial.print(F("\n"));
    }
    else if(str == "pressure_low"){
      Serial.println(F("Unesi novu donju granicu tlaka:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_press_low = value.toFloat();
      Serial.print(F("Nova donju granica tlaka: "));
      Serial.print(bound_press_low);
      Serial.print(F("\n"));
    }
    else if(str == "humidity_high"){
      Serial.println(F("Unesi novu gornju granicu vlage:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_humid_high = value.toFloat();
      Serial.print(F("Nova gornja granica vlage: "));
      Serial.print(bound_humid_high);
      Serial.print(F("\n"));
    }
    else if(str == "humidity_low"){
      Serial.println(F("Unesi novu donju granicu vlage:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_humid_low = value.toFloat();
      Serial.print(F("Nova donja granica vlage: "));
      Serial.print(bound_humid_low);
      Serial.print(F("\n"));
    }
    else if(str == "iaq"){
      Serial.println(F("Unesi novu granicu kvalitete zraka:"));
      while(Serial.available() <= 0){}
      String value = Serial.readString();
      bound_iaq = value.toFloat();
      Serial.print(F("Nova granica kvalitete zraka: "));
      Serial.print(bound_iaq);
      Serial.print(F("\n"));
    }
    else{
      help();
    }
  }

  // End of measurement.
  if (!bme.endReading()) {
    Serial.println(F("Failed to complete reading :("));
    return;
  }

  // Variables
  float temperature = bme.temperature;
  float pressure = bme.pressure / 100;
  float humidity = bme.humidity;
  float gas_resistance = bme.gas_resistance;
  float iaq = CalculateIAQ(temperature, humidity, gas_resistance);

  lcd.clear();
  lcd.print(temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print(iaq);
  lcd.print(" IAQ");
  lcd.home();
  delay(2000);

  lcd.clear();
  lcd.print(humidity);
  lcd.print(" %");
  lcd.setCursor(0, 1);
  lcd.print(pressure);
  lcd.print(" hPa");
  lcd.home();
  delay(2000);

  // Alarms
  if (temperature >= bound_temp_high || temperature <= bound_temp_low){
    if (!flag_temp){
      digitalWrite(LEDS[0],HIGH);
      flag_temp = 1;
    }
  }
  else flag_temp = 0;

  if (pressure >= bound_press_high || pressure <= bound_press_low){
    if (!flag_press){
      digitalWrite(LEDS[1],HIGH);
      flag_press = 1;
    }
  }
  else flag_press = 0;

  if (humidity >= bound_humid_high || humidity <= bound_humid_low){
    if (!flag_humid){
      digitalWrite(LEDS[2],HIGH);
      flag_humid = 1;
    }
  }
  else flag_humid = 0;

  if (iaq >= bound_iaq){
    if (!flag_iaq){
      digitalWrite(LEDS[3],HIGH);
      flag_iaq = 1;
    }
  }
  else flag_iaq = 0;
}

float CalculateIAQ(float temperature, float humidity, float gas_resistance)
{
  // Maximum absolute water density
  float rho_max = (6.112 * 100 * exp((17.62 * temperature)/(243.12 + temperature)))/(461.52 * (temperature + 273.15));
 
  // Absolute humidity
  float hum_abs = humidity * 10 * rho_max;
 
  float ph_slope = -0.055;
  // Bare VOC resistance
  float comp_gas = gas_resistance * exp(ph_slope * hum_abs);
 
  // Air quality
  float AQ = MIN(pow(comp_gas / GAS_CEIL, 2), 1);
 
  // Convert AQ to IAQ
  return AQ * 500;
}

void reset(){
  for (int i = 0; i < 4; i++)
    digitalWrite(LEDS[i], LOW);
}

void help(){
  // Ispis granica
  Serial.print(F("Gornja granica temperature: "));
  Serial.print(bound_temp_high);
  Serial.print("\n");
  Serial.print(F("Donja granica temperature: "));
  Serial.print(bound_temp_low);
  Serial.print("\n");
  Serial.print(F("Gornja granica vlage: "));
  Serial.print(bound_humid_high);
  Serial.print("\n");
  Serial.print(F("Donja granica vlage: "));
  Serial.print(bound_humid_low);
  Serial.print("\n");
  Serial.print(F("Gornja granica tlaka: "));
  Serial.print(bound_press_high);
  Serial.print("\n");
  Serial.print(F("Donja granica tlaka: "));
  Serial.print(bound_press_low);
  Serial.print("\n");
  Serial.print(F("Granica kvalitete zraka: "));
  Serial.print(bound_iaq);
  Serial.print("\n");

  Serial.println("Ako zelite mjenjati gornju/donju granicu temperature unesite temperature_high/temperature_low te nakon toga broj");
  Serial.println("Ako zelite mjenjati gornju/donju granicu vlage unesite humidity_high/humidity_low te nakon toga broj");
  Serial.println("Ako zelite mjenjati gornju/donju granicu tlaka unesite pressure_high/pressure_low te nakon toga broj");
  Serial.println("Ako zelite mjenjati granicu kvalitete zraka unesite iaq te nakon toga broj");
}