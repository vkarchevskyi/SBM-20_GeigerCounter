/*
Підключення SD карти:
  ** MOSI - D11
  ** MISO - D12
  ** CLK/SCK - D13
  ** CS - D10 
*/

#include <Wire.h>

#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>

#define DEBUG 1
#define SENSOR_PIN 2
#define BUZZER_PIN 7
#define BATTERY_PIN A3
#define MEASUREMENT_SECS 40

volatile unsigned int newTicks = 0;

unsigned int ticksLog[MEASUREMENT_SECS];
unsigned int tickCountsLog[MEASUREMENT_SECS];
int logsIndex = -1;
unsigned int ticksCount = 0;
unsigned int ticksCountPerSec = 0;
unsigned long tickCountsLogSum = 0;

unsigned long lastTickLogsUpdateTime = 0;
unsigned long lastVoltageReadTime = 0;

float voltage = 0;

float sbm20Multiplier = 0.57f;

File sensorData;
const int CSpin = 10;

TinyGPS gps;
SoftwareSerial gpsSerial(7, 8);
bool newdata = false;
unsigned long start;
long lat, lon;
unsigned long time, date;

void setup() {
  gpsSerial.begin(9600);
  Serial.begin(9600);
  delay(1000); // додаємо таймер на випадок, якщо резистори неправильно підключені
  pinMode(SENSOR_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), sensorISR, FALLING);

  pinMode(CSpin, OUTPUT);
  if (SD.begin())
  {
    Serial.println("SD card is ready to use.");
  } else
  {
    Serial.println("SD card initialization failed");
    return;
  }

  if(!SD.exists("data.csv")) {
      sensorData = SD.open("data.csv", FILE_WRITE);
      sensorData.println("Lon,Lat,Rad");
      sensorData.close();
  } 
  start = millis();

#if DEBUG
  Serial.println("Setup complete");
#endif
}

void sensorISR() {
  newTicks++;
}

void loop() {
  unsigned int _newTicks = newTicks;
  newTicks = 0;
  updateTicks(_newTicks);
  updateTickLogs();
  readVoltage();
  boolean hasNewTicks = (_newTicks > 0);

  // затримка в 2 секунди між оновленням координат
  if (millis() - start > 2000)
  {
      newdata = readgps();
      if (newdata)
      {
          start = millis();
          gps.get_position(&lat, &lon);
          gps.get_datetime(&date, &time);
          
          Serial.print("Lat: "); Serial.print(lat);
          Serial.print(" Long: "); Serial.print(lon);
          Serial.print(" Date: "); Serial.print(date);
          Serial.print(" Time: "); Serial.println(time);

          //saveData();
      }
      saveData();
      start = millis();
  }
}

void readVoltage() {
  if(!isTimeOut(lastVoltageReadTime, 1000)) {
    return;
  }
  
  voltage = (analogRead(BATTERY_PIN) * 5.0) / 1024.0; 
   
  if (voltage < 0.1) {
    voltage = 0.0;
  }
}

void updateTicks(unsigned int newValue) {
  ticksCountPerSec += newValue;
  ticksCount += newValue;

#if DEBUG
  if(newValue > 0) {
    Serial.print(millis());
    Serial.print(" ");  
    Serial.print(newValue);
    Serial.print(" ");
    Serial.println(ticksCount);
  }
#endif
}

void updateTickLogs() {
  if(!isTimeOut(lastTickLogsUpdateTime, 1000)) {
    return;
  }
      
  if(logsIndex < (MEASUREMENT_SECS - 1)) {
    logsIndex++;
  } else {
    ticksCount -= ticksLog[0];
    tickCountsLogSum -= tickCountsLog[0];
    
    for(byte i = 1; i <= logsIndex; i++) {
      ticksLog[i - 1] = ticksLog[i];
      tickCountsLog[i - 1] = tickCountsLog[i];
    }
  }

  ticksLog[logsIndex] = ticksCountPerSec;
  ticksCountPerSec = 0;
  
  tickCountsLog[logsIndex] = ticksCount;
  
  tickCountsLogSum += ticksCount;
}

boolean isTimeOut(unsigned long& lastTimestamp, unsigned long duration) {
  unsigned long currentTimestamp = millis();
  
  if((currentTimestamp - lastTimestamp) < duration) {
    return false;
  }
  
  lastTimestamp = currentTimestamp;

  return true;
}

// Перевіряємо наявність даних
bool readgps()
{
    while (gpsSerial.available())
    {
        int b = gpsSerial.read();
        //у бібліотеці TinyGPS є баг: не обробляються дані, що мають символи "\r" та "\n"
        if('\r' != b)
        {
            if (gps.encode(b))
            return true;
        }
    }
    return false;
}

void saveData(){
    Serial.println("Write to sd...");
    if(SD.exists("data.csv")) { // перевірити, чи sd карта все ще наявна
        // Додаємо нову інформацію до файла
        sensorData = SD.open("data.csv", FILE_WRITE);
        if (sensorData) {
            
            String s_lon = String(lon / 1000000) + "." + String(lon);
            s_lon.remove(3, 2);
            String s_lat = String(lat / 1000000) + "." + String(lat);
            s_lat.remove(3, 2);
            sensorData.println(String(s_lon) + "," + String(s_lat) + "," + String(ticksCount * sbm20Multiplier));
            sensorData.close(); // закриваємо файл
        }
    } else {
      Serial.println("Error to write");
    }
}
