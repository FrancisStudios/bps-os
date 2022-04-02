#include<Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SFE_BMP180.h>
#include <SPI.h>
#include <SD.h>
#include <DS3231.h>
#include <Adafruit_MCP4725.h>

// BASE INITIALISATION
Adafruit_MCP4725 MCP4725;
DS3231 clock(SDA, SCL);
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 16, 2);
SFE_BMP180 pressure;
#define ALTITUDE 140.0

// GLOBALS
const int oxyswitch = 3;
char status;
double T, P, p0, a;

// INIT VARIABLES
String Temp, Press, pnul, year, month, day, hour, minutes, seconds, ds_date, ds_time;
int measureCounter = 0;
int arduinoPreviousSampleTime = 0;
int arduinoSelfTime = 0;
float alt, measure;
bool oxygen;
File record, baseline, header;


void setup() {
  Serial.begin(9600); lcd.init(); lcd.backlight();
  MCP4725.begin(0x60);
  Wire.begin(); clock.begin();
  pinMode(oxyswitch, INPUT);

  if (pressure.begin()) {
    showStartup();
    status = pressure.getPressure(P, T); delay(2000);
    timeSet(false); // if set true you can set time
    if (SD.begin(10)) {
      createDataTableHeader();
    } else {
      sdInitError();
    }
  } else {
    sensorInitError();
  }
}

void loop() {
  uint32_t MCP4725_value;
  String  time_string = clock.getTimeStr();
  if (time_string.substring(6, 8).toInt() % 5 == 0) {
    // NICE! FROM THIS POINT MEASUREMENTS CAN COME!
    getSample();
  }
  delay(1000); displayMeasurement();
}


// ---------------------------------------- [ CUSTOM FUNCTIONS ] ------------------------------------------- //
void getSample() {
  status = pressure.startTemperature();

  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);

    if (status != 0) {
      Temp = String(T, 2);
      status = pressure.startPressure(3);

      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);

        if (status != 0) {
          Press = String(P, 2); a = pressure.altitude(P, p0);
          createBaselineFile();
          getTimeAndDate();
          displayTimeAndDate();

          int oxyvalue = digitalRead(oxyswitch);

          if (oxyvalue == HIGH) { oxygen = true; } else { oxygen = false; }
          fileHandler(Temp, Press, a, oxygen);
          //Serial.println("From sampling:  Temp: " + Temp + " O2(y/n-1/0): " + oxygen)
        }
      }
    }
  }   
}

void fileHandler(String temperature, String pressure, double altitudum, bool oxygen){
    if(SD.begin(10)){
      record = SD.open("record.csv", FILE_WRITE);
      Serial.println(""+ds_date+";"+ds_time+";"+temperature+";"+pressure+";"+altitudum+";"+(oxygen ? "1" : "0")+"");
      generateControlVoltage(altitudum);
      record.println(""+ds_date+";"+ds_time+";"+temperature+";"+pressure+";"+altitudum+";"+(oxygen ? "1" : "0")+"" );
      record.close(); 
    }else{
      fileWritingError();
      delay(2000);
    }
}

void createDataTableHeader() {
  header = SD.open("record.csv", FILE_WRITE);
  header.println("Date(DD:MM:YY);Time(HH:MM:SS);Temperature(C);Pressure(mB);Altitude(m);Oxygen");
  header.close();
}

void sdInitError() {
  lcd.setCursor(0, 0); lcd.print("BPS ERROR");
  lcd.setCursor(0, 1); lcd.print("SD init fail");
}

void timeSet(bool wannaSetTime) {
  if (wannaSetTime) {
    // TIME SET 0
    //kb 15 sec az upload parancs után az óra indítása
    clock.setTime(16, 23, 45);
    clock.setDate(15, 9, 2020);
  }
}

void showStartup() {
  lcd.setCursor(0, 0); lcd.print("BPS v3.3");
  lcd.setCursor(0, 1); lcd.print("Init success!");
}

void sensorInitError() {
  lcd.setCursor(0, 0); lcd.print("BPS ERROR");
  lcd.setCursor(0, 1); lcd.print("Sensor init fail");
}

// PRINT MEASURED VALUES ON DISPLAY
void displayMeasurement() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:" + Temp + " " + "P:" + Press);
  lcd.setCursor(0, 1);
  if (String(a) == "INF" || String(a) == "-INF") { lcd.print("Alt:Calculating"); } else { lcd.print("Alt:" + String(a)); }
}


// CREATE BASELINE FILE AFTER THIRD MEASUREMENT
void createBaselineFile() {
  if (measureCounter == 0 || measureCounter == 1) { measureCounter++; } 
  else if (measureCounter == 2) {
    p0 = P;
    measure = p0;
    baseline = SD.open("baseline.txt", FILE_WRITE);
    baseline.println("time=" + ds_date + " " + ds_time);
    baseline.println("baseline=" + String(p0));
    baseline.close();
    measureCounter ++;
  }
}

void getTimeAndDate(){
  ds_date = clock.getDateStr();
  ds_time = clock.getTimeStr();
}

void displayTimeAndDate(){
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ds_date);
  lcd.setCursor(0, 1);
  lcd.print(ds_time);  
}

void fileWritingError(){
  lcd.clear(); lcd.setCursor(0,0); lcd.print("BPSandal ERROR");
  lcd.setCursor(0,1); lcd.print("SD init fail");
}

void generateControlVoltage(double altitudum){
  MCP4725.setVoltage((altitudum/1.5)+100 , false);
}

  
  
