#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <I2Cdev.h>
#include <MPU6050.h>

const int voltage_pin = 17;
const int blueLED = 14;
const int greenLED = 16;
const int redLED = 15;

int sd_cs = 0;

String status = "prelaunch";

File datalog;
Adafruit_BMP280 bmp;
MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  Serial.begin(9600);

  while (!Serial){
    
  }

  pinMode(redLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, HIGH);
    digitalWrite(greenLED, HIGH);

  accelgyro.initialize();

  if (!accelgyro.testConnection()){
    digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, HIGH); 
    Serial.println("MPU6050 CONNECTION FAILED");
  }

   Serial.println("CONNECTED TO MPU6050");

  if (!bmp.begin()) {
    digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, HIGH);
    Serial.println("BMP CONNECTION FAILED");
  }

  Serial.println("CONNECTED TO BMP");

  accelgyro.setXAccelOffset(-20425);
  accelgyro.setYAccelOffset(-32689);
  accelgyro.setZAccelOffset(21616);

  accelgyro.setXGyroOffset(12);
  accelgyro.setYGyroOffset(-174);
  accelgyro.setZGyroOffset(-41);

  if (!SD.begin(sd_cs)) {
    digitalWrite(redLED, HIGH);
    digitalWrite(blueLED, LOW);
    digitalWrite(greenLED, HIGH);
    Serial.println("SD CONNECTION FAILED");
  }

  Serial.println("CONNECTED TO SD");

  if (SD.exists("r1_f1_datalog.txt")){
    Serial.println("r1_f1_datalog.txt EXISTS");
  } else {
    Serial.println("r1_f1_datalog.txt DOES NOT EXIST");
  }

  datalog = SD.open("r1_f1_datalog.txt", FILE_WRITE);
  datalog.close();

  if (SD.exists("r1_f1_datalog.txt")){
    Serial.println("FILE SUCCESSFULLY CREATED");
  } else {
    digitalWrite(redLED, LOW);
    digitalWrite(blueLED, HIGH);
    digitalWrite(greenLED, HIGH);
    Serial.println("FILE CREATION FAILED");
    return;
  }


  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16, 
                  Adafruit_BMP280::STANDBY_MS_500);
  
}

int loop_delay = 1000;
int flight_time;
double local_pressure;
double starting_altitude;

void loop() {
  String dataString = "";

  int volt_in = analogRead(voltage_pin);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // DATALOG: TIMESTEP, VOLTAGE, PRESSURE, ALT, TEMP, AX, AY, AZ, GX, GY, GZ
  if (status == "prelaunch"){
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, LOW);
  }
  if (status == "launch"){
    flight_time += loop_delay;
    digitalWrite(blueLED, HIGH);
    digitalWrite(redLED, HIGH);
    digitalWrite(greenLED, HIGH);

    dataString += flight_time + ", ";
    dataString += volt_in +  ", ";
    dataString += bmp.readPressure();
    dataString += ", ";
    dataString += bmp.readAltitude(local_pressure);
    dataString += ", ";
    dataString += bmp.readTemperature();
    dataString += ", ";
    dataString += ax/16384.0 * 9.805;
    dataString += ", ";
    dataString += ay/16384.0 * 9.805;
    dataString += ", ";
    dataString += az/16384.0 * 9.805;
    dataString += ", ";
    dataString += gx/131;
    dataString += ", ";
    dataString += gy/131;
    dataString += ", ";
    dataString += gz/131;
    dataString += ", ";


    if (datalog){
      datalog.println(dataString);
    }
  }

  delay(loop_delay);
}