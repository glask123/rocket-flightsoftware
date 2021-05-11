#include <Arduino.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <I2Cdev.h>
#include <MPU6050.h>

//PIN MAPPING
#define voltage_pin 17
#define blue_led 14
#define green_led 16
#define red_led 15
#define sd_chip_select 0

//FUNCTIONS

void blink_connect(int num, int led) {
  for (int i = 0; i < num; i++){
    digitalWrite(led, LOW);
    delay(300);
    digitalWrite(led, HIGH);
    delay(300);
  }
  delay(1000);
}

//STATE MANAGEMENT
String status = "prelaunch";

//INITIALIZE BREAKOUTS
File datalog;
Adafruit_BMP280 bmp;
MPU6050 imu(0x68);

//INITIALIZE GYRO VALUES
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

  //LED MAPPING
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);

  //LED INITIALIZING
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, HIGH);
  digitalWrite(blue_led, HIGH);

  //CREATE GYRO
  imu.initialize();

  imu.setXAccelOffset(-20425);
  imu.setYAccelOffset(-32689);
  imu.setZAccelOffset(21616);

  imu.setXGyroOffset(12);
  imu.setYGyroOffset(-174);
  imu.setZGyroOffset(-41);

  while (!imu.testConnection()){
    Serial.println(imu.getDeviceID());
    Serial.println("MPU6050 CONNECTION FAILED");
    digitalWrite(red_led, LOW);
    delay(500);
    digitalWrite(red_led, HIGH);
    delay(500);
  }

  Serial.println("CONNECTED TO MPU6050");
  blink_connect(1, green_led);


  while (!bmp.begin()) {
    Serial.println("BMP CONNECTION FAILED");
    digitalWrite(red_led, LOW);
    delay(500);
    digitalWrite(red_led, HIGH);
    delay(500);
  }

  Serial.println("CONNECTED TO BMP");
  blink_connect(2, green_led);


  while (!SD.begin(sd_chip_select)) {
    Serial.println("SD CONNECTION FAILED");
    digitalWrite(red_led, LOW);
    delay(500);
    digitalWrite(red_led, HIGH);
  }

  Serial.println("CONNECTED TO SD");
  blink_connect(3, green_led);

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
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, HIGH);
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
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // DATALOG: TIMESTEP, VOLTAGE, PRESSURE, ALT, TEMP, AX, AY, AZ, GX, GY, GZ
  if (status == "prelaunch"){
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, LOW);
  }
  if (status == "launch"){
    flight_time += loop_delay;
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, HIGH);

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


