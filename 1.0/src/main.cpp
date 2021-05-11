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

String filename = "test.txt";

//FUNCTIONS

void blink_connect(int num, int led) {
  for (int i = 0; i < num; i++){
    digitalWrite(led, LOW);
    delay(200);
    digitalWrite(led, HIGH);
    delay(200);
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

  //LED MAPPING
  pinMode(red_led, OUTPUT);
  pinMode(green_led, OUTPUT);
  pinMode(blue_led, OUTPUT);

  //LED INITIALIZING
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, HIGH);
  digitalWrite(blue_led, HIGH);

  digitalWrite(red_led, LOW);
  delay(200);
  digitalWrite(blue_led, LOW);
  delay(200);
  digitalWrite(green_led, LOW);
  delay(200);
  digitalWrite(red_led, HIGH);
  digitalWrite(green_led, HIGH);
  digitalWrite(blue_led, HIGH);
  delay(1000);

  //SERIAL
  Serial.begin(9600);

  while (!Serial){

  }

  for (int i = 0; i < 2; i++){
    digitalWrite(red_led, LOW);
    digitalWrite(green_led, LOW);
    digitalWrite(blue_led, LOW);
    delay(500);
    digitalWrite(red_led, HIGH);
    digitalWrite(green_led, HIGH);
    digitalWrite(blue_led, HIGH);
    delay(500);
  }
  delay(1000);
  
  
  //CREATE GYRO
  imu.initialize();

  imu.setXAccelOffset(-20425);
  imu.setYAccelOffset(-32689);
  imu.setZAccelOffset(21616);

  imu.setXGyroOffset(12);
  imu.setYGyroOffset(-174);
  imu.setZGyroOffset(-41);

  /*
  while (!imu.testConnection()){
    Serial.println(imu.getAccelerationX());
    Serial.println("MPU6050 CONNECTION FAILED");
    digitalWrite(red_led, LOW);
    delay(500);
    digitalWrite(red_led, HIGH);
    delay(500);
  }
  */

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

  if (SD.exists("text.txt")){
    Serial.println("FILE EXISTS");
    blink_connect(4, green_led);
  } else {
    Serial.println("FILE DOES NOT EXIST");
  }

  datalog = SD.open("text.txt", FILE_WRITE);
  datalog.close();

  if (SD.exists("text.txt")){
    Serial.println("LOG SUCCESSFULLY OPENED");
  } else {
    Serial.println("LOG FAILED TO OPEN");
    blink_connect(2, red_led);
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

float acc_x, acc_y, acc_z;
float gyro_x, gyro_y, gyro_z;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float elapsedTime, currentTime, previousTime;

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

    acc_x = ax / 16384.0 * 9.805;
    acc_y = ay / 16384.0 * 9.805;
    acc_z = az / 16384.0 * 9.805;

    previousTime = currentTime;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 1000;

    gyro_x = gx / 131;
    gyro_y = gy / 131;
    gyro_z = gz / 131;

    accAngleX = (atan(acc_y / sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / PI);
    accAngleY = (atan(-1 * acc_x / sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / PI);

    gyroAngleX = gyroAngleX + gyro_x * elapsedTime;
    gyroAngleY = gyroAngleY + gyro_y * elapsedTime;
    yaw =  yaw + gyro_z * elapsedTime;
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

    dataString += flight_time + ", ";
    dataString += volt_in +  ", ";
    dataString += bmp.readPressure();
    dataString += ", ";
    dataString += bmp.readAltitude(local_pressure);
    dataString += ", ";
    dataString += bmp.readTemperature();
    dataString += ", ";
    dataString += acc_x;
    dataString += ", ";
    dataString += acc_y;
    dataString += ", ";
    dataString += acc_z;
    dataString += ", ";
    dataString += gyro_x;
    dataString += ", ";
    dataString += gyro_z;
    dataString += ", ";
    dataString += gyro_z;
    dataString += ", ";

    Serial.print(yaw);
    Serial.print(",");
    Serial.print(roll);
    Serial.print(",");
    Serial.print(pitch);
    Serial.print(",");
    Serial.println();

    /*
    if (datalog){
      datalog.println(dataString);
    }
    */
  }

  delay(loop_delay);
}


