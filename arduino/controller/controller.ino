#include <iarduino_Pressure_BMP.h>
#include <MPU9255.h>
#include<SD.h>
#include <SoftwareSerial.h>

#define ENGINE_START 5 //зажигание
#define HEADLIGHT 6 //парашют
#define BATTERY_VOLTAGE A2 //напряжение на батарейке
#define ENGINE_CHECK 3 //проверка двигателей
#define GND 2 
#define g 9.81
#define magnetometer_cal 0.06
#define RX 8
#define TX 9
#define THRESHOLD_VOLTAGE 4.2
File dataFile;
bool sd_is_working = false;
SoftwareSerial mySerial(RX, TX);
unsigned long current_time, start_time, delay_time;
float temperature, altitude, pressure, max_altitude;
iarduino_Pressure_BMP sensor; 
MPU9255 mpu;

void setup() {
  //initialization pins
  pinMode(HEADLIGHT, OUTPUT);
  pinMode(ENGINE_START, OUTPUT);
  pinMode(BATTERY_VOLTAGE, INPUT);
  pinMode(ENGINE_CHECK, INPUT);
  pinMode(GND, OUTPUT);

  //initialization serial ports
  Serial.begin(2000000);
  mySerial.begin(1200);

  //GND from pin
  digitalWrite(GND, LOW);

  //some output
  Serial.print(analogRead(BATTERY_VOLTAGE));
  Serial.print(digitalRead(ENGINE_CHECK));
  
  //starting BMP
  sensor.begin(); 

  //initialization SD card
  Serial.print(F("Initializing SD card...\n"));
  if (!SD.begin(4)) Serial.print(F("Card failed, or not present\n\n"));
  else {
    Serial.print(F("Card initialized\n\n"));
    sd_is_working = true;
  }
  dataFile = SD.open("datalog.dat", FILE_WRITE);

  //initialization and calibration MPU
  Serial.print(F("Initialization MPU\n"));
  if (mpu.init()) Serial.print(F("Initialization failed\n\n"));
  else Serial.print(F("MPU initialized\n\n"));
  Serial.print(F("Calibarting MPU\n\n"));
  adjust_offset(); //calibrating MPU

  //getting command from bluetooth
  Serial.print(F("Waiting start command\n"));
  bool launch = false;
  while (!launch) {
    bool done = false;
    while (!done) {
      while (!mySerial.available()) {}
      String line = read_from_bluetooth();
      clean_buffer();
      if (line != F("49")) Serial.print(line + "\n");
      else done = true;
    }
  
    //checking rokkit
    if (analogRead(BATTERY_VOLTAGE) / 1023.0 * 5.0 < THRESHOLD_VOLTAGE) {
      mySerial.write("1"); //low voltage
      Serial.println(F("Low voltage\nTry again"));
    } else if (!digitalRead(ENGINE_CHECK)) {
      mySerial.write("2"); //engine broken
      Serial.println(F("Engine is broken\nTry again"));
    } else if (!sd_is_working) {
      mySerial.write("3"); //sd doesn't work
      launch = true;
      Serial.println(F("SD doesn't work"));
    } else {
      mySerial.write("0");
      launch = true;
      Serial.println(F("Rocket launched"));
    }
  }
  
  //getting time
  current_time = millis();
  start_time = millis();
  //digitalWrite(ENGINE_START, HIGH);
}

void loop() {
  update_data();
  Serial.print("Accel: "); 
  Serial.print(String(process_acceleration(mpu.ax,scale_2g)) + " "); Serial.print(String(process_acceleration(mpu.ay,scale_2g)) + " "); Serial.print(String(process_acceleration(mpu.az,scale_2g)));
  Serial.print("  Gyro: "); 
  Serial.print(String(process_angular_velocity(mpu.gx,scale_250dps)) + " "); Serial.print(String(process_angular_velocity(mpu.gy,scale_250dps)) + " "); Serial.print(String(process_angular_velocity(mpu.gz,scale_250dps)));
  Serial.print("  Mag: "); 
  Serial.print(String(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity)) + " "); Serial.print(String(process_magnetic_flux(mpu.my,mpu.my_sensitivity)) + " "); Serial.print(String(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity)));
  Serial.print("  Temp: "); Serial.print(temperature);
  Serial.print("  Press: "); Serial.print(pressure);
  Serial.print("  Alt: "); Serial.println(altitude);
  write_to_sd();
//  if (millis() - start_time > 30000) {
//    dataFile.close();
//    Serial.println("1");
//    digitalWrite(HEADLIGHT, HIGH);
//    delay(100000);
//  }
//  if (millis() - start_time > 10000) {
//    digitalWrite(ENGINE_START, LOW);
//  }
  delay(400);
}

void update_data() {
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
  sensor.read(2);
  temperature = sensor.temperature;
  pressure = sensor.pressure;
  altitude = sensor.altitude;
  if (altitude > max_altitude) {
    max_altitude = altitude;
  } else if (max_altitude - altitude >= 1.5) {
    digitalWrite(HEADLIGHT, HIGH);
  }
  delay_time = millis() - current_time;
  current_time = millis();
}


float process_acceleration(int input, scales sensor_scale ) {
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  float output = 1;
  if (sensor_scale == scale_2g) {
    output = input;
    output = output/16384;
    output = output*g;
  } else if (sensor_scale == scale_4g) {
    output = input;
    output = output/8192;
    output = output*g;
  } else if (sensor_scale == scale_8g) {
    output = input;
    output = output/4096;
    output = output*g;
  } else if (sensor_scale == scale_16g) {
    output = input;
    output = output/2048;
    output = output*g;
  }
  return output;
}


float process_angular_velocity(int16_t input, scales sensor_scale) {
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */
  if (sensor_scale == scale_250dps) return input / 131.0;
  else if (sensor_scale == scale_500dps) return input / 65.5;
  else if (sensor_scale == scale_1000dps) return input / 32.8;
  else if (sensor_scale == scale_2000dps) return input / 16.4;
  return 0;
}


float process_magnetic_flux(int16_t input, float sensitivity){
  return (input * magnetometer_cal * sensitivity) / 0.6;
}

struct datastore {
  float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z, temperature, pressure, altitude, delay_time;
};


void write_to_sd() {
  struct datastore myData;
  myData.g_x = process_angular_velocity(mpu.gx, scale_250dps);
  myData.g_y = process_angular_velocity(mpu.gy, scale_250dps);
  myData.g_z = process_angular_velocity(mpu.gz, scale_250dps);
  myData.m_x = process_magnetic_flux(mpu.mx, mpu.mx_sensitivity);
  myData.m_y = process_magnetic_flux(mpu.my, mpu.my_sensitivity);
  myData.m_z = process_magnetic_flux(mpu.mz, mpu.mz_sensitivity);
  myData.a_x = process_acceleration(mpu.ax, scale_2g);
  myData.a_y = process_acceleration(mpu.ay, scale_2g);
  myData.a_z = process_acceleration(mpu.az, scale_2g);
  myData.pressure = pressure;
  myData.altitude = altitude;
  myData.temperature = temperature;
  myData.delay_time = (float)delay_time;
  dataFile.write((const uint8_t *)&myData, sizeof(myData));
}

void clean_buffer() {
  bool done = false;
  int amount_of_symbols_to_clean = 10;
  while (!done) {
    for (int i = 0; i < amount_of_symbols_to_clean; ++i) {
      if (String(mySerial.read()) != "-1") break;
      if (i == amount_of_symbols_to_clean - 1) done = true;
    }
  }
}

String read_from_bluetooth() {
  String line = "";
  while (true) {
    String chars[10];
    bool written = false;
    for (int i = 0; i < 10; ++i) {
      String current_char = String(mySerial.read());
      if (current_char != "-1") {
        line += current_char;
        written = true;
      }
    }
    if (!written) return line;
  }
}

void adjust_offset()
{
  //set bandwidths to 5Hz to reduce the noise
  mpu.set_acc_bandwidth(acc_5Hz);
  mpu.set_gyro_bandwidth(gyro_5Hz);
  
  int16_t gX_offset = 0;
  int16_t gY_offset = 0;
  int16_t gZ_offset = 0;
  int16_t aX_offset = 0;
  int16_t aY_offset = 0;
  int16_t aZ_offset = 0;
  bool update_gX = true;
  bool update_gY = true;
  bool update_gZ = true;
  bool update_aX = true;
  bool update_aY = true;
  bool update_aZ = true;

  //discard the first reading
  mpu.read_acc();
  mpu.read_gyro();
  delay(10);

  while(update_gX or update_gY or update_gZ or update_aX or update_aY or update_aZ) //offset adjusting loop
  {
    mpu.read_acc();
    mpu.read_gyro();

    //-------- adjust ----------
    if(mpu.ax > 0 and update_aX) --aX_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.ax < 0 and update_aX) ++aX_offset; //increment offset
    if(mpu.ay > 0 and update_aY) --aY_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.ay < 0 and update_aY) ++aY_offset; //increment offset
    if(mpu.az > 0 and update_aZ) --aZ_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.az < 0 and update_aZ) ++aZ_offset; //increment offset
    if(mpu.gx > 0 and update_gX) --gX_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.gx < 0 and update_gX) ++gX_offset; //increment offset
    if(mpu.gy > 0 and update_gY) --gY_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.gy < 0 and update_gY) ++gY_offset; //increment offset
    if(mpu.gz > 0 and update_gZ) --gZ_offset; //if X axis readings are greater than 0 decrement offset
    if(mpu.gz < 0 and update_gZ) ++gZ_offset; //increment offset

    //set new offset
    if(update_gX) mpu.set_gyro_offset(X_axis, gX_offset);
    if(update_gY) mpu.set_gyro_offset(Y_axis, gY_offset);
    if(update_gZ) mpu.set_gyro_offset(Z_axis, gZ_offset);
    if(update_aX) mpu.set_acc_offset(X_axis, aX_offset);
    if(update_aY) mpu.set_acc_offset(Y_axis, aY_offset);
    if(update_aZ) mpu.set_acc_offset(Z_axis, aZ_offset);

    //------ Check if each axis is adjusted -----
    const short maximum_error = 5; //set maximum deviation to 5 LSB
    if((abs(mpu.ax) - maximum_error) <= 0) update_aX = false;
    if((abs(mpu.ay) - maximum_error) <= 0) update_aY = false;
    if((abs(mpu.az) - maximum_error) <= 0) update_aZ = false;
    if((abs(mpu.gx) - maximum_error) <= 0) update_gX = false;
    if((abs(mpu.gy) - maximum_error) <= 0) update_gY = false;
    if((abs(mpu.gz) - maximum_error) <= 0) update_gZ = false;

    delay(10);
  }
}
