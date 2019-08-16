#include <iarduino_Pressure_BMP.h>
#include <MPU9255.h>// include MPU9255 library
#include<SD.h>
#include <SoftwareSerial.h>
#include <Wire.h>

#define ENGINE_START 5 //зажигание
#define CHUTE_DEPLOY 6 // парашют
#define BATTERY_VOLTAGE A2 // напряжение на батарейке
#define ENGINE_CHECK 3 // проверка двигателей
#define GND 2 
#define g 9.81
#define magnetometer_cal 0.06
#define RX 8
#define TX 9
#define THRESHOLD_VOLTAGE 4.2
MPU9255 mpu;
File dataFile;
SoftwareSerial mySerial(RX, TX);
unsigned long current_time, start_time, delay_time;
float temperature, altitude, pressure;
iarduino_Pressure_BMP sensor; 

bool check_rokkit() {
  if (analogRead(BATTERY_VOLTAGE) / 1023 * 5 < THRESHOLD_VOLTAGE) {
    return false;
  } else if (digitalRead(ENGINE_CHECK) == 0) {
    return false;
  } else {
    return true;
  }
}

void setup() {
  pinMode(CHUTE_DEPLOY, OUTPUT);
  pinMode(ENGINE_START, OUTPUT); //мощно
  pinMode(BATTERY_VOLTAGE, INPUT); //battery
  pinMode(ENGINE_CHECK, INPUT); //check ignitor
  pinMode(GND, OUTPUT);
  digitalWrite(GND, LOW);
  Serial.begin(2000000);
  Serial.print(analogRead(A2));
  Serial.print(digitalRead(3));
  mySerial.begin(1200);
  sensor.begin(); 
  Serial.print(F("Initializing SD card...\n"));
  if (!SD.begin(4)) {
    Serial.print(F("Card failed, or not present\n\n"));
  } else {
    Serial.print(F("Card initialized\n\n"));
  }
  dataFile = SD.open("datalog.dat", FILE_WRITE);
  Serial.print(F("Initialization MPU\n"));
  if (mpu.init()) {
    Serial.print(F("Initialization failed\n\n"));
  } else {
    Serial.print(F("MPU initialized\n\n"));
  }
  Serial.print(F("Calibarting MPU\n\n"));
  adjust_offset();
  Serial.print(F("Waiting start command\n"));
  bool done = true;
  while (done) {
    while (!mySerial.available()) {}
    String line = read_from_bluetooth();
    clean_buffer();
    if (line != F("49")) {
      Serial.print(line + "\n");
    } else {
      done = false;
    }
  }
  if (!check_rokkit()) {
    Serial.println("Все херово");
  }
  current_time = millis();
  start_time = millis();
}

void loop() {
  update_data();
  Serial.println("vavadh");
//  Serial.print("Accel: "); 
//  Serial.print(String(process_acceleration(mpu.ax,scale_2g)) + " "); Serial.print(String(process_acceleration(mpu.ay,scale_2g)) + " "); Serial.print(String(process_acceleration(mpu.az,scale_2g)));
//  Serial.print("  Gyro: "); 
//  Serial.print(String(process_angular_velocity(mpu.gx,scale_250dps)) + " "); Serial.print(String(process_angular_velocity(mpu.gy,scale_250dps)) + " "); Serial.print(String(process_angular_velocity(mpu.gz,scale_250dps)));
//  Serial.print("  Mag: "); 
//  Serial.print(String(process_magnetic_flux(mpu.mx,mpu.mx_sensitivity)) + " "); Serial.print(String(process_magnetic_flux(mpu.my,mpu.my_sensitivity)) + " "); Serial.print(String(process_magnetic_flux(mpu.mz,mpu.mz_sensitivity)));
//  Serial.print("  Temp: "); Serial.print(temperature);
//  Serial.print("  Press: "); Serial.print(pressure);
//  Serial.print("  Alt: "); Serial.println(altitude);
  write_to_sd();
  if (millis() - start_time > 30000) {
    dataFile.close();
    Serial.println("1");
    delay(100000);
  }
  //delay(400);
}

void update_data() {
  mpu.read_acc();
  mpu.read_gyro();
  mpu.read_mag();
  sensor.read(2);
  temperature = sensor.temperature;
  pressure = sensor.pressure;
  altitude = sensor.altitude;
  delay_time = millis() - current_time;
  current_time = millis();
}

//process raw acceleration data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : acceleration in m/s^2
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
  }

  if (sensor_scale == scale_4g) {
    output = input;
    output = output/8192;
    output = output*g;
  }

  if (sensor_scale == scale_8g) {
    output = input;
    output = output/4096;
    output = output*g;
  }

  if (sensor_scale == scale_16g) {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
float process_angular_velocity(int16_t input, scales sensor_scale) {
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */
  if (sensor_scale == scale_250dps) {
    return input/131.0;
  }

  if (sensor_scale == scale_500dps) {
    return input/65.5;
  }

  if (sensor_scale == scale_1000dps) {
    return input/32.8;
  }

  if (sensor_scale == scale_2000dps) {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
float process_magnetic_flux(int16_t input, float sensitivity){
  return (input*magnetometer_cal*sensitivity)/0.6;
}

struct datastore {
  float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z, temperature, pressure, altitude, delay_time;
};


void write_to_sd() {
  struct datastore myData;
  myData.g_x = process_angular_velocity(mpu.gx,scale_250dps);
  myData.g_y = process_angular_velocity(mpu.gy,scale_250dps);
  myData.g_z = process_angular_velocity(mpu.gz,scale_250dps);
  myData.m_x = process_magnetic_flux(mpu.mx,mpu.mx_sensitivity);
  myData.m_y = process_magnetic_flux(mpu.my,mpu.my_sensitivity);
  myData.m_z = process_magnetic_flux(mpu.mz,mpu.mz_sensitivity);
  myData.a_x = process_acceleration(mpu.ax,scale_2g);
  myData.a_y = process_acceleration(mpu.ay,scale_2g);
  myData.a_z = process_acceleration(mpu.az,scale_2g);
  myData.pressure = pressure;
  myData.altitude = altitude;
  myData.temperature = temperature;
  myData.delay_time = (float)delay_time;
  dataFile.write((const uint8_t *)&myData, sizeof(myData));
}

void clean_buffer() {
  bool done = false;
  while (!done) {
    for (int i = 0; i < 10; ++i) {
      if (String(mySerial.read()) != "-1") {
        break;
      }
      if (i == 9) {
        done = true;
      }
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
    if (!written) {
      return line;
    }
  }
}

void adjust_offset()
{
  //set bandwidths to 5Hz to reduce the noise
  mpu.set_acc_bandwidth(acc_5Hz);
  mpu.set_gyro_bandwidth(gyro_5Hz);

  int16_t gX_offset = 0;//gyroscope X axis offset
  int16_t gY_offset = 0;//gyroscope Y axis offset
  int16_t gZ_offset = 0;//gyroscope Z axis offset

  int16_t aX_offset = 0;//accelerometer X axis offset
  int16_t aY_offset = 0;//accelerometer Y axis offset
  int16_t aZ_offset = 0;//accelerometer Z axis offset

  //update flags

  //gyroscope
  bool update_gX = true;
  bool update_gY = true;
  bool update_gZ = true;

  //accelerometer
  bool update_aX = true;
  bool update_aY = true;
  bool update_aZ = true;

  //discard the first reading
  mpu.read_acc();
  mpu.read_gyro();
  delay(10);

  while(1)//offset adjusting loop
  {
    mpu.read_acc();
    mpu.read_gyro();

    //-------- adjust accelerometer X axis offset ----------

    if(mpu.ax>0 && update_aX==true)//if X axis readings are greater than 0
    {
      aX_offset --;//decrement offset
    }


    if(mpu.ax<0 && update_aX==true)
    {
      aX_offset ++;//increment offset
    }

    //-------- adjust accelerometer Y axis offset ----------

    if(mpu.ay>0 && update_aY==true)//if X axis readings are greater than 0
    {
      aY_offset --;//decrement offset
    }

    if(mpu.ay<0 && update_aY==true)
    {
      aY_offset ++;//increment offset
    }

    //-------- adjust accelerometer Z axis offset ----------

    if(mpu.az>0 && update_aZ==true)//if X axis readings are greater than 0
    {
      aZ_offset --;//decrement offset
    }

    if(mpu.az<0 && update_aZ==true)
    {
      aZ_offset ++;//increment offset
    }

        //-------- adjust gyroscope X axis offset ----------

    if(mpu.gx>0 && update_gX==true)//if X axis readings are greater than 0
    {
      gX_offset --;//decrement offset
    }

    if(mpu.gx<0 && update_gX==true)
    {
      gX_offset ++;//increment offset
    }

    //-------- adjust gyroscope Y axis offset ----------

    if(mpu.gy>0 && update_gY==true)//if X axis readings are greater than 0
    {
      gY_offset --;//decrement offset
    }

    if(mpu.gy<0 && update_gY==true)
    {
      gY_offset ++;//increment offset
    }

    //-------- adjust gyroscope Z axis offset ----------

    if(mpu.gz>0 && update_gZ==true)//if X axis readings are greater than 0
    {
      gZ_offset --;//decrement offset
    }

    if(mpu.gz<0 && update_gZ==true)
    {
      gZ_offset ++;//increment offset
    }

    //set new offset
    if(update_gX==true)
    {
      mpu.set_gyro_offset(X_axis,gX_offset);
    }

    if(update_gY==true)
    {
      mpu.set_gyro_offset(Y_axis,gY_offset);
    }

    if(update_gZ==true)
    {
      mpu.set_gyro_offset(Z_axis,gZ_offset);
    }

    if(update_aX==true)
    {
      mpu.set_acc_offset(X_axis,aX_offset);
    }

    if(update_aY==true)
    {
      mpu.set_acc_offset(Y_axis,aY_offset);
    }

    if(update_aZ==true)
    {
      mpu.set_acc_offset(Z_axis,aZ_offset);
    }

    //------ Check if each axis is adjusted -----
    const short maximum_error = 5;//set maximum deviation to 5 LSB
    if((mpu.ax-maximum_error) <= 0)
    {

    }

    if((abs(mpu.ax)-maximum_error) <= 0)
    {
      update_aX = false;
    }

    if((abs(mpu.ay)-maximum_error) <= 0)
    {
      update_aY = false;
    }

    if((abs(mpu.az)-maximum_error) <= 0)
    {
      update_aZ = false;
    }

    if((abs(mpu.gx)-maximum_error) <= 0)
    {
      update_gX = false;
    }

    if((abs(mpu.gy)-maximum_error) <= 0)
    {
      update_gY = false;
    }

    if((abs(mpu.gz)-maximum_error) <= 0)
    {
      update_gZ = false;
    }


    //------ Adjust procedure end condition ------
    if(update_gX==false && update_gY==false && update_gZ==false && update_aX==false && update_aY==false && update_aZ==false)
    {
      break;
    }

    delay(10);
  }
}
