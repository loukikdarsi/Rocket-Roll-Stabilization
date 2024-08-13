#include <MPU9250.h>
#include <Math.h>
#include <Adafruit_BMP280.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>


#define DEG2RAD (M_PI / 180)
#define RAD2DEG (180 / M_PI)
#define Ki 1
#define Kp 1

#define Servo1 3
#define Servo2 5
#define Servo3 6
#define Servo4 9

MPU9250 mpu;
Adafruit_BMP280 bmp;
Servo myservo;

float AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ;
float Roll, Pitch, Yaw;
float EulerX, EulerY, EulerZ;
float LinAccX, LinAccY, LinAccZ;
float QuatX, QuatY, QuatZ, QuatW;

float imuTemp, bmpTemp;
float Pressure, Altitude;

float VelX, VelY, VelZ;
float prevGyroZ = 0;
float deltaGyroZ = 0;
float Attack;
float finAngle = 0;

const float alpha = 0.2;  // Filter coefficient (adjust as needed)
float filteredAccX, filteredAccY, filteredAccZ = 0.0;

uint32_t prevTime = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    delay(2000);

    myservo.attach(9);
    myservo.write(90);

    if (!mpu.setup(0x68)) {  // Input the address of the IMU
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }

    // Sensor Calibration
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    mpu.verbose(true);
    delay(5000);
    mpu.calibrateAccelGyro();

    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    mpu.calibrateMag();

    print_calibration();
    delay(2000);
    mpu.verbose(false);

    while ( !Serial ) delay(100);   // wait for native usb
    Serial.println(F("BMP280 test"));

    unsigned status;
    status = bmp.begin(0x76);

    if (!status) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or " "try a different address!"));
      Serial.print("SensorID was: 0x");
      Serial.println(bmp.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");

      while (1) delay(10);
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;
    
    mpu.setup(0x68, setting);

}

void loop() {
    if (mpu.update()) {
        static uint32_t prev_ms = millis();
        if (millis() > prev_ms + 15) {
            calculateDeltaGyro();
            stabilizeRoll();
            calculateAngleofAttack();
            read_data();
            calculateVelocity();
            //print_mpu_raw_values();
            //print_bmp_raw_values();
            prev_ms = millis();
        }
    }
}

void read_data() {

    //MPU Data
    AccX = mpu.getAccX();
    AccY = mpu.getAccY();
    AccZ = mpu.getAccZ();
    GyroX = mpu.getGyroX();
    GyroY = mpu.getGyroY();
    GyroZ = mpu.getGyroZ();
    MagX = mpu.getMagX();
    MagY = mpu.getMagY();
    MagZ = mpu.getMagZ();

    LinAccX = mpu.getLinearAccX();
    LinAccY = mpu.getLinearAccY();
    LinAccZ = mpu.getLinearAccZ();

    Roll = mpu.getRoll();
    Pitch = mpu.getPitch();
    Yaw = mpu.getYaw();

    EulerX = mpu.getEulerX();
    EulerY = mpu.getEulerY();
    EulerZ = mpu.getEulerZ();

    QuatX = mpu.getQuaternionX();
    QuatY = mpu.getQuaternionY();
    QuatZ = mpu.getQuaternionZ();
    QuatW = mpu.getQuaternionW();

    imuTemp = mpu.getTemperature();

    //BMP Data
    bmpTemp = bmp.readTemperature();

    Pressure = bmp.readPressure();
    Altitude = bmp.readAltitude(1013.25);
}

void print_mpu_raw_values() {
    
    Serial.println("9-axis Measurements: ");
    Serial.print("AccX: "); Serial.print(AccX); Serial.print("\tAccY: "); Serial.print(AccY); Serial.print("\tAccZ: "); Serial.println(AccZ); 
    Serial.print("GyroX: "); Serial.print(GyroX); Serial.print("\tGyroY: "); Serial.print(GyroY); Serial.print("\tGyroZ: "); Serial.println(GyroZ); 
    Serial.print("MagX: "); Serial.print(MagX); Serial.print("\tMagY: "); Serial.print(MagY); Serial.print("\tMagZ: "); Serial.println(MagZ); 
    Serial.println("");
    
    Serial.println("Aircraft Angles: ");
    Serial.print("Roll: "); Serial.print(Roll); Serial.print("\tPitch: "); Serial.print(Pitch); Serial.print("\tYaw: "); Serial.println(Yaw); 
    Serial.println("");
    
    Serial.println("Euler Angles: ");
    Serial.print("EulerX: "); Serial.print(EulerX); Serial.print("\tEulerY: "); Serial.print(EulerY); Serial.print("\tEulerZ: "); Serial.println(EulerZ); 
    Serial.println("");
    
    Serial.println("Linear Acceleration: ");
    Serial.print("LinearAccX: "); Serial.print(LinAccX); Serial.print("\tLinearAccY: "); Serial.print(LinAccY); Serial.print("\tLinearAccZ: "); Serial.println(LinAccZ);
    Serial.println("");
    
    Serial.println("Quaternions: ");
    Serial.print("QuatX: "); Serial.print(QuatX); Serial.print("\tQuatY: "); Serial.print(QuatY); Serial.print("\tQuatZ: "); Serial.print(QuatZ); Serial.print("\tQuatW: "); Serial.println(QuatW);
    Serial.println("");

    //delay(200);
}

void print_bmp_raw_values() {

    Serial.print(F("Temperature = "));
    Serial.print(bmpTemp);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(Pressure);
    Serial.println(" Pa");

    Serial.print(F("Approx altitude = "));
    Serial.print(Altitude); /* Adjusted to local forecast! */
    Serial.println(" m");

    Serial.println();

    //delay(200);
}

void print_calculated_values() {

    //Serial.print("Angle of Attack: ");
    //Serial.println(Attack);

}

void lowPassFilter(float newReading) {
  filteredAccX = alpha * newReading + (1 - alpha) * filteredAccX;
	filteredAccY = alpha * newReading + (1 - alpha) * filteredAccY;
	filteredAccZ = alpha * newReading + (1 - alpha) * filteredAccZ;
}

void stabilizeRoll() {

    finAngle = Ki * mpu.getYaw() + Kp * deltaGyroZ;
    finAngle = map(finAngle, -400, 400, 180, 0);

    if (finAngle <= 50 && finAngle >= 130) {
      finAngle = 90;
    }

    if (Attack > 35) {
      finAngle = 90;
    }
    myservo.write(finAngle);
    //Serial.println(finAngle);
}

void calculateVelocity() {
    uint32_t currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;  // Time in seconds

    lowPassFilter(LinAccX);
	lowPassFilter(LinAccY);
	lowPassFilter(LinAccZ);

    if ((abs(filteredAccZ)) > 0.2) {
      // Integrate linear acceleration to find velocity
      VelX += filteredAccX * dt; 
	  VelY += filteredAccY * dt;
	  VelZ += filteredAccZ * dt;	  
    }

    // Update previous time
    prevTime = currentTime;
}

void calculateAngleofAttack() {

  Attack =  acos( ( (cos(mpu.getRoll()* DEG2RAD))*(cos(mpu.getPitch()* M_PI / 180)) ) / (sqrt( ( (pow(sin(mpu.getPitch()* M_PI / 180), 2)) + ((pow(sin(mpu.getRoll()* M_PI / 180), 2))*(pow(cos(mpu.getPitch()* M_PI / 180), 2))) + ((pow(cos(mpu.getRoll()* M_PI / 180), 2))*(pow(cos(mpu.getPitch()* M_PI / 180), 2))) ) ) ) );
  Attack *= 180.0 / M_PI;
}

void calculateDeltaGyro() {

  float currentGyroZ = mpu.getGyroZ();
  deltaGyroZ = currentGyroZ - prevGyroZ;
  prevGyroZ = currentGyroZ;
}

void print_calibration() {
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(mpu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(mpu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(mpu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(mpu.getMagBiasX());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasY());
    Serial.print(", ");
    Serial.print(mpu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(mpu.getMagScaleX());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleY());
    Serial.print(", ");
    Serial.print(mpu.getMagScaleZ());
    Serial.println();
}