// in this sketch we only send data in pre-agreed format,
// the actual inference (prediction)/ collection is done on the commputer
// with python script

// general purpose
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;

const char READY = 0xAA;
int rec;
int rec_p;

#define BUFF_SIZE 100 // should be enugh...
char buff[BUFF_SIZE];
struct timeval tv_now;

void setup() {
    Serial.begin(115200);

    // wait for monitor wakeup
    while (!Serial) {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }

    // Try to initialize MPU
    while (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip");
        delay(10);
    }

    // set MPU defaults
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("");
    delay(100);

    Serial.println("ESP32 ready to go!");
}

void loop() {
    rec = Serial.read();
    if(rec == READY){
        // Get new sensor events with the readings
        mpu.getEvent(&accel, &gyro, &temp);
        
        // Get timestamp in microseconds
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        
        // format message and set value
        snprintf(buff, BUFF_SIZE, "@%s,%f,%f,%f,%f,%f,%f,#", String(time_us), accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
        Serial.print(buff);
    } else if(rec != rec_p) {
        Serial.print(rec);
    }
    rec_p = rec;
}
