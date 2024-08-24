/***********************************************************/
/*                Libraries                                */
/***********************************************************/
// general purpose
#include <time.h>
#include <sys/time.h>
#include "esp_sntp.h"

// BLE
#include <BluetoothSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// classifier
#include "forrest.hpp"

// buzzer
#include "pitches.hpp"

/*************************************************************************************/
/*                                  BLE Parameters                                   */
/*************************************************************************************/
#define MY_DEVICE_ADDRESS  "24:62:ab:f2:af:46" // change the address to the hands address if needed
//the names of the hands services used to send commands to the hand 
#define HAND_DIRECT_EXECUTE_SERVICE_UUID     "e0198000-7544-42c1-0000-b24344b6aa70"//sends the command according to the given message written in the code
#define EXECUTE_ON_WRITE_CHARACTERISTIC_UUID "e0198000-7544-42c1-0001-b24344b6aa70"
#define HAND_TRIGGER_SERVICE_UUID            "e0198002-7544-42c1-0000-b24344b6aa70"//tells the hand to move according to the presets saved on the hand that were 
//given using the application -- we dont use it here could be used for different implemintations
#define TRIGGER_ON_WRITE_CHARACTERISTIC_UUID "e0198002-7544-42c1-0001-b24344b6aa70"

static BLEUUID serviceExeUUID(HAND_DIRECT_EXECUTE_SERVICE_UUID);// The remote service we wish to connect to.
static BLEUUID charExeUUID(EXECUTE_ON_WRITE_CHARACTERISTIC_UUID);// The characteristic of the remote service we are interested in - execute.
static BLEUUID serviceTrigUUID(HAND_TRIGGER_SERVICE_UUID);// The remote service we wish to connect to.
static BLEUUID charTrigUUID(TRIGGER_ON_WRITE_CHARACTERISTIC_UUID);// The characteristic of the remote service we are interested in - trigger

// Connection parameters:
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = true;
static BLERemoteCharacteristic* pRemoteCharExecute;
static BLERemoteCharacteristic* pRemoteCharTrigger;
static BLEAdvertisedDevice* myDevice;

//uint8_t preset_id;// which preset will be triggered (12 options)

/*************************************************************************************/
/*                 Go to Sleep and Keep Scanning Setting:                            */
/*************************************************************************************/
#define DT_SCAN    10000 //if not connected to BLE device scan every 10 seconds.
#define DT_SLEEP    (10 * 60 * 1000) //if not connected to BLE device go to sleep after 10 minute.
unsigned long t_scan; // last BLE scan timestamp (ms)
unsigned long t_disconnected; //the elapsed time from a disconecting event


/*************************************************************************************/
/*                 Go to Sleep and Keep Scanning Setting:                            */
/*************************************************************************************/
int disconnect_tune[] = { NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 };
#define BUZZZER_PIN 4
#define NOTE_DURATION 250

void play_connected_tune(){
    int tune[] = { NOTE_C4, NOTE_E4, NOTE_A4 };
    for(int i = 0; i < 3; i++){
        tone(BUZZZER_PIN, tune[i], NOTE_DURATION);
        delay(NOTE_DURATION);
        noTone(BUZZZER_PIN);
    }
}

void play_disconnected_tune(){
    int tune[] = { NOTE_B3, NOTE_G3 };
    for(int i = 0; i < 3; i++){
        tone(BUZZZER_PIN, tune[i], NOTE_DURATION);
        delay(NOTE_DURATION);
        noTone(BUZZZER_PIN);
    }
}

void play_ready_tune(){
    int tune[] = { NOTE_C4, NOTE_C4, 0, NOTE_C4, NOTE_C4 };
    for(int i = 0; i < 5; i++){
        tone(BUZZZER_PIN, tune[i], NOTE_DURATION);
        delay(NOTE_DURATION);
        noTone(BUZZZER_PIN);
    }
}

void play_mode_indication(int mode_num){
    for(int i = 0; i < mode_num + 1; i++){
        tone(BUZZZER_PIN, NOTE_D4, NOTE_DURATION);
        delay(NOTE_DURATION);
        noTone(BUZZZER_PIN);
    }
}

/*************************************************************************************/
/*                  BLE Class Definition and Set Callbacks:                          */
/*************************************************************************************/
//for connecting to the phone for calibration
BluetoothSerial SerialBT;

//for connecting to the hand
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    play_connected_tune();
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
    doScan = true;
    t_disconnected = millis();
    play_disconnected_tune();
  }
};
// Scan for BLE servers and find the first one that advertises the service we are looking for.
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {  // Called for each advertising BLE server.
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Advertised Device found: ");
        Serial.println(advertisedDevice.toString().c_str());

        // We have found a device, let us now see if it contains the service we are looking for.
        if (((String)advertisedDevice.getAddress().toString().c_str()).equals(MY_DEVICE_ADDRESS)) {
            BLEDevice::getScan()->stop();
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            doConnect = true;
            doScan = false;
        } // Found our server
    } // onResult
}; // MyAdvertisedDeviceCallbacks


/*************************************************************************************/
/*                              Our System Setting:                                 */
/*************************************************************************************/

Adafruit_MPU6050 mpu;
sensors_event_t accel, gyro, temp;

const char READY = 0xAA;

#define CAPTURE_WINDOW_US 1000000
#define PERIOD_US 12500  // == 80 [Hz] - sample rate
#define THRESHOLD 0.7 // # of votes
#define AXES 6 //atributes

float buff[NUM_FEATURES] = {0};
char msg[300];
int votes[NUM_CLASSES] = {0};
struct timeval tv_now;
int64_t time_us_p;
int64_t previous_catch = 0;
int thresh_votes = THRESHOLD * NUM_TREES;

#define CLOSE_ALL_HIGH_TORQUE {5, 0b11111000, 25, 0b01111000, 0b11111000} 
#define OPEN_ALL              {5, 0b11111000, 25, 0b01111000, 0b00000000}
#define CLOSE_ALL_LOW_TORQUE  {5, 0b00000000, 25, 0b01111000, 0b11111000}
#define TRIPOD                {5, 0b00000000, 25, 0b01111000, 0b11100000}
#define TRIPOD_HIGH           {5, 0b11111000, 25, 0b01111000, 0b11100000}
#define PINCH                 {5, 0b00000000, 25, 0b01111000, 0b10100000}
#define PINCH_HIGH            {5, 0b11111000, 25, 0b01111000, 0b10100000}
#define POINTER               {5, 0b00000000, 25, 0b01111000, 0b10111000}
#define COOL                  {5, 0b00000000, 25, 0b01111000, 0b10110000}
#define THE_FINGER            {5, 0b00000000, 25, 0b01111000, 0b11011000}
#define TURN_RIGHT            {5, 0b00000000, 25, 0b10000000, 0b10110000}
#define TURN_LEFT             {5, 0b00000000, 25, 0b10000000, 0b00110000}

#define NUM_ACTIONS 3

bool active = false; // a flag that represents if any action is currently active
int current_action = 0;
unsigned char idle[] = OPEN_ALL; // open all
unsigned char actions[][5] = {
    CLOSE_ALL_HIGH_TORQUE,
    POINTER,
    COOL
};


//bluetooth functions
bool connectToServer() {
  
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");

    // Execute Charachteristics:
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteExeService = pClient->getService(serviceExeUUID);
    if (pRemoteExeService == nullptr) {
        Serial.print("Failed to find our Execute service UUID: ");
        Serial.println(serviceExeUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our Execute service");
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharExecute = pRemoteExeService->getCharacteristic(charExeUUID);
    if (pRemoteCharExecute == nullptr) {
        Serial.print("Failed to find our Execute characteristic UUID: ");
        Serial.println(charExeUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our Execute characteristic");
 
    // Read the value of the characteristic.
    if(pRemoteCharExecute->canRead()) {
        String value = pRemoteCharExecute->readValue().c_str();
        Serial.print("Execute: The characteristic value was: ");
        Serial.println(value.c_str());
    }
    
    // Trigger Charachteristics:
    // Obtain a reference to the service we are after in the remote BLE server.
    
    BLERemoteService* pRemoteTrigService = pClient->getService(serviceTrigUUID);
    if (pRemoteTrigService == nullptr) {
        Serial.print("Failed to find our Trigger service UUID: ");
        Serial.println(serviceTrigUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our Trigger service");
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharTrigger = pRemoteTrigService->getCharacteristic(charTrigUUID);
    if (pRemoteCharTrigger == nullptr) {
        Serial.print("Failed to find our Trigger characteristic UUID: ");
        Serial.println(charTrigUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }
    Serial.println(" - Found our Trigger characteristic");
 
    // Read the value of the characteristic.
    if(pRemoteCharTrigger->canRead()) {
      String value = pRemoteCharTrigger->readValue().c_str();
      Serial.print("Trigger: The characteristic value was: ");
      Serial.println(value.c_str());
    }
    connected = true;
    return true;
    
}

void InitBLE() {
    BLEDevice::init("ESP32");
    // Retrieve a Scanner and set the callback we want to use to be informed when we
    // have detected a new device.  Specify that we want active scanning and start the
    // scan to run for 5 seconds.
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(1, false);
}

void add_sample_to_buffer(){
    for(int i = AXES; i < NUM_FEATURES; i++){
        buff[i-AXES] = buff[i];
    }
    buff[NUM_FEATURES-6] = accel.acceleration.x;
    buff[NUM_FEATURES-5] = accel.acceleration.y;
    buff[NUM_FEATURES-4] = accel.acceleration.z;
    buff[NUM_FEATURES-3] = gyro.gyro.x;
    buff[NUM_FEATURES-2] = gyro.gyro.y;
    buff[NUM_FEATURES-1] = gyro.gyro.z;
}

void setup() {
    Serial.begin(115200);
  
    // wait for monitor wakeup
    while (!Serial) {
        delay(10); // will pause Zero, Leonardo, etc until serial console opens
    }
    //create a bluetooth service for the phone to connect to while calibrating:
    SerialBT.begin("ESP32_BT"); // Bluetooth device name
 
    // Create the BLE Device
    InitBLE();
    t_scan = millis();

    // enable deep sleep mode for the esp32:
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1); //1 = High, 0 = Low , same GPIO as the button pin
    t_disconnected = millis();

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

    time_us_p = 0;

    Serial.println("ESP32 ready to go!");
    play_ready_tune();
}

void loop() {
    // put your main code here, to run repeatedly:
    if (doConnect == true) { //TRUE when we scanned and found the desired BLE server
        if (connectToServer()) Serial.println("We are now connected to the BLE Server."); // connect to the server. TRUE if connection was established
        else Serial.println("We have failed to connect to the server; there is nothin more we will do.");
        doConnect = false; //no need to reconnect to the server
    }
    if (connected) { //TRUE if we are already connected to the server
        // Get timestamp in microseconds
        gettimeofday(&tv_now, NULL);
        int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
        if(abs(time_us - time_us_p) > PERIOD_US){
            mpu.getEvent(&accel, &gyro, &temp);
            time_us_p = time_us;
            add_sample_to_buffer();
            votes[0] = votes[1] = votes[2] = 0;
            rf_predict(buff, votes);
            // sprintf(msg,"%lld: rest:%d, up:%d, forward:%d | accX:%f accY:%f accZ:%f gyroX:%f gyroY:%f gyroZ:%f\n", time_us, votes[0], votes[1] , votes[2], accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
            // Serial.print(msg);
            
            // TODO: consider filtering on predictions in a row
            if(abs(time_us - previous_catch) > CAPTURE_WINDOW_US ){
                if(votes[1] >= thresh_votes){
                    previous_catch = time_us;
                    if(!active){ // idel -> active
                        active = true;
                        pRemoteCharExecute->writeValue(actions[current_action], actions[current_action][0]);
                    } else { // active -> idle
                        active = false;
                        pRemoteCharExecute->writeValue(idle, idle[0]);
                        Serial.print(msg);
                    }
                    sprintf(msg,"%lld: UP, votes:%d, active: %d, mode: %d\n", time_us, votes[1], active, current_action);
                    Serial.print(msg);
                } else if(votes[2] >= thresh_votes){
                    previous_catch = time_us;
                    Serial.print(msg);
                    if(active){ // only change active mode if in idle
                        sprintf(msg,"not changing mode while active!\n", time_us, votes[2]);
                        Serial.print(msg);
                    } else {
                        current_action = (current_action + 1) % NUM_ACTIONS;
                        play_mode_indication(current_action);
                    }
                    sprintf(msg,"%lld: FORWARD, votes:%d, active: %d, mode: %d\n", time_us, votes[2], active, current_action);
                    Serial.print(msg);
                } 
            } // CAPTURE_WINDOW_US
        } // PERIOD_US
    }
  
    else { //not connected
        //scanning for server:
        if((millis() - t_scan > DT_SCAN)){ //not connected
            //BLEDevice::getScan()->start(0);  // start to scan after disconnect. 0 - scan for infinity, 1-1 sec, ..
            Serial.println("Scanning...");
            BLEDevice::getScan()->start(1, false);
            t_scan = millis();
        }
        // going to sleep if long time without connection
        else if (millis()-t_disconnected > DT_SLEEP){
            //Go to sleep now
            Serial.println("Going to sleep now");
            esp_deep_sleep_start();
        }
    }
}
