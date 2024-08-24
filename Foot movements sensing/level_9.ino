/***********************************************************/
/*                Libraries                                */
/***********************************************************/
#include <BluetoothSerial.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "calibration.h"
/*************************************************************************************/
/*                                  BlE Parameters                                   */
/*************************************************************************************/
#define MY_DEVICE_ADDRESS  "24:62:ab:f9:72:9e" // change the address to the hands address if needed
//the names of the hands services used to send commands to the hand 
#define HAND_DIRECT_EXECUTE_SERVICE_UUID     "e0198000-7544-42c1-0000-b24344b6aa70"//sends the command according to the given message written in the code
#define EXECUTE_ON_WRITE_CHARACTERISTIC_UUID "e0198000-7544-42c1-0001-b24344b6aa70"
#define HAND_TRIGGER_SERVICE_UUID            "e0198002-7544-42c1-0000-b24344b6aa70"//tells the hand to move according to the presets saved on the hand that were 
//given using the application -- we dont use it here could be used for different implemintations
#define TRIGGER_ON_WRITE_CHARACTERISTIC_UUID "e0198002-7544-42c1-0001-b24344b6aa70"
#define BACK_STATUS 1
#define FRONT_STATUS 2


static BLEUUID serviceExeUUID(HAND_DIRECT_EXECUTE_SERVICE_UUID);// The remote service we wish to connect to.
static BLEUUID    charExeUUID(EXECUTE_ON_WRITE_CHARACTERISTIC_UUID);// The characteristic of the remote service we are interested in - execute.
static BLEUUID serviceTrigUUID(HAND_TRIGGER_SERVICE_UUID);// The remote service we wish to connect to.
static BLEUUID    charTrigUUID(TRIGGER_ON_WRITE_CHARACTERISTIC_UUID);// The characteristic of the remote service we are interested in - trigger

// Connection parameters:
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = true;
static BLERemoteCharacteristic* pRemoteCharExecute;
static BLERemoteCharacteristic* pRemoteCharTrigger;
static BLEAdvertisedDevice* myDevice;

unsigned char* msg;//the command sent to the hand - we change it each time we want to move the hand differently according to the sensors
//uint8_t preset_id;// which preset will be triggered (12 options)

/*************************************************************************************/
/*                 Go to Sleep and Keep Scanning Setting:                            */
/*************************************************************************************/
#define DT_SCAN    10000 //if not connected to BLE device scan every 10 seconds.
#define DT_SLEEP    (10 * 60 * 1000) //if not connected to BLE device go to sleep after 10 minute.
unsigned long t_scan; //the elapsed time between BLE scanning
unsigned long t_disconnected; //the elapsed time from a disconecting event

/*************************************************************************************/
/*                  BLE Class Definition and Set Callbacks:                          */
/*************************************************************************************/
//for connecting to the phone for calibration
BluetoothSerial SerialBT;

//for connecting to the hand
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
    doScan = true;
    t_disconnected = millis();
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

//pins
int pin_switch = 15;
int pin_front = 13;//purple #PARAMETER
int pin_back = 12;//white #PARAMETER
int pin_right = 27;//orange #PARAMETER
int pin_left = 33;//yellow #PARAMETER
//********GLOBAL_VARIABLES*******
// variables for calibration
int calibration_flag = 0; // 0 if hasnt calibrated yet, 1 if finished calibration
//for force_calibration
int absolute_max_front = 0;//the greatest value recived by the front sensor while calibrating
int max_avg_front = 0;//the  average the front sensor values while leaning forward
int avg_front = 0;// the average of the front sensor values while standing normally

int absolute_max_back = 0;
int max_avg_back = 0;
int avg_back = 0;

int absolute_max_right = 0;
int max_avg_right = 0;
int avg_right = 0;

int absolute_max_left = 0;
int max_avg_left = 0;
int avg_left = 0;
// real time variables - change every cycle
int avg_real_time_front = 0;
int avg_real_time_back = 0;
int avg_real_time_right = 0;
int avg_real_time_left = 0;

//for state machine
int pressed_front = 0;
int pressed_back = 0;
int pressed_right = 0;
int pressed_left = 0;

//for left and right continues pressing- flags currently not used
//int right_flag = 0;
//int left_flag = 0;

int air_counter=0; //checks how long the whole foot is off the ground
int max_air_counter = 15; // the maximum amount of time that could be between two presses to measure a double click.[ms] #PARAMETER
int min_air_counter = 1;// the minimum amount of time that could be between two presses to measure a double click.[ms] #PARAMETER
int last_positions[2] = {0,0};//que for the previous states the leg was in, takes into consideration how long the leg has been in the air.

int fingers_closed = 0; // a flag that represents if the hand is opened or closed

//unsigned char no_task[] =  {5, 0b11111000, 20, 0b00000000, 0b11111000}; //movement length byte, torque,time,active motor, motor direction
unsigned char close_task[] =  {5, 0b11111000, 20, 0b01111000, 0b11111000}; //movement length byte, torque,time,active motor, motor direction
unsigned char open_task[] =  {5, 0b11111000, 20, 0b01111000, 0b00000000}; //movement length byte, torque,time,active motor, motor direction
unsigned char spin_right[] =  {5, 0b11111000, 20, 0b10000000, 0b00000000}; //movement length byte, torque,time,active motor, motor direction 
unsigned char spin_left[] =  {5, 0b11111000, 20, 0b10000000, 0b10000000}; //movement length byte, torque,time,active motor, motor direction
/*************************************************************************************/
/*                              Our functions                                 */
/*************************************************************************************/
//these functions print to the serial output - used by the calibration need to change the function if we change the output
void print_string(const char *message) {
    Serial.print(message);
}

void print_int(const int n) {
  Serial.println(n);
}

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
      std::string value = pRemoteCharExecute->readValue();
      Serial.print("Execute: The characteristic value was: ");
      //Serial.println(value.c_str());*********************************************************************************************************************
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
      std::string value = pRemoteCharTrigger->readValue();
      Serial.print("Trigger: The characteristic value was: ");
      Serial.println(value.c_str());
    }
    connected = true;
    return true;
    
}

void InitBLE() {
  BLEDevice::init("SwitchLeg");
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


void setup() {
  // put your setup code here, to run once:

  ESP.getFreeHeap();//****************************************************find the amount of free space on the heap

  Serial.begin(115200);
  //create a bluetooth service for the phone to connect to while calibrating:
  SerialBT.begin("ESP32_BT"); // Bluetooth device name
 
  // Create the BLE Device
  InitBLE();
  t_scan = millis();

  // enable deep sleep mode for the esp32:
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_4, 1); //1 = High, 0 = Low , same GPIO as the button pin
  t_disconnected = millis();

  // add here your system setup:
}

void loop() {
  // put your main code here, to run repeatedly:
  if (doConnect == true) { //TRUE when we scanned and found the desired BLE server
    if (connectToServer()) Serial.println("We are now connected to the BLE Server."); // connect to the server. TRUE if connection was established
    else Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    doConnect = false; //no need to reconnect to the server
  }

  if (connected) { //TRUE if we are already connected to the server
    // put your main code here, to run repeatedly:
    pressed_front = 0;
    pressed_back = 0;
    pressed_right = 0;
    pressed_left = 0;
    if(!calibration_flag){
      //calibrate the four force sensors at once
        force_calibration(&absolute_max_front, &max_avg_front, &avg_front, &absolute_max_back, &max_avg_back,&avg_back, &absolute_max_right,
                          &max_avg_right,&avg_right, &absolute_max_left, &max_avg_left,&avg_left, pin_front, pin_back, pin_right, pin_left);
        //indicate that the calibration is over
        calibration_flag = 1;
        print_string("Ended calibration \n");
          //***********************************************************************************************************************************************************
    Serial.print(" max_avg_front is  ");
    Serial.println(max_avg_front);
    Serial.print("\n avg_front is  ");
    Serial.println(avg_front);
    Serial.print("\n max_avg_back is  ");
    Serial.println(max_avg_back);
    Serial.print("\n avg_back is  ");
    Serial.println(avg_back);
    //*****************************************************************************for debugging*******************************************************************
    }
    //as long as the switch is on operate nomally
    int switch_value = int(analogRead(pin_switch));
    if(switch_value == 4095){//**************************************************************************************for the switch
      Serial.print("\npin switch is acive");
      Serial.println(analogRead(pin_switch));
      //uses function to get the average samples
      avg_samples(&avg_real_time_front,&avg_real_time_back,&avg_real_time_left,&avg_real_time_right,pin_front,
        pin_back, pin_left, pin_right);


      /*                              double press recognition                             */
      /*************************************************************************************/
      //current status
      if(avg_real_time_back >= (avg_back + 0.8*(max_avg_back - avg_back))){
        pressed_back=1;
        Serial.print("pressed back");//*******************************************************************************************************
      }
      if(avg_real_time_front > (avg_front + 0.99*(max_avg_front - avg_front))){
        pressed_front=1;
        Serial.print("pressed front");//*******************************************************************************************************
      }
      // double press recognition logic, using que and air counter
      if(pressed_front || pressed_back){ //when the foot is not in the air
        if (air_counter < max_air_counter && air_counter > min_air_counter){ // the minimum and maximum amount of air time to recognize double click
          if(last_positions[0] == BACK_STATUS && last_positions[1] == BACK_STATUS && pressed_back == 1){
            //Serial.print("double pressed back \n ");
            if (fingers_closed) {
              pRemoteCharExecute->writeValue(open_task,open_task[0]); // sends the command to the hand
              //Serial.println("OPEN Message was sent");
              fingers_closed=0;
              delay(1500); // the delay is used in order to not stop the closing or opening of the hand by accidentl overwriting the current messege
            }
            else {
              pRemoteCharExecute->writeValue(close_task,close_task[0]);
              //Serial.println("CLOSE Message was sent");
              fingers_closed=1;
              delay(1500);
            }
          }/*
          if(last_positions[0] == FRONT_STATUS && last_positions[1] == FRONT_STATUS && pressed_front == 1){
            Serial.print("double pressed front \n "); // currently not used, could be used to operate a preset
          }
          */
        }
        else{
          //update the que
          last_positions[1] = last_positions[0];
          last_positions[0] = BACK_STATUS * pressed_back + FRONT_STATUS * pressed_front;
        }
        air_counter = 0;//a sensor was pressed therefore airtime is 0
      }
      else{
        air_counter++;
      }


      if(avg_real_time_right > (avg_right + 0.99*(max_avg_right - avg_right))){
        //pressed_right = 1; //could be used for the flags
        pRemoteCharExecute->writeValue(spin_right,spin_right[0]);
        //Serial.println("Spin right Message was sent");
      }
      
      if(avg_real_time_left > (avg_left + 0.99*(max_avg_left - avg_left))){
        //pressed_left = 1; //could be used for the flags
        pRemoteCharExecute->writeValue(spin_left,spin_left[0]);
        //Serial.println("Spin left right Message was sent");
      }
      }//*************************************************************************************for the switch
      
      
    }
  
    else { //not connected
      //scanning for server:
      if((millis()-t_scan>DT_SCAN)){ //not connected
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
