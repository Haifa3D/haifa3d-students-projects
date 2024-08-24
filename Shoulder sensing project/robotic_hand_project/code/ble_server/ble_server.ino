/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
    updates by chegewara
*/
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define HAND_DIRECT_EXECUTE_SERVICE_UUID     "e0198000-7544-42c1-0000-b24344b6aa70"
#define EXECUTE_ON_WRITE_CHARACTERISTIC_UUID "e0198000-7544-42c1-0001-b24344b6aa70"
#define HAND_TRIGGER_SERVICE_UUID            "e0198002-7544-42c1-0000-b24344b6aa70"
//given using the application -- we dont use it here could be used for different implemintations
#define TRIGGER_ON_WRITE_CHARACTERISTIC_UUID "e0198002-7544-42c1-0001-b24344b6aa70"

BLEServer *pServer;


class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pserver){
    Serial.println("Connected!");
  }
  void onDisconnect(BLEServer *pserver){
    Serial.println("Disconnected!");
  }
};

class MyCharacteristicCallBack : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    Serial.print("data is received from UUID: "); 
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(" "); 
    Serial.print(pCharacteristic->getValue().c_str()); 
    Serial.print("\n"); 
  }

  void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    Serial.print("data was read from UUID: "); 
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(" "); 
    char buff[100];
    strcpy(buff,pCharacteristic->getValue().c_str());
    for(int i = 0; buff[i] != '\0'; i++){
      Serial.print(buff[i]); 
      Serial.print(" "); 
    }
    Serial.print("\n"); 
  }
};


void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("Pretend Im The Hand");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *ExeOnWriteService = pServer->createService(
    HAND_DIRECT_EXECUTE_SERVICE_UUID);
  BLECharacteristic *ExeOnWriteCharacteristic = ExeOnWriteService->createCharacteristic(
    EXECUTE_ON_WRITE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
    );
  ExeOnWriteCharacteristic->setValue("Hello World says Neil");
  ExeOnWriteCharacteristic->setCallbacks(new MyCharacteristicCallBack());
  ExeOnWriteService->start();
  
  BLEService *TriggerOnWriteService = pServer->createService(
    HAND_TRIGGER_SERVICE_UUID);
  BLECharacteristic *TriggerOnWriteCharacteristic = TriggerOnWriteService->createCharacteristic(
    TRIGGER_ON_WRITE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE
    );
  TriggerOnWriteCharacteristic->setValue("Goodbye World says Buzz");
  TriggerOnWriteCharacteristic->setCallbacks(new MyCharacteristicCallBack());
  TriggerOnWriteService->start();
  

  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(HAND_DIRECT_EXECUTE_SERVICE_UUID);
  pAdvertising->addServiceUUID(HAND_TRIGGER_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
  Serial.print("connected devices: ");
  Serial.print(pServer->getConnectedCount());
  Serial.print("\n");

}