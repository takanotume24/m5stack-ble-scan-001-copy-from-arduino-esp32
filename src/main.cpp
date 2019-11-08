/*
   Based on Neil Kolban example for IDF:
   https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleScan.cpp
   Ported to Arduino ESP32 by Evandro Copercini
*/
#include <Arduino.h>
#include <BLEAdvertisedDevice.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>

int scanTime = 5;  // In seconds
BLEScan* pBLEScan;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    char uuid[60];
    BLEAddress ble_addr = advertisedDevice.getAddress();
    int rssi = advertisedDevice.getRSSI();
    std::string data = advertisedDevice.getManufacturerData();
    if (data.length() != 25) return;
    if (data[0] != 0x4c) return;
    if (data[1] != 0x00) return;
    if (data[2] != 0x02) return;
    if (data[3] != 0x15) return;
    if ((data[0] == 0x4c) && (data[1] == 0x00) && (data[2] == 0x02) &&
        (data[3] == 0x15)) {
      sprintf(uuid,
              "%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%"
              "02X%02X",
              data[4], data[5], data[6], data[7], data[8], data[9], data[10],
              data[11], data[12], data[13], data[14], data[15], data[16],
              data[17], data[18], data[19]);
      int major = (int)(((data[20] & 0xff) << 8) + (data[21] & 0xff));
      int minor = (int)(((data[22] & 0xff) << 8) + (data[23] & 0xff));
      signed char power = (signed char)(data[24] & 0xff);
      Serial.printf("addr=%s rssi=%d uuid=%s,major=%d,minor=%d,power=%d\n",
                    ble_addr.toString().c_str(), rssi, uuid, major, minor,
                    power);
    }
  }
};

void setup() {
  Serial.begin(115200);
  Serial.println("Scanning...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();  // create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  // active scan uses more power, but get results faster
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);  // less or equal setInterval value
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEScanResults foundDevices = pBLEScan->start(scanTime, false);
  Serial.print("Devices found: ");
  Serial.println(foundDevices.getCount());
  Serial.println("Scan done!");
  // delete results fromBLEScan buffer to release memory
  pBLEScan->clearResults();
  delay(2000);
}