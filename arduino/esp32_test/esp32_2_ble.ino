/*
 * FFC Team
 * FITHOU - VN
 * BLE handling
*/

/*
 * Bluetooth Low Energy
 * Require pairing
 * 20 bytes per packet limited
 * Service UUID must be saved
 */

//#include <BluetoothSerial.h>
//#include <SimpleBLE.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

//BluetoothSerial BT;
//SimpleBLE ble;
BLEServer *ble_srv = NULL;
BLECharacteristic *ble_txchar;
BLECharacteristic* ble_rxchar;
bool ble_connected = false;
bool ble_old_connected = false;
uint8_t ble_txval = 0x0;

/* SERVICE, CHARACTERISTIC TX AND RX UUID */
#define BLE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

/* BLE server callbacks */
class MyBLEServerCbs: public BLEServerCallbacks {
    void onConnect(BLEServer* pserver) {
      Serial.println("BLE: connected.");
      ble_connected = true;
    };
    void onDisconnect(BLEServer* pserver) {
      Serial.println("BLE: disconnected.");
      ble_connected = false;
    }
};

/* BLE reading callbacks */
class MyBLERXCbs: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pchar) {
      std::string rxval = ble_rxchar->getValue();
      if (rxval.length() > 0) {
        Serial.print("BLE: received = ");
        for (int i = 0; i < rxval.length(); i++) {
          Serial.print(rxval[i]);
        }
        Serial.println();
      }
    }
};

void setup_ble(void) {
  /*
    if (BT.begin("MyESPBoard")) {
      Serial.println("Bluetooth stated. Name: 'My ESP Board'");
    }
  */
  /*
    if (ble.begin("MyESPBoard")) {
      Serial.println("Bluetooth LE stated. Name: 'My ESP Board'");
    } else {
      Serial.println("Bluetooth LE failed.");
    }
  */
  Serial.print("BLE: starting...");

  BLEDevice::init("MyESPBoard");
  ble_srv = BLEDevice::createServer();
  ble_srv->setCallbacks(new MyBLEServerCbs());

  BLEService *pservice = ble_srv->createService(BLE_UUID);

  Serial.println(" OK");

  ble_txchar = pservice->createCharacteristic(BLE_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  ble_txchar->addDescriptor(new BLE2902());

  ble_rxchar = pservice->createCharacteristic(BLE_UUID, BLECharacteristic::PROPERTY_WRITE);
  ble_rxchar->setCallbacks(new MyBLERXCbs());

  pservice->start();
  ble_srv->getAdvertising()->start();

  Serial.println("BLE: done, waiting a client to notify");

}

void loop_ble (void) {
}


