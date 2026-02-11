#include <Arduino.h>
#include <Wire.h>

// ==== Servo (para ESP32) ====
// Si usás otra librería, cámbiala aquí:
#include <ESP32Servo.h>

// ==== BLE ====
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// ------------------- HARDWARE -------------------
#define SERVO_PIN   6   // AJUSTAR PIN DEL SERVO
#define LEVEL_PIN   3   // AJUSTAR PIN ADC DEL SENSOR DE NIVEL

Servo valveServo;

// ------------------- PROTOCOLO -------------------
typedef struct __attribute__((packed)) {
  uint8_t  type;  // 0 = ANGULO, 1 = NIVEL AGUA
  uint16_t data;  // 0–180 o 0–4095 según type
} Message;

// ------------------- UUIDs (MISMOS QUE EN CONTROL) -------------------
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // cliente -> servidor (WRITE)
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // servidor -> cliente (NOTIFY)

// ------------------- VARIABLES BLE -------------------
static BLEAddress*          pServerAddress = nullptr;
static bool                 deviceConnected = false;
static bool                 doScan = true;

BLERemoteCharacteristic*    pRemoteTxCharacteristic = nullptr; // para NOTIFY (ángulo)
BLERemoteCharacteristic*    pRemoteRxCharacteristic = nullptr; // para WRITE (nivel)

// Ángulo actual objetivo
volatile int targetAngle = 0;

// ----------- CALLBACK: Notificación desde el servidor (ángulo) -----------
static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify
) {
  if (length != sizeof(Message)) return;

  Message msg;
  memcpy(&msg, pData, sizeof(Message));

  if (msg.type == 0) {  // ANGULO
    int ang = msg.data;
    if (ang < 0)   ang = 0;
    if (ang > 180) ang = 180;
    targetAngle = ang;
    Serial.printf("Ángulo recibido (BLE): %d°\n", ang);
  }
}

// ----------- CALLBACKS DEL CLIENTE (conexión / desconexión) -----------
class MyClientCallbacks : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) override {
    Serial.println("Conectado a VALVE_CONTROL");
    deviceConnected = true;
  }

  void onDisconnect(BLEClient* pClient) override {
    Serial.println("Desconectado de VALVE_CONTROL");
    deviceConnected = false;
    doScan = true;  // volver a escanear
  }
};

// ----------- CALLBACK SCAN: Buscar "VALVE_CONTROL" -----------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    // Buscamos por nombre, tal como se configuró en el servidor
    std::string name = advertisedDevice.getName();
    if (name == "VALVE_CONTROL") {
      Serial.print("Encontrado servidor: ");
      Serial.println(name.c_str());

      // Guardamos la dirección y paramos el scan
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      advertisedDevice.getScan()->stop();
      doScan = false;
    }
  }
};

// ----------- CONECTAR AL SERVIDOR VALVE_CONTROL -----------
bool connectToServer() {
  if (!pServerAddress) {
    Serial.println("No tengo direccion del servidor aún");
    return false;
  }

  Serial.print("Conectando a servidor BLE en: ");
  Serial.println(pServerAddress->toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallbacks());

  if (!pClient->connect(*pServerAddress)) {
    Serial.println("Fallo al conectar :(");
    return false;
  }

  Serial.println("Conectado, buscando servicio...");

  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.println("No se encontró el servicio UART en el servidor");
    pClient->disconnect();
    return false;
  }

  // Característica TX (servidor -> cliente) : notify (ángulo)
  pRemoteTxCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_TX);
  if (pRemoteTxCharacteristic == nullptr) {
    Serial.println("No se encontró la característica TX");
    pClient->disconnect();
    return false;
  }

  // Característica RX (cliente -> servidor) : write (nivel)
  pRemoteRxCharacteristic = pRemoteService->getCharacteristic(CHARACTERISTIC_UUID_RX);
  if (pRemoteRxCharacteristic == nullptr) {
    Serial.println("No se encontró la característica RX");
    pClient->disconnect();
    return false;
  }

  // Activar notificaciones de TX (ángulo)
  if (pRemoteTxCharacteristic->canNotify()) {
    pRemoteTxCharacteristic->registerForNotify(notifyCallback);
  }

  Serial.println("Servicio y características configurados correctamente");
  deviceConnected = true;
  return true;
}

// ----------- ENVIAR NIVEL DEL AGUA AL CONTROL -----------
void sendWaterLevel() {
  if (!deviceConnected || pRemoteRxCharacteristic == nullptr) return;

  uint16_t raw = analogRead(LEVEL_PIN); // 0–4095

  Message msg;
  msg.type = 1;    // NIVEL AGUA
  msg.data = raw;

  pRemoteRxCharacteristic->writeValue((uint8_t*)&msg, sizeof(msg), false);
  float pct = (raw / 4095.0f) * 100.0f;
  Serial.printf("Nivel enviado: raw=%u (%.1f%%)\n", raw, pct);
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(1500);

  // Servo
  valveServo.attach(SERVO_PIN);
  valveServo.write(0);

  // ADC
  analogReadResolution(12);

  // BLE CLIENT
  BLEDevice::init("VALVE_SERVO"); // nombre del cliente (solo informativo)
  
  BLEScan* pScan = BLEDevice::getScan();
  pScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);

  Serial.println("Escaneando VALVE_CONTROL...");
  pScan->start(5, false);  // 5 segundos, no continuar tras callback

  // Si lo encuentra, doScan se pone en false y pServerAddress se setea
  // Intentamos conectar
  if (pServerAddress != nullptr) {
    connectToServer();
  } else {
    Serial.println("No se encontró VALVE_CONTROL en el primer scan");
  }
}

// ------------------- LOOP -------------------
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 200; // ms

void loop() {
  // Si no estamos conectados y necesitamos escanear de nuevo
  if (!deviceConnected && doScan) {
    BLEScan* pScan = BLEDevice::getScan();
    Serial.println("Re-escanenando VALVE_CONTROL...");
    pScan->start(5, false);

    if (pServerAddress != nullptr) {
      connectToServer();
    }
  }

  // Mover servo hacia el último ángulo recibido
  valveServo.write(targetAngle);

  // Enviar periódicamente el nivel del agua al control
  unsigned long now = millis();
  if (deviceConnected && (now - lastSend >= SEND_INTERVAL)) {
    lastSend = now;
    sendWaterLevel();
  }

  delay(10);
}
