#include <U8g2lib.h>
#include <Wire.h>

// ==== BLE (ESP32 BLE Arduino / NimBLE) ====
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// OLED I2C
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// Pines I2C para ESP32-C3
#define OLED_SDA 8
#define OLED_SCL 9

// ------------------- PROTOCOLO -------------------
typedef struct __attribute__((packed)) {
  uint8_t  type;   // 0 = ANGULO, 1 = NIVEL AGUA
  uint16_t data;   // 0–180 o 0–4095
} Message;

// Potenciómetro
#define POT_PIN 3  // AJUSTAR SEGÚN TU HARDWARE
int lastAngle = -1;
const int THRESH = 2;   // enviar solo si cambia ≥2°
int ang = 0;
volatile int currentAngle = 0;
volatile uint16_t waterRaw = 0;

// Control modo
#define pinBtnModo 6
#define openDelay 5000 //tiempo de apertura/cierre en milisegundos
#define maxAng 180
bool modo = 0;
unsigned long int startTime = 0;
bool stateServo = 0;
// ------------------- BLE: UUIDs tipo UART -------------------
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // cliente -> servidor
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" // servidor -> cliente

BLEServer*        pServer         = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr; // para notificar ángulo al nodo servo
BLECharacteristic* pRxCharacteristic = nullptr; // para recibir nivel de agua
bool deviceConnected      = false;

// ------------------- CALLBACKS BLE -------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("Cliente BLE conectado");
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("Cliente BLE desconectado, re-advertising...");
    display.clearBuffer();
    display.setFont(u8g2_font_6x12_tf);
    display.drawStr(20, 28, "Desconectado...");
    display.drawStr(20, 44, "Buscando...");
    display.sendBuffer();
    BLEDevice::startAdvertising();
  }
};

class MyRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    std::string rxValue = pCharacteristic->getValue();
    if (rxValue.size() != sizeof(Message)) {
      return;
    }

    Message msg;
    memcpy(&msg, rxValue.data(), sizeof(Message));

    if (msg.type == 1) {  // NIVEL DE AGUA
      waterRaw = msg.data;
      float pct = (waterRaw / 4095.0f) * 100.0f;
      Serial.printf("Nivel recibido (BLE): raw=%u (%.1f%%)\n", waterRaw, pct);
    }
  }
};

// ------------------- ENVIAR ANGULO POR BLE -------------------
void sendAngle(int ang) {
  if (!deviceConnected || pTxCharacteristic == nullptr) {
    return; // si no hay cliente conectado, no hacemos nada
  }

  if (ang < 0)   ang = 0;
  if (ang > 180) ang = 180;

  Message msg;
  msg.type = 0;      // ANGULO
  msg.data = ang;

  pTxCharacteristic->setValue((uint8_t*)&msg, sizeof(msg));
  pTxCharacteristic->notify();   // notificar al cliente

  currentAngle = ang;
  Serial.printf("Ángulo enviado (BLE): %d°\n", ang);
}

// ------------------- OLED: Actualización -------------------
void drawOLED() {
  display.clearBuffer();
  float pct = (waterRaw / 4095.0f) * 100.0f;
  display.setFont(u8g2_font_6x12_tf);
  if(modo){
    display.setCursor(1, 12);
    float remening = (openDelay - (millis() - startTime));
    display.printf("Automatico: %.1f s", remening/1000);
  }
  else
    display.drawStr(1, 12, "Manual");
  // Ángulo
  display.setCursor(1, 28);
  display.printf("Ang: %3d%c", currentAngle, '°');

  // Nivel
  display.setCursor(1, 42);
  display.printf("Nivel: %5.1f%%", pct);

  // Barra de nivel
  int barW = 120;
  int fill = (int)(barW * pct / 100.0);
  display.drawFrame(1, 48, barW, 14);
  display.drawBox(2, 49, fill, 12);

  display.sendBuffer();
}

// ------------------- SETUP -------------------
void setup() {
  Serial.begin(115200);
  delay(1500);
  pinMode(pinBtnModo, INPUT_PULLUP);
  // I2C + OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(20, 36, "Iniciando...");
  display.sendBuffer();
  delay(500);

  analogReadResolution(12);

  // ---- BLE SERVER ----
  BLEDevice::init("VALVE_CONTROL");  // nombre que verás al escanear BLE
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  // TX: servidor -> cliente (notify) para ANGULO
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );
  pTxCharacteristic->addDescriptor(new BLE2902());

  // RX: cliente -> servidor (write) para NIVEL AGUA
  pRxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_RX,
                        BLECharacteristic::PROPERTY_WRITE
                      );
  pRxCharacteristic->setCallbacks(new MyRxCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("BLE listo, esperando cliente...");
  display.clearBuffer();
  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(20, 36, "Conectando...");
  display.sendBuffer();
  
}

// ------------------- LOOP -------------------
unsigned long lastOLED = 0;

void loop() {
  int raw = analogRead(POT_PIN);
  if(!digitalRead(pinBtnModo))
    modo = !modo;
  if((millis() - startTime> openDelay)&&modo){
    startTime = millis();
    stateServo = !stateServo;
    ang = maxAng*stateServo;
  }
  else if(!modo){
    ang = map(raw, 0, 4095, 0, maxAng);
  }
  if (lastAngle < 0 || abs(ang - lastAngle) >= THRESH) {
    lastAngle = ang;
    sendAngle(ang);
  }

  if ((millis() - lastOLED >= 200)&&deviceConnected) {
    lastOLED = millis();
    drawOLED();
  }

  delay(20);
}
