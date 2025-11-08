/*
 * ESP32_S3_Gripper_BLE.ino
 * Control de Gripper vía BLE para ESP32-S3 LoRa WiFi V3
 * GPIO19 para servo GXservo
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ESP32Servo.h>

// UUIDs para BLE
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// Configuración del Servo
Servo gripperServo;
const int SERVO_PIN = 19;
const int GRIPPER_OPEN_ANGLE = 80;
const int GRIPPER_CLOSE_ANGLE = 0;
const int LED_PIN = 48;

// Variables globales
bool gripperOpen = true;
bool deviceConnected = false;
BLECharacteristic *pCharacteristic;

// DECLARACIONES DE FUNCIONES (antes de usarlas)
void processCommand(char cmd);
void openGripper();
void closeGripper();

// Callback para conexiones
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println(">>> CLIENTE BLE CONECTADO <<<");
      digitalWrite(LED_PIN, HIGH);
    }

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println(">>> CLIENTE BLE DESCONECTADO <<<");
      digitalWrite(LED_PIN, LOW);
      delay(500);
      BLEDevice::startAdvertising();
      Serial.println("Esperando nueva conexion...");
    }
};

// Callback para recibir comandos
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String value = pCharacteristic->getValue().c_str();  // CORREGIDO

      if (value.length() > 0) {
        char command = value[0];
        
        Serial.print("[BLE] Comando recibido: '");
        Serial.print(command);
        Serial.println("'");
        
        processCommand(command);
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("   ESP32-S3 Gripper Control BLE");
  Serial.println("   LoRa WiFi V3 - GPIO19");
  Serial.println("========================================");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("\n[1/3] Configurando servo...");
  Serial.print("      Pin: GPIO");
  Serial.println(SERVO_PIN);
  
  gripperServo.attach(SERVO_PIN);
  gripperServo.write(GRIPPER_OPEN_ANGLE);
  
  Serial.println("      Servo inicializado");
  Serial.print("      Posicion inicial: ");
  Serial.print(GRIPPER_OPEN_ANGLE);
  Serial.println(" grados (ABIERTO)");
  
  Serial.println("\n[2/3] Inicializando BLE...");
  BLEDevice::init("ESP32_Gripper");
  
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setCallbacks(new MyCallbacks());
  
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  
  Serial.println("      BLE inicializado correctamente");
  Serial.println("      Nombre: ESP32_Gripper");
  
  BLEDevice::startAdvertising();
  Serial.println("\n[3/3] Advertising iniciado");
  Serial.println("      Dispositivo visible por BLE");
  
  Serial.println("\n========================================");
  Serial.println("Sistema listo");
  Serial.println("Esperando conexion desde Python...");
  Serial.println("========================================\n");
  
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
}

void loop() {
  if (deviceConnected) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  delay(50);
}

void processCommand(char cmd) {
  String response = "";
  
  switch(cmd) {
    case 'O':
    case 'o':
      openGripper();
      response = "OK:OPEN";
      Serial.println("-> Respuesta: OK:OPEN");
      break;
      
    case 'C':
    case 'c':
      closeGripper();
      response = "OK:CLOSE";
      Serial.println("-> Respuesta: OK:CLOSE");
      break;
      
    case 'S':
    case 's':
      response = gripperOpen ? "STATUS:OPEN" : "STATUS:CLOSE";
      Serial.print("-> Estado actual: ");
      Serial.println(response);
      break;
      
    default:
      response = "ERROR:UNKNOWN";
      Serial.println("Comando desconocido");
      break;
  }
  
  if (deviceConnected) {
    pCharacteristic->setValue(response.c_str());
    pCharacteristic->notify();
  }
}

void openGripper() {
  if (!gripperOpen) {
    Serial.println("-> Abriendo gripper...");
    Serial.print("   Moviendo servo a: ");
    Serial.print(GRIPPER_OPEN_ANGLE);
    Serial.println(" grados");
    
    gripperServo.write(GRIPPER_OPEN_ANGLE);
    gripperOpen = true;
    delay(500);
    
    Serial.println("Gripper ABIERTO");
  } else {
    Serial.println("Gripper ya estaba abierto");
  }
}

void closeGripper() {
  if (gripperOpen) {
    Serial.println("-> Cerrando gripper...");
    Serial.print("   Moviendo servo a: ");
    Serial.print(GRIPPER_CLOSE_ANGLE);
    Serial.println(" grados");
    
    gripperServo.write(GRIPPER_CLOSE_ANGLE);
    gripperOpen = false;
    delay(500);
    
    Serial.println("Gripper CERRADO");
  } else {
    Serial.println("Gripper ya estaba cerrado");
  }
}
