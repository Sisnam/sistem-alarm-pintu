/*
 * This is a MFRC522 library example; for further details and other examples see: https://github.com/miguelbalboa/rfid 
 *
 * When the Arduino and the MFRC522 module are connected (see the pin layout below), load this sketch into Arduino IDE
 * then verify/compile and upload it. To see the output: use Tools, Serial Monitor of the IDE (hit Ctrl+Shft+M). When
 * you present a PICC (that is: a RFID Tag or Card) at reading distance of the MFRC522 Reader/PCD, the serial output
 * will show the type, and the NUID if a new card has been detected. Note: you may see "Timeout in communication" messages
 * when removing the PICC from reading distance too early.
 * 
 * @license Released into the public domain.
 * 
 * Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      Arduino       Arduino   Arduino    Arduino          Arduino
 *             Reader/PCD   Uno/101       Mega      Nano v3    Leonardo/Micro   Pro Micro
 * Signal      Pin          Pin           Pin       Pin        Pin              Pin
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          9             5         D9         RESET/ICSP-5     RST
 * SPI SS      SDA(SS)      10            53        D10        10               10
 * SPI MOSI    MOSI         11 / ICSP-4   51        D11        ICSP-4           16
 * SPI MISO    MISO         12 / ICSP-1   50        D12        ICSP-1           14
 * SPI SCK     SCK          13 / ICSP-3   52        D13        ICSP-3           15
 *
 * More pin layouts for other boards can be found here: https://github.com/miguelbalboa/rfid#pin-layout
 */

#include <SPI.h>
#include <MFRC522.h>
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include <queue.h>

// Pin Definitions for RC522
#define SS_PIN 53
#define RST_PIN 5

MFRC522 mfrc522(SS_PIN, RST_PIN);

// List of Authorized UIDs TODO:Masukkan UID kartu
const byte authorizedUIDs[][4] = {
  //{0xDE, 0xAD, 0xBE, 0xEF},  // contoh UID Kartu
  { 0x8B, 0x7D, 0x30, 0x02 },
  { 0xF3, 0xE8, 0x53, 0x14 }
};

const int numOfUIDs = sizeof(authorizedUIDs) / sizeof(authorizedUIDs[0]);
#define UID_LENGTH 4

// Queue
QueueHandle_t xQueueAccessGranted;
QueueHandle_t xQueueSystemStatus;

// Structure for RFID access status
typedef struct
{
  bool accessGranted;
  byte detectedUID[UID_LENGTH];
} DataPacket_t;

// Structure for system status
typedef struct {
  bool doorStatus;     // true = open, false = closed
  bool alarmStatus;    // true = active, false = inactive
  bool systemStatus;   // true = armed, false = disarmed
} SystemStatus_t;

// Global variable to track system state
bool currentSystemState = false;

// Function prototypes
void TaskReadRFID(void *pvParameters);
void TaskReceiveAtmelStatus(void *pvParameters);
void TaskHandleRFIDControl(void *pvParameters);
void TaskUpdateSerial(void *pvParameters);


void setup() {
  Serial.begin(9600);   // Debug Serial
  Serial1.begin(9600);  // Communication with Atmel
  SPI.begin();
  mfrc522.PCD_Init();

  Serial.println("Dashboard Ready...");

  // Create queues
  xQueueAccessGranted = xQueueCreate(10, sizeof(DataPacket_t));
  xQueueSystemStatus = xQueueCreate(10, sizeof(SystemStatus_t));

  // Create FreeRTOS tasks
  xTaskCreate(TaskReadRFID, "ReadRFID", 256, NULL, 1, NULL);
  xTaskCreate(TaskReceiveAtmelStatus, "ReceiveStatus", 256, NULL, 1, NULL);
  xTaskCreate(TaskHandleRFIDControl, "HandleRFID", 128, NULL, 1, NULL);
  xTaskCreate(TaskUpdateSerial, "UpdateSerial", 256, NULL, 1, NULL);
}

void loop() {
  // FreeRTOS handles the tasks; loop can remain empty.
}

// Task untuk membaca kartu RFID
void TaskReadRFID(void *pvParameters) {
  DataPacket_t dataPacket;

  for (;;) {
    // Cek jika ada kartu RFID
    if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
      byte readUID[UID_LENGTH];
      for (byte i = 0; i < UID_LENGTH; i++) {
        readUID[i] = mfrc522.uid.uidByte[i];
      }

      strcpy(dataPacket.detectedUID, readUID);
      dataPacket.accessGranted = checkAccess(readUID);

      if (xQueueSend(xQueueAccessGranted, &dataPacket, pdMS_TO_TICKS(100)) == pdPASS) {
        printf("Sent: UUID: %s, access=%d\n", dataPacket.detectedUID, dataPacket.accessGranted);
      }

      mfrc522.PICC_HaltA();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay untuk mengurangi beban CPU
  }
}

// Task to handle RFID access and system control
void TaskHandleRFIDControl(void *pvParameters) {
  DataPacket_t receivedPacket;
  
  for (;;) {
    if (xQueueReceive(xQueueAccessGranted, &receivedPacket, pdMS_TO_TICKS(100)) == pdPASS) {
      if (receivedPacket.accessGranted) {
        // Toggle system state
        currentSystemState = !currentSystemState;
        
        // Send new state to Atmel
        Serial1.print("SYSTEM|");
        Serial1.println(currentSystemState ? "1" : "0");
        
        Serial.print("System state changed to: ");
        Serial.println(currentSystemState ? "ARMED" : "DISARMED");
      } else {
        Serial.println("Unauthorized card detected!");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to receive status updates from Atmel
void TaskReceiveAtmelStatus(void *pvParameters) {
  SystemStatus_t status;
  String receivedString;

  for (;;) {
    if (Serial1.available() > 0) {
      receivedString = Serial1.readStringUntil('\n');
      
      // Expected format: "STATUS|door|alarm|system"
      // Example: "STATUS|1|0|1"
      if (receivedString.startsWith("STATUS|")) {
        String parts[4];
        int partIndex = 0;
        int lastIndex = 7;  // Length of "STATUS|"
        
        // Parse the status string
        for (int i = lastIndex; i < receivedString.length() && partIndex < 3; i++) {
          if (receivedString.charAt(i) == '|') {
            parts[partIndex] = receivedString.substring(lastIndex, i);
            lastIndex = i + 1;
            partIndex++;
          }
        }
        // Get the last part
        parts[partIndex] = receivedString.substring(lastIndex);
        
        // Update status structure
        status.doorStatus = (parts[0] == "1");
        status.alarmStatus = (parts[1] == "1");
        status.systemStatus = (parts[2] == "1");
        
        // Send to queue
        xQueueSend(xQueueSystemStatus, &status, pdMS_TO_TICKS(100));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Task to update serial monitor with current status
void TaskUpdateSerial(void *pvParameters) {
  SystemStatus_t status;
  
  for (;;) {
    if (xQueueReceive(xQueueSystemStatus, &status, pdMS_TO_TICKS(100)) == pdPASS) {
      Serial.println("\n--- System Status ---");
      Serial.print("Door: ");
      Serial.println(status.doorStatus ? "TERBUKA" : "TERTUTUP");
      Serial.print("Alarm: ");
      Serial.println(status.alarmStatus ? "AKTIF" : "NONAKTIF");
      Serial.print("System: ");
      Serial.println(status.systemStatus ? "AKTIF" : "NONAKTIF");
      Serial.println("------------------\n");
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Fungsi untuk memeriksa UID dengan list akses sah
bool checkAccess(byte uid[]) {
  for (int i = 0; i < numOfUIDs; i++) {
    bool match = true;
    for (int j = 0; j < UID_LENGTH; j++) {
      if (uid[j] != authorizedUIDs[i][j]) {
        match = false;
        break;
      }
    }
    if (match) return true;  // UID cocok
  }
  return false;  // UID tidak cocok
}
