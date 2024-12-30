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
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <Servo.h>

// Servo
Servo myservo;
#define SERVO_PIN 9

// Pin Definitions for RC522
#define SS_PIN 53
#define RST_PIN 5

MFRC522 mfrc522(SS_PIN, RST_PIN);

// Pin Definitions for Common Cathode RGB LED
#define RED_PIN 11
#define GREEN_PIN 12
#define BLUE_PIN 8

// List of Authorized UIDs TODO:Masukkan UID kartu
const byte authorizedUIDs[][4] = {
  //{0xDE, 0xAD, 0xBE, 0xEF},  // contoh UID Kartu
  { 0x8B, 0x7D, 0x30, 0x02 },
  { 0xF3, 0xE8, 0x53, 0x14 }
};

const int numOfUIDs = sizeof(authorizedUIDs) / sizeof(authorizedUIDs[0]);

// Queue
#define UID_LENGTH 4

QueueHandle_t xQueueAccessGranted;

typedef struct
{
  bool accessGranted;
  byte detectedUID[UID_LENGTH];
} DataPacket_t;


// Function prototypes
void TaskReadRFID(void *pvParameters);
void TaskHandleData(void *pvParameters);

void setup() {
  Serial.begin(9600);   // UART ke ATMega
  Serial1.begin(9600);  // UART ke Dashboard
  SPI.begin();
  mfrc522.PCD_Init();

  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  
  // Turn off all LEDs initially
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, LOW);

  Serial.println("RFID Reader Ready...");
  
  // Turn on Blue LED for idle
  digitalWrite(BLUE_PIN, HIGH);

  // Initialize Servo
  myservo.attach(SERVO_PIN);
  myservo.write(90);  // door close

  // Initialize Queue
  xQueueAccessGranted = xQueueCreate(10, sizeof(DataPacket_t));

  // Create FreeRTOS tasks
  xTaskCreate(TaskReadRFID, "ReadRFID", 256, NULL, 1, NULL);
  xTaskCreate(TaskHandleData, "HandleData", 256, NULL, 1, NULL);
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

// Task untuk menangani data dan mengirim log
void TaskHandleData(void *pvParameters) {
  DataPacket_t receivedPacket;

  for (;;) {
    if (xQueueReceive(xQueueAccessGranted, &receivedPacket, pdMS_TO_TICKS(100)) == pdPASS) { 
      // Turn off Blue LED while processing
      digitalWrite(BLUE_PIN, LOW);   
        
      // Kirim log ke dashboard melalui Serial
      String UID = convertUIDToString(receivedPacket.detectedUID);
      String accessGranted = receivedPacket.accessGranted ? "TRUE" : "FALSE";
      
      delay(25);

      // Kirim status ke ATmega melalui Serial1 (TRUE/FALSE saja)
      sendToAtmega(receivedPacket.accessGranted);

      // Kirim status ke Dashboard melalui Serial (UID dan TRUE/FALSE)
      sendLogToDashboard(UID, accessGranted);

      // Rotasi servo
      if (receivedPacket.accessGranted) {
        rotateServo();
      }
      else{
          digitalWrite(BLUE_PIN, LOW);
          digitalWrite(RED_PIN, HIGH);
          digitalWrite(GREEN_PIN, LOW);
          delay(4000);
          // After handling, turn back to idle (Blue ON, others OFF)
          digitalWrite(RED_PIN, LOW);
          digitalWrite(GREEN_PIN, LOW);
          digitalWrite(BLUE_PIN, HIGH);
      }
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);  // Delay untuk menjaga task tetap berjalan
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

String convertUIDToString(byte uid[]) {
  // Format UID menjadi string tanpa separator
  String uidString = "";

  for (byte i = 0; i < UID_LENGTH; i++) {
    if (uid[i] < 0x10) {
      uidString += "0";  // Tambahkan leading zero jika perlu
    }
    uidString += String(uid[i], HEX);
  }

  return uidString;
}

// Fungsi untuk mengirim log akses ke Dashboard melalui Serial1
void sendLogToDashboard(String uid, String status) {
  String log = "UID: " + uid + " | ACCESS: " + status;
  Serial.println(log);  // Kirim log ke Dashboard
}

// Fungsi untuk mengirim data ke ATmega melalui Serial1 (TRUE/FALSE saja)
void sendToAtmega(bool accessGranted) {
  if (accessGranted) {
    Serial1.println("TRUE");
  } else {
    Serial1.println("FALSE");
  }
}

void rotateServo() {
  digitalWrite(GREEN_PIN, HIGH);
  int pos;
  for (pos = 90; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(5);
  }

  delay(4000);

  for (pos = 0; pos <= 90; pos += 1) {
    myservo.write(pos);
    delay(5);
  }

  // After handling, turn back to idle (Blue ON, others OFF)
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);
  digitalWrite(BLUE_PIN, HIGH);
}
