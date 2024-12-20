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

// Pin Definitions for RC522
#define SS_PIN 53
#define RST_PIN 5

MFRC522 mfrc522(SS_PIN, RST_PIN);

// List of Authorized UIDs TODO:Masukkan UID kartu
const byte authorizedUIDs[][4] = {
    //{0xDE, 0xAD, 0xBE, 0xEF},  // contoh UID Kartu
    {0x8B, 0x7D, 0x30, 0x02},
    {0xF3, 0xE8, 0x53, 0x14}
};

const int numOfUIDs = sizeof(authorizedUIDs) / sizeof(authorizedUIDs[0]);

void setup() {
    Serial.begin(9600);   // UART ke ATMega
    Serial1.begin(9600);  // UART ke Arduino Dashboard
    SPI.begin();
    mfrc522.PCD_Init();

    Serial.println("RFID Reader Ready...");
    Serial1.println("RFID Reader Log Ready...");
}

void loop() {
    // Cek jika ada kartu RFID
    if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
        delay(100);
        return;
    }

    // Membaca UID
    byte readUID[4];
    for (byte i = 0; i < 4; i++) {
        readUID[i] = mfrc522.uid.uidByte[i];
    }

    // Format UID menjadi string
    String uidString = "";
    for (byte i = 0; i < 4; i++) {
        uidString += String(readUID[i], HEX);
        if (i < 3) uidString += ":";
    }

    // Periksa akses UID
    bool accessGranted = checkAccess(readUID);

    // Kirim status ke ATMega melalui Serial
    if (accessGranted) {
        Serial.println("ACCESS: TRUE");
        sendLogToDashboard(uidString, "TRUE");
    } else {
        Serial.println("ACCESS: FALSE");
        sendLogToDashboard(uidString, "FALSE");
    }

    delay(1000);
    mfrc522.PICC_HaltA();
}

// Fungsi untuk memeriksa UID dengan list akses sah
bool checkAccess(byte uid[]) {
    for (int i = 0; i < numOfUIDs; i++) {
        bool match = true;
        for (int j = 0; j < 4; j++) {
            if (uid[j] != authorizedUIDs[i][j]) {
                match = false;
                break;
            }
        }
        if (match) return true;  // UID cocok
    }
    return false;  // UID tidak cocok
}

// Fungsi untuk mengirim log akses ke Dashboard melalui Serial1
void sendLogToDashboard(String uid, String status) {
    String log = "UID: " + uid + " | ACCESS: " + status;
    Serial1.println(log);  // Kirim log ke Dashboard
}
