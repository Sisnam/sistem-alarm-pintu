#include <SPI.h>
#include <MFRC522.h>

// Pin Definitions for RC522
#define SS_PIN 10
#define RST_PIN 9

MFRC522 mfrc522(SS_PIN, RST_PIN);

// List of Authorized UIDs TODO:Masukkan UID kartu
const byte authorizedUIDs[][4] = {
    //{0xDE, 0xAD, 0xBE, 0xEF},  // contoh UID Kartu
    //{0x12, 0x34, 0x56, 0x78}   // contoh UID Kartu 
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
