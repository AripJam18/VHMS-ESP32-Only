#include <WiFi.h>
#include "Nextion.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Adafruit_Thermal.h"
#include <Messages.h>

// Variabel untuk waktu
unsigned long previousMillis1 = 0;
const long interval1 = 5000; // Interval pengiriman data (5 detik)

// Konstanta untuk protokol komunikasi
const uint8_t ESCIT = 0x10;
const uint8_t STX = 2;
const uint8_t ETX = 3;
const uint8_t maxMsgLen = 100;

// WiFi credentials
const char* password = "password123";
char selectedSSID[32] = "";  // Placeholder for selected SSID

// Pin configurations
#define RX1 4  // Communication with Arduino Mega
#define TX1 5
#define RX2 16 // Communication with Nextion
#define TX2 17
#define PRINTER_RX 27 // Printer RX
#define PRINTER_TX 14 // Printer TX

// SoftwareSerial and Printer
SoftwareSerial printerSerial(PRINTER_RX, PRINTER_TX);
Adafruit_Thermal printer(&printerSerial);

// WiFi and client
WiFiClient client;

// Buffers and variables
#define BUFFER_SIZE 10
String payloadBuffer[BUFFER_SIZE];
String buffer = "";  // Buffer untuk menampung data sementara
int bufferIndex = 0;
bool shouldSendData = false;
unsigned long startTime = 0;

// Unit name for the printer header
const char* unitName = "HD78140KM";

// Nextion components
NexButton BtnStart = NexButton(0, 1, "BtnStart");
NexButton BtnStop = NexButton(0, 2, "BtnStop");
NexButton BtnScan = NexButton(0, 25, "BtnScan");
NexCombo CmbSSID = NexCombo(0, 30, "CmbSSID");
NexText TxtStatus = NexText(0, 28, "TxtStatus");
NexText TxtSSID = NexText(0, 29, "TxtSSID");
NexText TxtData = NexText(0, 10, "TxtData");
NexText TxtKirim = NexText(0, 12, "TxtKirim");
NexText TxtJam = NexText(0, 3, "TxtJam");
NexNumber nRit = NexNumber(0, 27, "nRit");
NexText TxtClient = NexText(0, 15, "TxtClient");
NexText TxtTanggal = NexText(0, 14, "TxtTanggal");

NexTouch *nex_listen_list[] = {
  &BtnStart,
  &BtnStop,
  &BtnScan,
  NULL
};

// States
enum State {
  IDLE,
  CONNECTING,
  TRANSMITTING,
  DISCONNECTED
};
State currentState = IDLE;

void setup() {
  Serial.begin(115200);

  Serial1.begin(9600, SERIAL_8N1, RX1, TX1); //ini untuk ke VHMS langsung
  Serial2.begin(9600, SERIAL_8N1, RX2, TX2); //ini untuk ke Nextion
  nexInit();

  printerSerial.begin(9600);
  printer.begin();
  printer.sleep();

  BtnStart.attachPop(BtnStartPopCallback, &BtnStart);
  BtnStop.attachPop(BtnStopPopCallback, &BtnStop);
  BtnScan.attachPop(BtnScanPopCallback, &BtnScan);

  TxtStatus.setText("System ready. Press SCAN.");
  Serial.println("System ready.");
  // Cetak pesan pertama
  Messages::printMessage();
}

void loop() {
  nexLoop(nex_listen_list);

  switch (currentState) {
    case IDLE:
      TxtStatus.setText("IDLE");
      break;

    case CONNECTING: {
      TxtStatus.setText("CONNECTING");
      Serial.printf("Connecting to WiFi: %s\n", selectedSSID);
      WiFi.begin(selectedSSID, password);

      unsigned long wifiTimeout = millis();
      while (WiFi.status() != WL_CONNECTED && millis() - wifiTimeout < 10000) {
        delay(1000);
        TxtSSID.setText("Connecting...");
      }

      if (WiFi.status() == WL_CONNECTED) {
    TxtSSID.setText(WiFi.SSID().c_str());
    Serial.printf("Connected to WiFi: %s\n", WiFi.SSID().c_str());

    // Sambungkan WiFiClient ke server
    if (!client.connect("192.168.4.1", 80)) {
        Serial.println("Failed to connect to server.");
        TxtStatus.setText("Failed to connect to server.");
        currentState = DISCONNECTED;
        return;
       }
        Serial.println("WiFiClient connected to server.");
        TxtStatus.setText("TRANSMITTING");
        startTime = millis();
        shouldSendData = true;
        currentState = TRANSMITTING;
        } else {
        TxtSSID.setText("WiFi connection failed.");
        currentState = IDLE;
        TxtStatus.setText("IDLE");
       }

      break;
    }

    case TRANSMITTING:
        TxtStatus.setText("TRANSMITTING");

        if (!client.connected()) {
        Serial.println("Lost connection to server.");
        TxtStatus.setText("DISCONNECTED");
        currentState = DISCONNECTED;
        break;
        }

        // Proses data dari Serial1 (sesuai kode yang ada)
        while (Serial1.available()) {
          getVHMS();
        }
      break;

    case DISCONNECTED:
      TxtSSID.setText("DISCONNECTED");
      stopConnection();
      break;
  }

  delay(1000);
}

// Button callbacks
void BtnScanPopCallback(void *ptr) {
  Serial.println("BtnScanPopCallback");

  TxtStatus.setText("Scanning for SSIDs...");
  int n = WiFi.scanNetworks();
  if (n == 0) {
    TxtStatus.setText("No networks found.");
    return;
  }

  String SSIDs = "";
  for (int i = 0; i < n; ++i) {
    if (i > 0) SSIDs += "\r\n";
    SSIDs += WiFi.SSID(i);
    Serial.println(WiFi.SSID(i));  // Debugging
  }

  String cmdTxt = String("CmbSSID.txt=\"") + String(n) + " Networks\"";
  sendCommand(cmdTxt.c_str());

  String cmdPath = String("CmbSSID.path=\"") + SSIDs + "\"";
  sendCommand(cmdPath.c_str());

  if (!recvRetCommandFinished()) {
    Serial.println("Error updating ComboBox.");
    TxtStatus.setText("Error updating ComboBox.");
    return;
  }

  TxtStatus.setText("Scan complete. Select SSID.");
}

void BtnStartPopCallback(void *ptr) {
  Serial.println("BtnStartPopCallback");
  CmbSSID.getSelectedText(selectedSSID, sizeof(selectedSSID));

  if (strcmp(selectedSSID, "Select SSID") == 0 || strlen(selectedSSID) == 0) {
    TxtStatus.setText("Select a valid SSID.");
    return;
  }

  Serial.printf("Selected SSID: %s\n", selectedSSID);
  TxtSSID.setText("Connecting to WiFi...");
  currentState = CONNECTING;
  TxtStatus.setText("CONNECTING");
}

void BtnStopPopCallback(void *ptr) {
  Serial.println("BtnStopPopCallback");
  stopConnection();
  // Hapus atau ganti dengan kode untuk menampilkan buffer
  printLast10Data();
}

void reconnect() {
  TxtStatus.setText("Reconnecting to server...");
  Serial.println("Attempting to reconnect to server...");

  int retries = 0;
  const int maxRetries = 5;
  while (!client.connect("192.168.4.1", 80)) {
    retries++;
    Serial.printf("Reconnect attempt %d/%d\n", retries, maxRetries);
    TxtStatus.setText("Reconnecting...");
    delay(1000);

    if (retries >= maxRetries) {
      TxtStatus.setText("Reconnect failed.");
      currentState = DISCONNECTED;
      return;
    }
  }
  currentState = TRANSMITTING;
}

void stopConnection() {
  TxtSSID.setText("Stopping connection...");
  shouldSendData = false;

  if (client.connected()) {
    client.stop();
    TxtSSID.setText("Disconnected from server.");
  }

  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
    TxtSSID.setText("Disconnected from WiFi.");
  }

  delay(500);
  currentState = IDLE;
  TxtStatus.setText("IDLE");
}

// Fungsi tambahan untuk menampilkan data terakhir dari buffer (jika diinginkan)
void printLast10Data() {
    char timeBuffer[20];
    char clientName[50];
    char dateBuffer[20];
    memset(timeBuffer, 0, sizeof(timeBuffer));
    memset(clientName, 0, sizeof(clientName));
    memset(dateBuffer, 0, sizeof(dateBuffer));

    // Ambil nilai nRit sekali
    uint32_t ritValue = 0;
    if (nRit.getValue(&ritValue)) {
        Serial.printf("nRit: %d\n", ritValue);
    } else {
        Serial.println("Gagal mendapatkan nilai nRit.");
        ritValue = 0; // Nilai default jika gagal
    }

    // Ambil nilai TxtClient sekali
    if (TxtClient.getText(clientName, sizeof(clientName))) {
        cleanString(clientName); // Bersihkan string
        Serial.printf("Client Name: %s\n", clientName);
    } else {
        strcpy(clientName, "Unknown Client"); // Nilai default jika gagal
        Serial.println("Gagal mendapatkan nama client.");
    }

    // Ambil nilai TxtTanggal sekali
    if (TxtTanggal.getText(dateBuffer, sizeof(dateBuffer))) {
        cleanString(dateBuffer); // Bersihkan string
        Serial.printf("Tanggal: %s\n", dateBuffer);
    } else {
        strcpy(dateBuffer, "01-01-2025"); // Nilai default jika gagal
        Serial.println("Gagal mendapatkan tanggal.");
    }

    // Mulai mencetak ke printer
    printer.justify('C');                // Teks rata tengah
    printer.setSize('S');                // Ukuran teks medium
    // Cetak nama client dan tanggal
    printer.printf("Client: %s\n", clientName);
    printer.printf("Tanggal: %s\n", dateBuffer);
        // Cetak garis pemisah
    for (int i = 0; i < 30; i++) {
        printer.print('*'); // Cetak '-' satu per satu
    }
    printer.println();

    printer.justify('L');                // Teks rata kiri
    printer.setSize('S');                // Ukuran teks kecil

    // Header tabel
    printer.println("TIME     RIT     PAYLOAD");
        // Cetak garis pemisah
    for (int i = 0; i < 30; i++) {
        printer.print('*'); // Cetak '-' satu per satu
    }
    printer.println();

    // Cetak data dari buffer
    for (int i = 0; i < BUFFER_SIZE; i++) {
        if (!payloadBuffer[i].isEmpty()) {
            // Ambil nilai TxtJam setiap baris baru
            if (TxtJam.getText(timeBuffer, sizeof(timeBuffer))) {
                cleanString(timeBuffer); // Bersihkan dari karakter tidak valid
            } else {
                strcpy(timeBuffer, "00:00:00"); // Nilai default jika gagal
            }

            // Cetak data
            printer.printf("%s    %d       %s\n", timeBuffer, ritValue, payloadBuffer[i].c_str());
            Serial.printf("Buffer[%d]: TIME=%s RIT=%d PAYLOAD=%s\n", i, timeBuffer, ritValue, payloadBuffer[i].c_str());
            delay(100); // Tambahkan delay jika diperlukan
        }
    }

        // Cetak garis pemisah
    for (int i = 0; i < 30; i++) {
        printer.print('*'); // Cetak '-' satu per satu
    }
    printer.println();
    printer.setSize('L');
    printer.println(""); // Tambahkan baris kosong
    printer.sleep(); // Matikan printer setelah mencetak
    Serial.println("Pencetakan selesai.");
}

void cleanString(char *str) {
    int len = strlen(str);
    for (int i = 0; i < len; i++) {
        if (str[i] < 32 || str[i] > 126) { // ASCII dapat dicetak
            str[i] = '\0';
            break;
        }
    }
}


//fungsi untuk pengambilan data dari VHMS


void data_V(const uint8_t* buffer, size_t len, bool chkOk) {
  dumpLine(buffer, len, chkOk);
  if (len == 20) {  // Panjang data yang valid adalah 20 byte
    String dataString = "";

    // Membuat array untuk menyimpan setiap part data dan tambahan informasi lainnya
    String parts[9];

    // Menambahkan Front-Left suspension pressure
    parts[0] = getPressureValue(buffer, 3);
    dataString += parts[0];
    dataString += "-";

    // Menambahkan Front-Right suspension pressure
    parts[1] = getPressureValue(buffer, 5);
    dataString += parts[1];
    dataString += "-";

    // Menambahkan Rear-Left suspension pressure
    parts[2] = getPressureValue(buffer, 7);
    dataString += parts[2];
    dataString += "-";

    // Menambahkan Rear-Right suspension pressure
    parts[3] = getPressureValue(buffer, 9);
    dataString += parts[3];
    dataString += "-";

    // Menambahkan Payload
    parts[4] = getPayloadValue(buffer, 13);
    dataString += parts[4];
    dataString += "-";

    String payload =parts[4]; //variable untuk menyimpan payload ke buffer
    // Simpan payload di buffer
    if (bufferIndex < BUFFER_SIZE) {
      payloadBuffer[bufferIndex++] = payload;
      } else {
        // Geser buffer jika penuh
        for (int i = 1; i < BUFFER_SIZE; i++) {
        payloadBuffer[i - 1] = payloadBuffer[i];
      }
      payloadBuffer[BUFFER_SIZE - 1] = payload;
    }


    // **Tambahkan namaclient, tanggal, time, dan rit ke data**
    char clientBuffer[20];
    char dateBuffer[20];
    char timeBuffer[20];
    char ritBuffer[10];
    memset(clientBuffer, 0, sizeof(clientBuffer));
    memset(dateBuffer, 0, sizeof(dateBuffer));
    memset(timeBuffer, 0, sizeof(timeBuffer));
    memset(ritBuffer, 0, sizeof(ritBuffer));

    // Ambil Client (TxtClient)
    if (TxtClient.getText(clientBuffer, sizeof(clientBuffer))) {
        cleanString(clientBuffer);  // Bersihkan string dari karakter tidak valid
    } else {
        strcpy(clientBuffer, "HD78101KM");  // Default jika gagal
    }

    // Ambil tanggal (TxtTanggal)
    if (TxtTanggal.getText(dateBuffer, sizeof(dateBuffer))) {
    cleanString(dateBuffer);  // Bersihkan string dari karakter tidak valid
    
    // Gantilah tanda "-" dengan "/"
    for (int i = 0; dateBuffer[i] != '\0'; i++) {
    if (dateBuffer[i] == '-') {
        dateBuffer[i] = '/';
        }
      }
    } else {
      strcpy(dateBuffer, "01/01/2025");  // Default jika gagal
    }

    // Ambil waktu (TxtJam)
    if (TxtJam.getText(timeBuffer, sizeof(timeBuffer))) {
        cleanString(timeBuffer);  // Bersihkan string dari karakter tidak valid
    } else {
        strcpy(timeBuffer, "00:00:00");  // Default jika gagal
    }

    // Ambil rit (nRit)
    uint32_t ritValue = 0;
    if (!nRit.getValue(&ritValue)) {
        ritValue = 0;  // Default jika gagal
    }
    sprintf(ritBuffer, "%d", ritValue);  // Konversi rit menjadi string

    // Menambahkan Identitas Unit (Dumptruck)
    parts[5] = String(clientBuffer);
    dataString += parts[5];
    dataString += "-";

    // Menambahkan tanggal
    parts[6] = String(dateBuffer);
    dataString += parts[6];
    dataString += "-";

    // Menambahkan Jam
    parts[7] = String(timeBuffer);
    dataString += parts[7];
    dataString += "-";

    // Menambahkan Rit
    parts[8] = String(ritBuffer);
    dataString += parts[8];

      // Langsung kirim data ke server
    if (client.connected()) {
        Serial.printf("Sending data to server: %s\n", dataString.c_str());
        if (client.println(dataString)) {
            TxtKirim.setText(dataString.c_str());  // Tampilkan dataString yang dikirim
        } else {
            Serial.println("Send failed");
            TxtKirim.setText("Send failed");
            reconnect();
        }
    } else {
        Serial.println("Server not connected.");
        TxtKirim.setText("Server not connected.");
    }
  }
}

// Fungsi untuk mendapatkan nilai tekanan suspensi sebagai angka saja
String getPressureValue(const uint8_t* buffer, size_t pos) {
  uint16_t susP = buffer[pos] + (buffer[pos + 1] << 8);
  float pressureMPa = (susP * 0.1) * 0.0980665;  // Konversi ke MPa
  return String(pressureMPa, 2); // Format dengan 2 digit desimal
}

// Fungsi untuk mendapatkan nilai payload sebagai angka saja
String getPayloadValue(const uint8_t* buffer, size_t pos) {
  uint16_t pay = buffer[pos] + (buffer[pos + 1] << 8);
  float payloadValue = pay * 0.1; // Konversi payload
  return String(payloadValue, 1); // Format dengan 1 digit desimal
}

void dumpLine(const uint8_t* buffer, size_t len, bool chkOk) {
  Serial.println();
  if (len < 10) {
    Serial.write(' ');
  }
}

enum RxState {
  waitForSTX,
  collectUntilETX,
  waitForCRC,
};

void getVHMS() {
  static RxState rxs = waitForSTX;
  static bool nextCharLiteral = false;
  static uint8_t chkSum;
  static uint8_t bIndex;
  static uint8_t buffer[maxMsgLen + 1];
  if (Serial1.available()) {
    uint8_t inChar = Serial1.read();
    if (nextCharLiteral) {
      chkSum ^= inChar;
      buffer[bIndex++] = inChar;
      nextCharLiteral = false;
      return;
    } else if (inChar == ESCIT) {
      chkSum ^= inChar;
      nextCharLiteral = true;
      return;
    }
    switch (rxs) {
      case waitForSTX:
        if (inChar == STX) {
          bIndex = 0;
          chkSum = inChar;
          buffer[bIndex++] = inChar;
          rxs = collectUntilETX;
        }
        break;
      case collectUntilETX:
        chkSum ^= inChar;
        buffer[bIndex++] = inChar;
        if (inChar == ETX) {
          rxs = waitForCRC;
        }
        break;
      case waitForCRC:
        buffer[bIndex++] = inChar;

        data_V((const uint8_t*)(buffer + 1), bIndex - 3, inChar == chkSum);
        bIndex = 0;
        rxs = waitForSTX;
        break;
    }
  }
}
