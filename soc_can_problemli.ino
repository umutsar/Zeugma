#include <EEPROM.h>
#include <SPI.h>
#include <mcp2515.h>
#include <math.h>

// CAN
struct can_frame Received_Data;
struct can_frame Transmitted_Data;
MCP2515 mcp2515(10);
// #


const double fullCapacity = 56000.0;  // mAh cinsinden toplam batarya kapasitesi (56 Ah)
double soc = 100.0;                   // Başlangıçta %100 SOC
const unsigned long interval = 1000;  // 1 saniyede bir ölçüm al
int potPin = A2;                      // Potansiyometre bağlı olduğu pin
int canFlag = 0;

unsigned long lastTime = 0;

void setup() {
  Serial.begin(9600);

  // EEPROM'dan başlangıç SOC'yi oku
  EEPROM.get(0, soc);

  // EEPROM'dan geçerli SOC'yi okunamıyorsa varsayılan olarak %100 yükle
  if (soc <= 0 || soc > 100) {
    soc = 100.0;
  }

  lastTime = millis();  // lastTime başlangıçta ayarla

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  Transmitted_Data.can_id = 124;
  Transmitted_Data.can_dlc = 8;
  Transmitted_Data.data[0] = 212;

  Transmitted_Data.data[4] = 0;
  Transmitted_Data.data[5] = 0;
  Transmitted_Data.data[6] = 0;
  Transmitted_Data.data[7] = 0;
}


bool canReadFunction() {
  unsigned long startTime = millis();

  while (millis() - startTime < 3000) {

    if (mcp2515.readMessage(&Received_Data) == MCP2515::ERROR_OK) {
      if (Received_Data.data[0] == 88) {
        Serial.println("VERI GELDI");
        canFlag = 1;
        Received_Data.data[0] = 0;
        return true;
      }
    }
  }

  Serial.println("Timeout waiting for response");
  return false;
}




void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastTime >= interval) {
    // Geçen süreyi saat cinsine çevir
    double elapsedHours = (currentTime - lastTime) / 3600000.0;
    lastTime = currentTime;  // lastTime'ı burada güncelle

    // Potansiyometreden akım değerini oku (0-1023 arasında bir değer)
    int potValue = analogRead(potPin);
    // Akım değeri (mA) hesapla (Örnek: 0-1023 arasında değeri 0-5000 mA aralığına ölçekleyelim)
    double current_mA = potValue * 5000.0 / 1023.0;

    // Akımı zamanla entegre et (Q = I * t)
    double deltaCapacity = current_mA * elapsedHours;

    // Bataryanın kapasitesini güncelle
    double currentCapacity = (soc / 100) * fullCapacity;
    currentCapacity -= deltaCapacity;

    // Yeni SOC'yi hesapla
    soc = (currentCapacity / fullCapacity) * 100.0;

    // Verileri ekrana yazdır
    Serial.print("Current:       ");
    Serial.print(current_mA);
    Serial.println(" mA");
    Serial.print("Remaining Capacity: ");
    Serial.print(currentCapacity);
    Serial.println(" mAh");
    Serial.print("State of Charge (SOC): ");
    Serial.print(soc);
    Serial.println(" %");
    Serial.println("");

    // SOC'yi EEPROM'a kaydet
    EEPROM.put(0, soc);

    if (canFlag == 1) {
      Transmitted_Data.data[1] = soc;

      Transmitted_Data.data[2] = (int)(currentCapacity / 256);
      Transmitted_Data.data[3] = (int)fmod(currentCapacity, 256);


      mcp2515.sendMessage(&Transmitted_Data);
      canFlag = 0;
    }
  }
}
