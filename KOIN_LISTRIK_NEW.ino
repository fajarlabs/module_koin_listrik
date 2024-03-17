#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <PZEM004Tv30.h>
#include "EEvar.h"
#include <TimeOut.h>
#include <avr/wdt.h>
#include <MemoryFree.h>

#define TITLE "Easy EV-Charger"
#define FW_VERSION "v.1.7"
#define EEADDR 166            // Jangan di ubah
#define RELAY_PIN 6           // Main Pin Relay
#define RELAY_PIN_SUPPORT 7   // Support Pin Relay
#define RESET_PIN 4           // reset ZERO
#define BUZZER_PIN A3         // Buzzer Analog
#define INTERRUPT_COIN_PIN 2  // Pin ini dipakai untuk Coin PIN
#define MAX_COUNTER 2         // Ini adalah setting ketika menahan tombol perlu 2 detik baru aktif
#define LCD_TYPE_COLS 16      // jenis LCD 16x2
#define LCD_TYPE_ROWS 2       // jenis LCD 16x2

#define PERKWH 27000            // Setting KWH 1820
#define NORMAL_CURRENT 3        // batas normal arus yang bisa dilewati jangan lebih besar daripada limit current
#define LIMIT_CURRENT 4         // otomatis cutoff jika lebih atau sama dengan 16 Ampere
#define LIMIT_MINIMUM_KWH 0.01  // batas toleransi bawah agar cutoff
//#define RESTART_TIME 36000000  // every 10 hours

#if !defined(PZEM_RX_PIN) && !defined(PZEM_TX_PIN)
#define PZEM_RX_PIN 9
#define PZEM_TX_PIN 10
#endif

typedef struct
{
  float harga_1_watt;
  float dapatkan_kredit_watt;
  float sisa_kredit_daya_dalam_detik;
  float harga_perkwh;
  long uang;
} StoreData;

typedef struct
{
  float voltage;
  float current;
  float power;
  float energy;
  float frequency;
  float pf;
} HistoryData;

void incomingImpuls();
void callback0();
StoreData konversiUangKeSisaDaya(long uang, float harga_per_kwh);
float sisaDayaKeWatt(float sisa_daya);
float hitungEstimasiHabis(float sisa_kredit_daya_dalam_detik, float beban_daya);
void showVars(StoreData *p);
bool wait(unsigned long duration);
bool waitBuzz(unsigned long duration);
bool waitHold(unsigned long duration);
void tulisLCD(int col, int row, String text);
void bersihkanLCD(int row, int total_cols);
void simpanRekaman(HistoryData hd, int eeaddr);
HistoryData dapatkanRekaman(int eeaddr);
void showPowerVars(float voltage, float current, float power, float energy, float frequency, float pf);
void initLCD();
void wLCD(int max_char, int cols, int rows, String message);
void blinkBuzz(bool set_on, bool set_backlight);
//void (*resetFunc)(void) = 0;
void _de(long t_stamp);
void _dWrite(uint8_t pin, uint8_t state);
void _pMode(uint8_t pin, uint8_t mode);
int _dRead(uint8_t pin);
void calculate();
void showDisplay(float voltage, float current, float power, String label_kwh = "");

EEstore<float> eeSisaKreditDayaDetik(0.0);
SoftwareSerial pzemSWSerial(PZEM_RX_PIN, PZEM_TX_PIN);
PZEM004Tv30 pzem(pzemSWSerial);
HistoryData hd;

// timeout 0 (check coin)
TimeOut timeout0;
int impulsCount = 0;

LiquidCrystal_I2C lcd(0x27, 20, 4);

// lock interrupt
boolean insert = false;

// Init Var Power Sensor
float voltage = 0;
float current = 0;
float power = 0;
float energy = 0;
float frequency = 0;
float pf = 0;

int holdReset = MAX_COUNTER;  // Berapa lama tombol di tekan
bool isBuzzer = false;
bool isStarted = false;
bool isReady = false;

int stage_overload = 0;
bool is_shutdown = false;

void setup() {
  wdt_disable();        // disable wdt
  delay(2000);          // tunggu hingga stabil
  wdt_enable(WDTO_8S);  // aktifkan watchdog timer pada rentang 8 detik
  Serial.begin(9600);
  initLCD();
  _pMode(INTERRUPT_COIN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(2), incomingImpuls, RISING);
  wLCD(LCD_TYPE_COLS, 0, 0, TITLE);
  wLCD(LCD_TYPE_COLS, 0, 1, FW_VERSION);
  _de(1000);

  // relay pin relay
  _pMode(RELAY_PIN, OUTPUT);
  _pMode(RELAY_PIN_SUPPORT, OUTPUT);
  // buzzer pin
  _pMode(BUZZER_PIN, OUTPUT);
  // coin acceptor interrupt
  _pMode(RESET_PIN, INPUT_PULLUP);
  _de(500);

  // set relay to low
  _dWrite(RELAY_PIN, LOW);
  _dWrite(RELAY_PIN_SUPPORT, LOW);
  _de(500);

  isStarted = true;
}
void loop() {
  wdt_reset();  // fungsi ini di eksekusi artinya program baik-baik saja
  TimeOut::handler();

  if (stage_overload < 2) {
    if (_dRead(RESET_PIN) == LOW) {
      if (waitHold(200)) {
        holdReset -= 1;
        if (holdReset < 1) {
          float resetSisaKredit = 0;
          eeSisaKreditDayaDetik << resetSisaKredit;
          Serial.println(F("*MSG*RESET BERHASIL"));
          holdReset = MAX_COUNTER;

          for (int i = 0; i < 3; i++) {
            wLCD(LCD_TYPE_COLS, 0, 0, "");
            wLCD(LCD_TYPE_COLS, 0, 1, "");
            _dWrite(BUZZER_PIN, LOW);
            _de(200);
            wLCD(LCD_TYPE_COLS, 0, 0, " RESET BERHASIL ");
            _dWrite(BUZZER_PIN, HIGH);
            _de(200);
            wLCD(LCD_TYPE_COLS, 0, 0, "");
            wLCD(LCD_TYPE_COLS, 0, 1, "");
            _dWrite(BUZZER_PIN, LOW);
          }
        }
      }
    } else {
      holdReset = MAX_COUNTER;
    }

    if (wait(1000)) {

      Serial.print(F("freeMemory()="));
      Serial.println(freeMemory());

      if (isReady == false) {
        wLCD(LCD_TYPE_COLS, 0, 0, "");
        wLCD(LCD_TYPE_COLS, 0, 1, "");
        wLCD(LCD_TYPE_COLS, 0, 0, "Memuat data...");
        _de(500);
      }

      // Check if the data is valid
      if (isnan(voltage)) {
        Serial.println(F("*ERR*Error reading voltage"));
      } else if (isnan(current)) {
        Serial.println(F("*ERR*Error reading current"));
      } else if (isnan(power)) {
        Serial.println(F("*ERR*Error reading power"));
      } else if (isnan(energy)) {
        Serial.println(F("*ERR*Error reading energy"));
      } else if (isnan(frequency)) {
        Serial.println(F("*ERR*Error reading frequency"));
      } else if (isnan(pf)) {
        Serial.println(F("*ERR*Error reading power factor"));
      } else {
        calculate();
      }
    }
  }

  if (waitBuzz(300)) {
    if (stage_overload >= 2) {
      blinkBuzz(true, true);
    }
  }
}

void calculate() {
  if (stage_overload < 2) {
    voltage = pzem.voltage();
    current = pzem.current();
    power = pzem.power();
    energy = pzem.energy();
    frequency = pzem.frequency();
    pf = pzem.pf();

    if (stage_overload < 2) {

      if (isReady == false) {
        isReady = true;
        wLCD(LCD_TYPE_COLS, 0, 0, "");
        wLCD(LCD_TYPE_COLS, 0, 1, "");
      }

      if (voltage > 0) {
        // debug power vars
        // showPowerVars(voltage, current, power, energy, frequency, pf);
        float sisaKreditDayaDetikTemp = 0;
        eeSisaKreditDayaDetik >> sisaKreditDayaDetikTemp;

        if (sisaKreditDayaDetikTemp >= 1) {
          float estimasi_habis = 0;
          float beban_daya = 0;
          // ambil daya yang paling sesuai
          // daya dari perhitungan P = V*I atau P = SENSOR jika tidak watt terlalu kecil
          beban_daya = power;

          // periksa jika sisa kredit daya kosong/null/error
          if (sisaKreditDayaDetikTemp < 1 || isnan(sisaKreditDayaDetikTemp)) {
            Serial.println(F("Credit is empty"));
          } else {
            estimasi_habis = hitungEstimasiHabis(sisaKreditDayaDetikTemp, beban_daya);
          }

          if (sisaKreditDayaDetikTemp > 0) {
            sisaKreditDayaDetikTemp = sisaKreditDayaDetikTemp - beban_daya;
          } else {
            sisaKreditDayaDetikTemp = 0;
          }

          /*
              Serial.print("Estimasi habis : ");
              Serial.println(estimasi_habis);
              Serial.print("Saldo daya dalam detik : ");
              Serial.print(sisaKreditDayaDetikTemp);
              Serial.println(" Watt/Second");
              Serial.print("Beban daya : ");
              Serial.print(beban_daya);
              Serial.println(" Watt");
              */

          float sisa_daya_aktual = sisaDayaKeWatt(sisaKreditDayaDetikTemp);
          /*
              Serial.print("Sisa daya aktual : ");
              Serial.print(sisa_daya_aktual);
              Serial.println(" Watt");
              */

          float convertToKWH = sisa_daya_aktual / 1000;

          if (convertToKWH > LIMIT_MINIMUM_KWH) {
            if (is_shutdown) is_shutdown = false;

            String overloadLabel;
            if (stage_overload == 1 && convertToKWH) {
              overloadLabel = String(convertToKWH) + String("KWH") + "80%";  // karater sudah penuh jadi % ini hilang
            } else {
              overloadLabel = String(convertToKWH) + String("KWH");
            }
            showDisplay(voltage, current, power, overloadLabel);

            _dWrite(RELAY_PIN, HIGH);
            _dWrite(RELAY_PIN_SUPPORT, HIGH);
            Serial.println(F("*MSG*Switch status : ON"));

            hd.voltage = voltage;
            hd.current = current;
            hd.power = power;
            hd.energy = energy;
            hd.frequency = frequency;
            hd.pf = pf;
            simpanRekaman(hd, EEADDR);
            eeSisaKreditDayaDetik << sisaKreditDayaDetikTemp;
          } else {
            _dWrite(RELAY_PIN, LOW);
            _dWrite(RELAY_PIN_SUPPORT, LOW);
            Serial.println(F("*MSG*Switch status : OFF"));
            if (!is_shutdown) {
              _dWrite(BUZZER_PIN, HIGH);
              wLCD(LCD_TYPE_COLS, 0, 0, "");
              wLCD(LCD_TYPE_COLS, 0, 1, "");
              wLCD(LCD_TYPE_COLS, 0, 0, "Pulsa anda habis");
              wLCD(LCD_TYPE_COLS, 0, 1, "Terimakasih");
              delay(2000);
              _dWrite(BUZZER_PIN, LOW);
              is_shutdown = true;
            }
            showDisplay(voltage, current, power);

            sisaKreditDayaDetikTemp = 0;
            eeSisaKreditDayaDetik << sisaKreditDayaDetikTemp;
          }
        } else {
          showDisplay(voltage, current, power);
          _dWrite(RELAY_PIN, LOW);
          _dWrite(RELAY_PIN_SUPPORT, LOW);
        }
      } else {
        wLCD(LCD_TYPE_COLS, 0, 0, "No voltage");
      }
    }
  }

  if (current < NORMAL_CURRENT) {
    stage_overload = 0;
    blinkBuzz(false, false);
  } else if (current >= NORMAL_CURRENT && current < LIMIT_CURRENT) {
    stage_overload = 1;
  } else if (current >= LIMIT_CURRENT) {
    if (stage_overload < 2) {
      wLCD(LCD_TYPE_COLS, 0, 0, "");
      wLCD(LCD_TYPE_COLS, 0, 1, "");
    }
    stage_overload = 2;
    _de(500);
    wLCD(LCD_TYPE_COLS, 0, 0, "Overload!!");
    _dWrite(RELAY_PIN, LOW);
    _dWrite(RELAY_PIN_SUPPORT, LOW);
    Serial.println(F("*MSG*Switch status : OFF"));
  }
}

void showDisplay(float voltage, float current, float power, String label_kwh = "") {
  static bool isLabelKWH = false;
  if (label_kwh == "") {
    if (isLabelKWH) {
      wLCD(LCD_TYPE_COLS, 0, 0, "");
      wLCD(LCD_TYPE_COLS, 0, 1, "");
      isLabelKWH = false;
    }
    wLCD(LCD_TYPE_COLS, 0, 0, "Masukkan koin");
    wLCD(LCD_TYPE_COLS, 0, 1, "500 atau 1000");
    //wLCD(11, 6, 1, String("0KWH"));
  } else {
    if (!isLabelKWH) {
      wLCD(LCD_TYPE_COLS, 0, 0, "");
      wLCD(LCD_TYPE_COLS, 0, 1, "");
      isLabelKWH = true;
    }
    wLCD(4, 0, 0, String(int(voltage)) + String("V"));
    wLCD(5, 5, 0, String(current) + String("A"));
    wLCD(5, 11, 0, String(int(power)) + String("W"));
    wLCD(6, 0, 1, "PULSA:");
    wLCD(11, 6, 1, label_kwh);
  }
}

void incomingImpuls() {
  impulsCount = impulsCount + 1;
  timeout0.cancel();
  timeout0.timeOut(300, callback0);  // delay, callback function
}

void callback0() {
  int coinUang = 0;
  if (impulsCount == 1) {
    Serial.println(F("*DBG*Coin 1"));
  }
  if (impulsCount == 5) {
    coinUang = 1000;
    Serial.println(F("*DBG*Coin 2"));
  }
  if (impulsCount == 10) {
    coinUang = 500;
    Serial.println(F("*DBG*Coin 3"));
  }
  Serial.println(F("*MSG*Add Coin"));
  StoreData sd = konversiUangKeSisaDaya(coinUang, PERKWH);
  float _sisaKreditDayaDetik = 0;
  eeSisaKreditDayaDetik >> _sisaKreditDayaDetik;
  float _NewWisaKreditDayaDetik = _sisaKreditDayaDetik + sd.sisa_kredit_daya_dalam_detik;
  eeSisaKreditDayaDetik << _NewWisaKreditDayaDetik;

  // float sisa_daya_aktual = sisaDayaKeWatt(sisaKreditDayaDetikTemp);
  /*
    Serial.print("Sisa daya aktual : ");
    Serial.print(sisa_daya_aktual);
    Serial.println(" Watt");
    */
  impulsCount = 0;
}

// void callback1() {
//   resetFunc();
// }

StoreData konversiUangKeSisaDaya(long uang, float harga_per_kwh) {
  StoreData obj;
  obj.harga_1_watt = harga_per_kwh / 1000;
  obj.dapatkan_kredit_watt = uang / obj.harga_1_watt;
  obj.sisa_kredit_daya_dalam_detik = obj.dapatkan_kredit_watt * 3600;
  obj.harga_perkwh = harga_per_kwh;
  obj.uang = uang;
  return obj;
}

float sisaDayaKeWatt(float sisa_daya) {
  return sisa_daya / 3600;
}

float hitungEstimasiHabis(float sisa_kredit_daya_dalam_detik, float beban_daya) {
  if (beban_daya < 1)
    return 0;
  else
    return (sisa_kredit_daya_dalam_detik / beban_daya) / 3600;
}

void showVars(StoreData *p) {
  // Serial.print("Uang dari koin acceptor : Rp.");
  // Serial.println(p->uang);
  // Serial.print("Harga per-Kwh : Rp.");
  // Serial.println(p->harga_perkwh);
  // Serial.print("Harga 1 watt per-Rupiah : Rp.");
  // Serial.println(p->harga_1_watt);
  // Serial.print("Mendapatkan Watt : ");
  // Serial.print(p->dapatkan_kredit_watt);
  // Serial.println(" watt/seconds");
  // Serial.print("Total kredit daya dalam detik selama 1 jam : ");
  // Serial.print(p->sisa_kredit_daya_dalam_detik);
  // Serial.println(" watt/seconds");
  // Serial.print("Sisa dalam watt : ");
  // Serial.print(sisaDayaKeWatt(p->sisa_kredit_daya_dalam_detik));
  // Serial.println(" watt");
}

bool wait(unsigned long duration) {
  static unsigned long startTime;
  static bool isStarted = false;

  if (isStarted == false) {
    startTime = millis();
    isStarted = true;
    return false;
  }

  if (millis() - startTime >= duration) {
    isStarted = false;
    return true;
  }
  return false;
}

bool waitBuzz(unsigned long duration) {
  static unsigned long buzzStartTime;
  static bool isBuzzStarted = false;

  if (isBuzzStarted == false) {
    buzzStartTime = millis();
    isBuzzStarted = true;
    return false;
  }

  if (millis() - buzzStartTime >= duration) {
    isBuzzStarted = false;
    return true;
  }
  return false;
}

bool waitHold(unsigned long duration) {
  static unsigned long holdStartTime;
  static bool isHoldStarted = false;

  if (isHoldStarted == false) {
    holdStartTime = millis();
    isHoldStarted = true;
    return false;
  }

  if (millis() - holdStartTime >= duration) {
    isHoldStarted = false;
    return true;
  }
  return false;
}

void tulisLCD(int col, int row, String text) {
  lcd.setCursor(col, row);
  lcd.print(text);
}

void bersihkanLCD(int row, int total_cols) {
  for (int n = 0; n < total_cols; n++) {
    lcd.setCursor(n, row);
    lcd.print(" ");
  }
  lcd.setCursor(0, row);  // set cursor in the beginning of deleted line
}

void simpanRekaman(HistoryData hd, int eeaddr) {
  EEPROM.put(eeaddr, hd);
}

HistoryData dapatkanRekaman(int eeaddr) {
  HistoryData hd;
  EEPROM.get(eeaddr, hd);
  return hd;
}

void showPowerVars(float voltage, float current, float power, float energy, float frequency, float pf) {
  /*
    Serial.print("Voltage: ");      Serial.print(voltage);      Serial.println("V");
    Serial.print("Current: ");      Serial.print(current);      Serial.println("A");
    Serial.print("Power: ");        Serial.print(power);        Serial.println("W");
    Serial.print("Energy: ");       Serial.print(energy,3);     Serial.println("kWh");
    Serial.print("Frequency: ");    Serial.print(frequency, 1); Serial.println("Hz");
    Serial.print("PF: ");           Serial.println(pf);
    */
}

void initLCD() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
}

void wLCD(int max_char, int cols, int rows, String message) {
  lcd.setCursor(cols, rows);
  if (message.length() > max_char) {
    for (int i = 0; i < max_char; i++) {
      message.concat(" ");
    }
    lcd.print("ERR");
  } else {
    int total_space = max_char - message.length();
    if (total_space > 0) {
      for (int i = 0; i < total_space; i++) {
        message.concat(" ");
      }
      lcd.print(message);
    } else {
      lcd.print(message);
    }
  }
}

void blinkBuzz(bool set_on, bool set_backlight) {
  if (set_on == true) {
    if (isBuzzer == false) {
      _dWrite(BUZZER_PIN, HIGH);
      isBuzzer = true;
      if (set_backlight == true) {
        lcd.noBacklight();
      }
    } else {
      _dWrite(BUZZER_PIN, LOW);
      isBuzzer = false;
      if (set_backlight == true) {
        lcd.backlight();
      }
    }
  } else {
    lcd.backlight();
    _dWrite(BUZZER_PIN, LOW);
  }
}

void _de(long t_stamp) {
  delay(t_stamp);
}

void _dWrite(uint8_t pin, uint8_t state) {
  digitalWrite(pin, state);
}

void _pMode(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
}

int _dRead(uint8_t pin) {
  return digitalRead(pin);
}