
#include <Arduino.h>
#include <SPI.h>

#include <array>
#include <hal/hal_io.h>
#include <hal/print_debug.h>
#include <keyhandler.h>
#include <lmic.h>

#include "STM32LowPower.h"
#include "lorakeys.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

SFE_UBLOX_GNSS gnss;
// GPS_PPS_PIN ?
HardwareSerial GpsSerial(GPS_UART);

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
constexpr uint32_t TX_INTERVAL = 140;

constexpr unsigned int BAUDRATE = 115200;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = RADIO_NSS,
    .prepare_antenna_tx =
        [](bool isTx) {
          if (isTx) {
            digitalWrite(RADIO_RF_CTX_PA, HIGH);
            digitalWrite(RADIO_RF_CBT_HF, LOW);
            digitalWrite(RADIO_RF_CRX_RX, LOW);
          } else {
            digitalWrite(RADIO_RF_CTX_PA, LOW);
            digitalWrite(RADIO_RF_CBT_HF, LOW);
            digitalWrite(RADIO_RF_CRX_RX, HIGH);
          }
        }, // = LMIC_UNUSED_PIN, //
    .rst = RADIO_RESET,
    .dio = {RADIO_DIO_0, RADIO_DIO_1},
};

RadioSx1276 radio{lmic_pins};
LmicEu868 LMIC{radio};

uint32_t nextSendEpoch;

STM32RTC &rtc = STM32RTC::getInstance();

void onEvent(EventType ev) {
  switch (ev) {
  case EventType::JOINING:
    PRINT_DEBUG(2, F("EV_JOINING"));
    //        LMIC.setDrJoin(0);
    break;
  case EventType::JOINED:
    PRINT_DEBUG(2, F("EV_JOINED"));
    // disable ADR because it will be mobile.
    LMIC.setLinkCheckMode(false);
    LMIC.setDutyRate(12);
    break;
  case EventType::TXCOMPLETE:
    PRINT_DEBUG(2, F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.getTxRxFlags().test(TxRxStatus::ACK)) {
      PRINT_DEBUG(1, F("Received ack"));
    }
    if (LMIC.getDataLen()) {
      PRINT_DEBUG(1, F("Received %d bytes of payload"), LMIC.getDataLen());
      auto data = LMIC.getData();
      if (data) {
        uint8_t port = LMIC.getPort();
      }
    }
    break;
  default:
    PRINT_DEBUG(2, F("Other"));
    break;
  }
}

constexpr uint32_t adcRefVoltage = 330; // cV
constexpr uint32_t maxBatt = 415;       // cV
constexpr uint32_t minBatt = 320;       // cV
constexpr uint32_t shutdownBatt = 310;  // cV

constexpr uint32_t maxAdc = (1 << 10) - 1;

void set_battery_level() {

  uint32_t val = adcRefVoltage * analogRead(PA2) / maxAdc * 5 / 3;

  PRINT_DEBUG(1, F("batt value %d0mv"), val);

  uint8_t batlevel = 255;

  if (val > maxBatt) {
    batlevel = 254;
  } else if (val > minBatt) {
    batlevel = 254 * (val - minBatt) / (maxBatt - minBatt);
  } else if (val > shutdownBatt) {
    batlevel = 1;
  }

  LMIC.setBatteryLevel(batlevel);
}

void emptyGpsBuffer() {
  while (GpsSerial.available()) {
    GpsSerial.read();
  }
}

int32_t old_latitude = 0;
int32_t old_longitude = 0;

void do_send(OsDeltaTime to_wait) {
  if (LMIC.getTxRxFlags().test(TxRxStatus::NEED_BATTERY_LEVEL)) {
    set_battery_level();
  }
  digitalWrite(LED1, LOW);
  delay(10);
  digitalWrite(LED1, HIGH);

  uint16_t ms_to_wait =
      to_wait > OsDeltaTime::from_sec(20) ? 20000 : to_wait.to_ms() / 2;
  if (!gnss.getPVT(ms_to_wait)) {
    PRINT_DEBUG(1, F("Fail to wait, empty serial buffer"));
    emptyGpsBuffer();
    return;
  }

  auto fixtype = gnss.getFixType();
  int32_t latitude = gnss.getLatitude();
  int32_t longitude = gnss.getLongitude();
  auto altitude = gnss.getAltitude();
  auto hdop = gnss.getHorizontalDOP();
  auto siv = gnss.getSIV();
  PRINT_DEBUG(1, F("Fix %d Coordonnee %d,%d alt %d hdops:%d, siv: %d"), fixtype,
              (int)latitude, (int)longitude, altitude, (int)hdop, (int)siv);
  if (fixtype < 3 && hdop > 200) {
    PRINT_DEBUG(1, F("GPS fix not good enought"));
    nextSendEpoch = rtc.getEpoch() + 5;
    return;
  }

  if (std::abs(latitude - old_latitude) < 1000 &&
      std::abs(longitude - old_longitude) < 1000) {
    PRINT_DEBUG(1, F("GPS position not changed"));
    nextSendEpoch = rtc.getEpoch() + TX_INTERVAL;
    return;
  }

  old_latitude = latitude;
  old_longitude = longitude;
  constexpr int64_t coorfactor = 10000000;
  int64_t latitudeBinary =
      (static_cast<int64_t>(latitude) + (90LL * coorfactor)) * 16777215LL /
      180LL / coorfactor;
  int64_t longitudeBinary =
      (static_cast<int64_t>(longitude) + (180LL * coorfactor)) * 16777215LL /
      360LL / coorfactor;

  std::array<uint8_t, 9> txBuffer;
  txBuffer[0] = (latitudeBinary >> 16) & 0xFF;
  txBuffer[1] = (latitudeBinary >> 8) & 0xFF;
  txBuffer[2] = latitudeBinary & 0xFF;

  txBuffer[3] = (longitudeBinary >> 16) & 0xFF;
  txBuffer[4] = (longitudeBinary >> 8) & 0xFF;
  txBuffer[5] = longitudeBinary & 0xFF;

  txBuffer[6] = (altitude / 1000 >> 8) & 0xFF;
  txBuffer[7] = altitude / 1000 & 0xFF;
  txBuffer[8] = (hdop / 10) & 0xFF;

  // Prepare upstream data transmission at the next possible time.
  LMIC.setTxData2(2, txBuffer.begin(), txBuffer.size(), false);
  PRINT_DEBUG(1, F("Packet queued"));
  nextSendEpoch = rtc.getEpoch() + TX_INTERVAL;
}

void setup() {
  LowPower.begin();
  rtc.begin(true);

  if (debugLevel > 0) {
    Serial.begin(BAUDRATE);
  }
  PRINT_DEBUG(1, F("Starting"));

  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, HIGH);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, HIGH);

  delay(100);

  pinMode(GPS_POWER_ON_PIN, OUTPUT);
  // just to be sure
  pinMode(GPS_PPS_PIN, INPUT);
  PRINT_DEBUG(1, F("GNSS starting"));

  digitalWrite(GPS_POWER_ON_PIN, LOW);
  delay(100);
  digitalWrite(GPS_POWER_ON_PIN, HIGH);

  GpsSerial.begin(38400);
  // gnss.enableDebugging(Serial, false);

  while (!gnss.begin(GpsSerial)) {
    PRINT_DEBUG(1, F("GNSS retry"));
    GpsSerial.begin(9600);
    gnss.begin(GpsSerial);
    // gnss.hardReset();
    delay(100);
    gnss.setSerialRate(38400);
    GpsSerial.begin(38400);
  }
  UBX_CFG_TP5_data_t conf;
  conf.tpIdx = 0;
  conf.flags.bits.active = 0;
  gnss.setTimePulseParameters(&conf);
  gnss.setUART1Output(COM_TYPE_UBX);
  gnss.saveConfiguration();
  // gnss.hardReset();
  PRINT_DEBUG(1, F("GNSS setup done"));

  // Enable RF switch Pin control
  pinMode(RADIO_RF_CTX_PA, OUTPUT);
  pinMode(RADIO_RF_CBT_HF, OUTPUT);
  pinMode(RADIO_RF_CRX_RX, OUTPUT);

  // enable XTAL for RF
  pinMode(RADIO_XTAL_EN, OUTPUT);
  digitalWrite(RADIO_XTAL_EN, 1);

  SPI.begin();
  // LMIC init
  os_init();
  LMIC.init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC.reset();

  LMIC.setEventCallBack(onEvent);
  SetupLmicKey<appEui, devEui, appKey>::setup(LMIC);
  // set clock error to allow good connection.
  LMIC.setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // reduce power
  // LMIC.setAntennaPowerAdjustment(-14);

  // first send
  nextSendEpoch = rtc.getEpoch();
}

void goToSleep(uint32_t nb_sec_to_sleep) {
  if (nb_sec_to_sleep < 20) {
    return;
  }

  PRINT_DEBUG(1, F("Sleep %ds"), nb_sec_to_sleep);
  gnss.powerSaveMode(true);
  // Try to power down GPS and wake it up 10s before MCU wakeup
  gnss.powerOff((nb_sec_to_sleep - 10) * 1000);
  delay(2000);

  auto const start_sleep_time = rtc.getEpoch();
  LowPower.deepSleep((nb_sec_to_sleep - 2) * 1000);
  auto const end_sleep_time = rtc.getEpoch();
  hal_add_time_in_sleep(
      OsDeltaTime::from_sec(end_sleep_time - start_sleep_time));

  gnss.powerSaveMode(false);
}

void goToSleep(OsDeltaTime time_to_sleep) { goToSleep(time_to_sleep.to_s()); }

void loop() {
  OsDeltaTime to_wait = LMIC.run();
  // OsDeltaTime to_wait = OsInfiniteDeltaTime;

  if (to_wait < OsDeltaTime::from_ms(500)) {
    // do not try to do something if we have less than 500ms
    return;
  }

  // we are in send mode
  if (LMIC.getOpMode().test(OpState::TXRXPEND)) {
    return;
  }

  if (LMIC.getOpMode().test(OpState::TXDATA)) {
    // refresh data
    do_send(to_wait);
    return;
  }

  int32_t timebeforesend = nextSendEpoch - rtc.getEpoch();
  if (timebeforesend < 0) {
    do_send(to_wait);
  } else {
    goToSleep(std::min(timebeforesend, to_wait.to_s()));
  }
}