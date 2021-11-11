#ifndef certificateprotocol_h
#define certificateprotocol_h

#include "lmic.h"

constexpr uint8_t certificationProtocolPort = 224;
constexpr uint8_t certificationPackageID = 6;
constexpr uint8_t certificationProtocolVersion = 1;

using VersionDetails = std::array<uint8_t const, 4>;

constexpr VersionDetails LORA_1_0_4 = {1, 0, 4, 0};
constexpr VersionDetails LORA_1_0_3 = {1, 0, 3, 0};

class CertificationProtocol {
  enum class Request : uint8_t {
    PackageVersion = 0x00,
    DutReset = 0x01,
    DutJoin = 0x02,
    SwitchClass = 0x03,
    AdrBitChange = 0x04,
    RegionalDutyCycleCtrl = 0x05,
    TxPeriodicityChange = 0x06,
    TxFramesCtrl = 0x07,
    EchoPayload = 0x08,
    RxAppCnt = 0x09,
    RxAppCntReset = 0x0A,
    LinkCheck = 0x20,
    DeviceTime = 0x21,
    PingSlotInfo = 0x22,
    TxCw = 0x7D,
    DutFPort224Disable = 0x7E,
    DutVersions = 0x7F,

  };

  enum class Response : uint8_t {
    PackageVersion = 0x00,
    EchoPayload = 0x08,
    RxAppCnt = 0x09,
    DutVersions = 0x7F,
  };

private:
  Lmic &lmic;

  bool nextFrameIsConfirmed = false;
  uint16_t rxAppCnt = 0;
  bool enableFPort224 = true;
  OsDeltaTime periodicity = OsDeltaTime(0);
  VersionDetails fwVersion = {0, 0, 0, 0};
  VersionDetails lrwanVersion = LORA_1_0_3;
  VersionDetails lrwanRpVersion = {0, 0, 0, 0};

  void handlePort224Message(Request req, const uint8_t *data, uint8_t size);

  void sendPackageVersion();
  void dutReset();
  void dutJoinReq();
  void switchClass(uint8_t const *message, uint8_t size);
  void adrBitChange(uint8_t const *message, uint8_t size);
  void regionalDutyCycleCtrl(uint8_t const *message, uint8_t size);
  void txPeriodicityChange(uint8_t const *message, uint8_t size);
  void txFramesCtrl(uint8_t const *message, uint8_t size);
  void echoPayload(uint8_t const *message, uint8_t size);
  void sendRxAppCnt();
  void rxAppCntReset();
  void sendLinkCheck();
  void sendDeviceTimeReques();
  void transmitContinuousWave(uint8_t const *message, uint8_t size);
  void disableFPort224();
  void sendDutVersions();

  void resetState();

public:
  CertificationProtocol(Lmic &almic, VersionDetails const &fimwareVersion,
                        VersionDetails const &lorawanVersion,
                        VersionDetails const &lorawanRpVersion);
  bool handle(EventType ev);

  bool isEnabled() const { return enableFPort224; };
  bool isNextFrameConfirmed() const { return nextFrameIsConfirmed; }
  OsDeltaTime getPeriodicity() const { return periodicity; }
};

#endif
