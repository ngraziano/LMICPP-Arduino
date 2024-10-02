
#include "certificationprotocol.h"
#include <algorithm>
#include <array>

CertificationProtocol::CertificationProtocol(
    Lmic &almic, VersionDetails const &fimwareVersion,
    VersionDetails const &lorawanVersion,
    VersionDetails const &lorawanRpVersion)
    : lmic(almic), fwVersion(fimwareVersion), lrwanVersion(lorawanVersion),
      lrwanRpVersion(lorawanRpVersion) {}

// return false if message is not handled
bool CertificationProtocol::handle(EventType ev) {
  if (!enableFPort224) {
    return false;
  }

  switch (ev) {
  case EventType::RXC:
  case EventType::TXCOMPLETE:
    if (lmic.getPort() > 0 || lmic.getTxRxFlags().test(TxRxStatus::ACK)) {
      rxAppCnt++;
    }

    if (lmic.getPort() == certificationProtocolPort) {
      auto data = lmic.getData();
      if (data && lmic.getDataLen() > 0) {
        handlePort224Message(static_cast<Request>(data[0]), data + 1,
                             lmic.getDataLen() - 1);
      }
      return true;
    }
    break;
  default:
    break;
  }
  return false;
}

void CertificationProtocol::handlePort224Message(const Request req,
                                                 uint8_t const *data,
                                                 uint8_t size) {
  switch (req) {
  case Request::PackageVersion:
    return sendPackageVersion();
  case Request::DutReset:
    return dutReset();
  case Request::DutJoin:
    return dutJoinReq();
  case Request::SwitchClass:
    return switchClass(data, size);
  case Request::AdrBitChange:
    return adrBitChange(data, size);
  case Request::RegionalDutyCycleCtrl:
    return regionalDutyCycleCtrl(data, size);
  case Request::TxPeriodicityChange:
    return txPeriodicityChange(data, size);
  case Request::TxFramesCtrl:
    return txFramesCtrl(data, size);
  case Request::EchoPayload:
    return echoPayload(data, size);
  case Request::RxAppCnt:
    return sendRxAppCnt();
  case Request::RxAppCntReset:
    return rxAppCntReset();
  case Request::LinkCheck:
    return sendLinkCheck();
  case Request::DeviceTime:
    return sendDeviceTimeReques();
  case Request::TxCw:
    return transmitContinuousWave(data, size);
  case Request::DutFPort224Disable:
    return disableFPort224();
  case Request::DutVersions:
    return sendDutVersions();

  case Request::PingSlotInfo:
  default:
    break;
  }
}

void CertificationProtocol::sendPackageVersion() {
  std::array<uint8_t, 3> data;
  data[0] = static_cast<uint8_t>(Response::PackageVersion);
  data[1] = certificationPackageID;
  data[2] = certificationProtocolVersion;
  lmic.setTxData2(certificationProtocolPort, data.begin(), data.size(),
                  nextFrameIsConfirmed);
}

void CertificationProtocol::dutReset() {
  // Instructs the DUT to execute/simulate a full DUT MCU reset.

  // We simulate a reset by resetting the LMIC state machine
  lmic.reset();
  resetState();
}

void CertificationProtocol::dutJoinReq() {
  // Instructs the DUT to reset the LoRaWAN MAC layer and to start issuing
  // Join-Request frames. The LoRaWAN MAC layer SHALL reinitialize such that all
  // RF parameters are restored to default settings and the end-device SHALL
  // then attempt to join the network as part of normal operation.
  lmic.reset();
}

void CertificationProtocol::switchClass(uint8_t const *data, uint8_t size) {
  // Instructs the DUT to switch to the specified class.

  if (size != 1) {
    return;
  }

  auto const classType = data[0];

  constexpr uint8_t classA = 0;
  constexpr uint8_t classB = 1;
  constexpr uint8_t classC = 2;

  if (classType == classA) {
    lmic.deactivateClassC();
  } else if (classType == classB) {
    lmic.deactivateClassC();
  } else if (classType == classC) {
    lmic.activateClassC();
  }
}

void CertificationProtocol::adrBitChange(uint8_t const *data, uint8_t size) {
  // The AdrBitChangeReq command sent by the TCL requests the DUT to
  // activate/deactivate the ADR feature.
  if (size != 1) {
    return;
  }
  bool const adr = data[0] == 1;
  lmic.setLinkCheckMode(adr);
}

void CertificationProtocol::regionalDutyCycleCtrl(uint8_t const *data,
                                                  uint8_t size) {
  // The RegionalDutyCycleCtrlReq command sent by the TCL requests the DUT to
  // activate/deactivate the regional duty-cycle enforcement for regions
  // requiring it.
  if (size != 1) {
    return;
  }
  bool const regionalDutyCycle = data[0] == 1;
  lmic.setRegionalDutyCycleVerification(regionalDutyCycle);
}

constexpr std::array<OsDeltaTime, 11> periodicityTable = {
    OsDeltaTime::from_sec(0),   OsDeltaTime::from_sec(5),
    OsDeltaTime::from_sec(10),  OsDeltaTime::from_sec(20),
    OsDeltaTime::from_sec(30),  OsDeltaTime::from_sec(40),
    OsDeltaTime::from_sec(50),  OsDeltaTime::from_sec(60),
    OsDeltaTime::from_sec(120), OsDeltaTime::from_sec(240),
    OsDeltaTime::from_sec(480)};

void CertificationProtocol::txPeriodicityChange(uint8_t const *data,
                                                uint8_t size) {
  // The TxPeriodicityChangeReq command sent by the TCL requests the DUT to
  // change the transmission periodicity.
  if (size != 1) {
    return;
  }
  uint8_t const periodicityIdx = data[0];
  if (periodicityIdx < periodicityTable.size()) {
    periodicity = periodicityTable[periodicityIdx];
  }
}

void CertificationProtocol::txFramesCtrl(uint8_t const *data, uint8_t size) {
  // The TxFramesCtrlReq command is used to convey the frame type to be used by
  // subsequent uplink frames. This command MAY also convey N extra octets.
  if (size < 1) {
    return;
  }
  uint8_t const frameType = data[0];
  if (frameType == 1) {
    nextFrameIsConfirmed = false;
  } else if (frameType == 2) {
    nextFrameIsConfirmed = true;
  }
}

void CertificationProtocol::echoPayload(uint8_t const *data, uint8_t size) {
  // The EchoPayloadReq command payload contains the N bytes to be echoed
  // plus one.
  // The N value is arbitrary. In case the N value is bigger than the
  // application payload buffer then the echoed packet SHALL be clipped to the
  // maximum payload buffer size.
  // For example, if the DUT receives a payload of [8 1 5 255] on FPort 224, it
  // will respond with [8 2 6 0] on the FPort 224. This echo functionality is
  // used to validate the DUT cryptography implementation as well as its
  // handling of the maximum payload for both uplinks and downlinks. The
  // EchoPayloadAns SHALL be clipped to maximum Regional Parameters allowed
  // uplink frame payload size.

  uint8_t buffer[size + 1];
  buffer[0] = static_cast<uint8_t>(Response::EchoPayload);
  for (uint8_t i = 0; i < size; i++) {
    buffer[i + 1] = data[i] + 1;
  }
  lmic.setTxData2(certificationProtocolPort, buffer, size + 1,
                  nextFrameIsConfirmed);
}

void CertificationProtocol::sendRxAppCnt() {
  // The RxAppCntReq command sent by the TCL requests the DUT to provide the
  // current RxAppCnt value. This command has no payload.

  std::array<uint8_t, 3> data;
  data[0] = static_cast<uint8_t>(Response::RxAppCnt);
  data[1] = rxAppCnt;
  data[2] = rxAppCnt >> 8;
  lmic.setTxData2(certificationProtocolPort, data.begin(), data.size(),
                  nextFrameIsConfirmed);
}

void CertificationProtocol::rxAppCntReset() {
  // The RxAppCntResetReq command sent by the TCL requests the DUT to reset the
  // RxAppCnt value to zero. This command has no payload.

  rxAppCnt = 0;
}

void CertificationProtocol::sendLinkCheck() {
  // the LinkCheckReq command has no payload.
  // Instructs the DUT to send a LinkCheckReq MAC command.

  lmic.askLinkCheck();
}

void CertificationProtocol::sendDeviceTimeReques() {
  // Instructs the DUT to send a DeviceTimeReq MAC command.
  // The DeviceTimeReq command has no payload.

  // TODO: implement device time request

  // lmic.askDeviceTime();
}

void CertificationProtocol::transmitContinuousWave(uint8_t const *message,
                                                   uint8_t size) {

  // the TxCwReq command payload is used to define the timeout, radio frequency
  // and radio transmission output power

  if (size != 6) {
    return;
  }
  /* 
  uint16_t const timeout =
      message[0] | (static_cast<uint16_t>(message[1]) << 8);
  uint32_t const frequency = message[2] |
                             (static_cast<uint32_t>(message[3]) << 8) |
                             (static_cast<uint32_t>(message[4]) << 16);
  uint8_t const power = message[5];
  */
  // TODO: implement continuous wave transmission
}

void CertificationProtocol::disableFPort224() {
  // Instructs the DUT to disable access to FPort 224 and executes a full reset
  // of the DUT.
  enableFPort224 = false;
  resetState();
  lmic.reset();
}

void CertificationProtocol::sendDutVersions() {
  // The DutVersionsReq command sent by the TCL requests the DUT to provide its
  // firmware version, Link Layer specification [TS001] version and Regional
  // Parameters specification [RP002] version. This command has no payload.

  // The versions (FwVersion, LrwanVersion and LrwanRpVersion) fields SHALL be
  // encoded as Major.Minor.Patch.Revision: 1 octet for Major, 1 octet for
  // Minor, 1 octet for Patch and 1 octet for Revision.

  std::array<uint8_t, 13> data;
  data[0] = static_cast<uint8_t>(Response::DutVersions);
  std::copy(fwVersion.begin(), fwVersion.end(), data.begin() + 1);
  std::copy(lrwanVersion.begin(), lrwanVersion.end(), data.begin() + 5);
  std::copy(lrwanRpVersion.begin(), lrwanRpVersion.end(), data.begin() + 9);

  lmic.setTxData2(certificationProtocolPort, data.begin(), data.size(),
                  nextFrameIsConfirmed);
}

void CertificationProtocol::resetState() {
  nextFrameIsConfirmed = false;
  periodicity = OsDeltaTime(0);
  rxAppCnt = 0;
}