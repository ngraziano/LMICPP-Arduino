#include <unity.h>
#include <vector>

#include "../activation_pretest.h"
#include "../dut.h"
#include "../packet_util.h"

TestServerState server_state;

constexpr OsDeltaTime defaultWaitTime = OsDeltaTime::from_sec(60);

void setUp(void) {
  dut::reset();
  sp1_intial_join(server_state);
}

// 2.4.1.a.i
void test_aes_encryption() {
  // Step 1
  // DUT sends Unconfirmed frame
  // FCntUp = n
  auto nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  auto n = server_state.fCntUp;

  // The TCL sends Unconfirmed frames
  // CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  auto nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  // FCntUp = n + 1
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT32(n + 1, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  auto payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  uint16_t rxAppCnt = (payload[2] << 8) | payload[1];

  // The TCL sends Unconfirmed frames
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08 (Various)
  // TODO use random data
  auto echoFrame = std::vector<uint8_t>{0x08, 0x01, 0x02};
  nextResponse = make_data_response(224, echoFrame, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  for (int i = 1; i < 4; i++) {
    // Step 3
    // DUT sends Unconfirmed frame
    // FCntUp = n + 1 + i
    // CP-CMD EchoPayloadAns (repeat i times- where i = 1 to 3)
    // FPort = 224
    // Payload = [0x]08 (Various)
    nextPacket = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
    TEST_ASSERT(is_data(nextPacket));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
    TEST_ASSERT_EQUAL_UINT32(n + 1 + i, server_state.fCntUp);
    TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
    payload = get_payload(nextPacket, server_state);
    TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
    TEST_ASSERT_EQUAL_UINT(echoFrame.size(), payload.size());
    for (size_t j = 1; j < payload.size(); j++) {
      TEST_ASSERT_EQUAL_UINT8(echoFrame[j] + 1, payload[j]);
    }

    // The TCL sends Unconfirmed frames
    // CP-CMD EchoPayloadReq (repeat i times- where i = 1 to 3)
    // FPort = 224
    // Payload = [0x]08 (Various)
    // This library do not handle message greater than 64 bytes in total
    // so we can't test the 255 byte message
    auto echoSize = i == 1 ? 12 : i == 2 ? 45 : 50;
    echoFrame = std::vector<uint8_t>{0x08};
    for (int j = 0; j < echoSize; j++) {
      echoFrame.push_back((j * 7) % 255);
    }
    nextResponse = make_data_response(224, echoFrame, false, server_state);
    nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);
  }

  // Step 4
  // DUT sends Unconfirmed frame
  // FCntUp = n + 5
  // CP-CMD EchoPayloadAns
  // FPort = 224
  // Payload = [0x]08 (Various)
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT32(n + 5, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x08, payload[0]);
  TEST_ASSERT_EQUAL_UINT(echoFrame.size(), payload.size());
  for (size_t j = 1; j < payload.size(); j++) {
    TEST_ASSERT_EQUAL_UINT8(echoFrame[j] + 1, payload[j]);
  }
  // The TCL sends Unconfirmed frames
  //  CP-CMD RxAppCntReq
  //  FPort = 224
  //  Payload = [0x]09
  nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextPacket.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // FCntUp = n + 6
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x + 5
  nextPacket = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextPacket, server_state));
  TEST_ASSERT(is_data(nextPacket));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextPacket));
  TEST_ASSERT_EQUAL_UINT32(n + 6, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextPacket));
  payload = get_payload(nextPacket, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto newRxAppCnt = (payload[2] << 8) | payload[1];
  TEST_ASSERT_EQUAL_UINT16(rxAppCnt + 5, newRxAppCnt);
}

// 2.4.1.a.ii
void test_message_integrity_code() {
  // Step 1
  // DUT sends Unconfirmed frame
  // FCntUp = n
  auto nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  auto n = server_state.fCntUp;

  // The TCL sends Unconfirmed frames
  // CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  auto nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  // FCntUp = n + 1
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 1, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  auto payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto rxAppCnt = (payload[2] << 8) | payload[1];

  // The TCL sends Unconfirmed frames
  // CP-CMD EchoPayloadReq
  // FPort = 224
  // Payload = [0x]08 (Various)
  // MIC Invalid
  auto echoFrame = std::vector<uint8_t>{0x08, 0x1, 0x02};
  nextResponse = make_data_response(224, echoFrame, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  // make mic invalid
  nextResponse.data[nextResponse.length - 1]++;
  dut::send_data(nextResponse);

  for (int i = 1; i < 5; i++) {
    // Step 3
    // DUT sends Unconfirmed frame
    // FCntUp = n + 1 + i
    nextMessage = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
    TEST_ASSERT(is_data(nextMessage));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
    TEST_ASSERT_EQUAL_UINT32(n + 1 + i, server_state.fCntUp);

    // The TCL sends Unconfirmed frames
    // CP-CMD EchoPayloadReq (repeat i times- where i = 1 to 4)
    // FPort = 224
    // Payload = [0x]08 (Various)
    // MIC Invalid
    // TODO increase the size of the echoFrame
    auto echoSize = i == 1 ? 12 : i == 2 ? 45 : i == 3 ? 50 : i == 4 ? 20 : 0;
    echoFrame = std::vector<uint8_t>{0x08};
    for (int j = 0; j < echoSize; j++) {
      echoFrame.push_back((j * 7) % 255);
    }
    nextResponse = make_data_response(224, echoFrame, false, server_state);
    nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
    // make mic invalid
    nextResponse.data[nextResponse.length - 1]++;
    dut::send_data(nextResponse);
  }

  // Step 4
  // DUT sends Unconfirmed frame
  // FCntUp = n + 6
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 6, server_state.fCntUp);

  // The TCL sends Unconfirmed frames
  // CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // FCntUp = n + 7
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x +1
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 7, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto newRxAppCnt = (payload[2] << 8) | payload[1];
  TEST_ASSERT_EQUAL_UINT(rxAppCnt + 1, newRxAppCnt);
}

// 2.4.1.b
void test_downling_sequence_number() {
  // DUT sends Unconfirmed frame
  // FCntUp = n
  auto nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  auto n = server_state.fCntUp;

  // The TCL sends Unconfirmed frames
  // CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  auto nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  // FCntUp = n + 1
  // CP-CMD RxAppCntAns
  // FPort 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 1, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  auto payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto rxAppCnt = (payload[2] << 8) | payload[1];

  // The TCL sends Unconfirmed frame
  // FCntDown = a
  // CP-CMD TxFramesCtrlReq
  // FPort = 224
  // Frame type = No change
  // Payload = [0x]0700
  auto const a = server_state.fCntDown;
  nextResponse = make_data_response(224, std::vector<uint8_t>{0x07, 0x00},
                                    false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  for (uint32_t i = 1; i < a; i++) {
    // Step 3
    // DUT sends Unconfirmed frame
    // FCntUp = n + 1 + y
    // Repeat y times
    nextMessage = dut::wait_for_data(defaultWaitTime);
    TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
    TEST_ASSERT(is_data(nextMessage));
    TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
    TEST_ASSERT_EQUAL_UINT32(n + 1 + i, server_state.fCntUp);

    // The TCL sends Unconfirmed frames
    // FCntDown = a – i (where i = [1, a-1]) Repeat y times
    // CP-CMD TxFramesCtrlReq
    // FPort = 224
    // Frame type = No change
    // Payload = [0x]0700
    server_state.fCntDown = a - i;
    nextResponse = make_data_response(224, std::vector<uint8_t>{0x07, 0x00},
                                      false, server_state);
    nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
    dut::send_data(nextResponse);
  }
  // Step 4
  // DUT sends Unconfirmed frame
  // FCntUp = n + 2 + y
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 2 + a - 1, server_state.fCntUp);

  // The TCL sends Unconfirmed frames  CP-CMD RxAppCntReq
  // FPort = 224
  //  Payload = [0x]09
  server_state.fCntDown = a + 1;
  nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Unconfirmed frame
  // FCntUp = n + 3 + y
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x + 2
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 3 + a - 1, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto newRxAppCnt = (payload[2] << 8) | payload[1];
  TEST_ASSERT_EQUAL_INT(rxAppCnt + 2, newRxAppCnt);
}

void test_confirmed_uplink() {
  // Step 1
  // DUT sends Unconfirmed frame
  // FCntUp = n
  auto nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  auto n = server_state.fCntUp;
  // The TCL sends Unconfirmed frames  CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  auto nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 2
  // DUT sends Unconfirmed frame
  // FCntUp = n + 1
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt = x
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_EQUAL_UINT32(n + 1, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  auto payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto rxAppCnt = (payload[2] << 8) | payload[1];

  // The TCL sends Unconfirmed frame
  // CP-CMD TxFramesCtrlReq
  // FPort 224
  // Frame Type = Confirmed
  // Payload [0x]0702
  nextResponse = make_data_response(224, std::vector<uint8_t>{0x07, 0x02},
                                    false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 3
  // DUT sends Confirmed frame
  // FCntUp >= n + 2
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_TRUE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 2, server_state.fCntUp);
  // The TCL sends Unconfirmed frame
  // Acknowledge
  // No FPort and no payload
  nextResponse = make_empty_response(true, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 4
  // DUT sends Confirmed frame
  // FCntUp >= n + 3
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_TRUE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 3, server_state.fCntUp);
  // The TCL sends Unconfirmed frame
  // Acknowledge
  // CP-CMD TxFramesCtrlReq
  // FPort = 224
  // Frame type = No change
  // Payload [0x]0700
  nextResponse = make_data_response(224, std::vector<uint8_t>{0x07, 0x00}, true,
                                    server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 5
  // DUT sends Confirmed frame
  // FCntUp >= n + 4
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_TRUE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 4, server_state.fCntUp);
  // The TCL sends Unconfirmed frame
  // Acknowledge
  // CP-CMD RxAppCntReq
  // FPort = 224
  // Payload = [0x]09
  nextResponse =
      make_data_response(224, std::vector<uint8_t>{0x09}, true, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 6
  // DUT sends Confirmed frame
  // FCntUp >= n + 5
  // CP-CMD RxAppCntAns
  // FPort = 224
  // Payload = [0x]09XXXX
  // RxAppCnt >= x + 4
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_TRUE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 5, server_state.fCntUp);
  TEST_ASSERT_EQUAL_UINT8(224, get_port(nextMessage));
  payload = get_payload(nextMessage, server_state);
  TEST_ASSERT_EQUAL_UINT8(0x09, payload[0]);
  TEST_ASSERT_EQUAL_UINT(3, payload.size());
  auto newRxAppCnt = (payload[2] << 8) | payload[1];
  TEST_ASSERT_GREATER_OR_EQUAL_UINT16(rxAppCnt + 4, newRxAppCnt);

  // Step 7
  // DUT sends Confirmed frame
  // FCntUp >= n + 6
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_TRUE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 6, server_state.fCntUp);
  // The TCL sends Unconfirmed frame
  // Acknowledge
  // CP-CMD TxFramesCtrlReq
  // FPort 224
  // Frame type = Unconfirmed
  // Payload [0x]0701
  nextResponse = make_data_response(224, std::vector<uint8_t>{0x07, 0x01},
                                    false, server_state);
  nextResponse.time = nextMessage.time + RECEIVE_DELAY2;
  dut::send_data(nextResponse);

  // Step 8
  // DUT sends Unconfirmed frame
  // FCntUp >= n + 7
  nextMessage = dut::wait_for_data(defaultWaitTime);
  TEST_ASSERT(check_is_next_packet(nextMessage, server_state));
  TEST_ASSERT(is_data(nextMessage));
  TEST_ASSERT_FALSE(is_confirmed_uplink(nextMessage));
  TEST_ASSERT_GREATER_OR_EQUAL_UINT32(n + 7, server_state.fCntUp);

}

void tearDown(void) {
  // clean stuff up here
}

void runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_aes_encryption);
  RUN_TEST(test_message_integrity_code);
  RUN_TEST(test_downling_sequence_number);
  RUN_TEST(test_confirmed_uplink);

  UNITY_END();
}

int main() { runUnityTests(); }

#ifdef ARDUINO
void setup() {
  delay(2000);
  runUnityTests();
}

void loop() {
#if defined(set_sleep_mode) && defined(sleep_mode)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode();
#endif
}
#endif