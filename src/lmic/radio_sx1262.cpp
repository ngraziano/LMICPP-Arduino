/*******************************************************************************

 *******************************************************************************/

#include "radio_sx1262.h"
#include "../hal/print_debug.h"

#include "../aes/lmic_aes.h"
#include "lmic_table.h"

#include "bufferpack.h"

#include <algorithm>

namespace {

enum RadioCommand : uint8_t {
  ResetStats = 0x00,
  ClearIrqStatus = 0x02,
  ClearDeviceErrors = 0x07,
  SetDioIrqParams = 0x08,
  WriteRegister = 0x0D,
  WriteBuffer = 0x0E,
  GetStats = 0x10,
  GetPacketType = 0x11,
  GetIrqStatus = 0x12,
  GetRxBufferStatus = 0x13,
  GetPacketStatus = 0x14,
  GetRssiInst = 0x15,
  GetDeviceErrors = 0x17,
  ReadRegister = 0x1D,
  ReadBuffer = 0x1E,
  SetStandby = 0x80,
  SetRx = 0x82,
  SetTx = 0x83,
  SetSleep = 0x84,
  SetRfFrequency = 0x86,
  SetCadParams = 0x88,
  Calibrate = 0x89,
  SetPacketType = 0x8A,
  SetModulationParams = 0x8B,
  SetPacketParams = 0x8C,
  SetTxParams = 0x8E,
  SetBufferBaseAddress = 0X8F,
  SetRxTxFallbackMode = 0x93,
  SetRxDutyCycle = 0x94,
  SetPaConfig = 0x95,
  SetRegulatorMode = 0x96,
  SetDIO3AsTcxoCtrl = 0x97,
  CalibrateImage = 0x98,
  SetDIO2AsRfSwitchCtrl = 0x9D,
  StopTimerOnPreamble = 0x9F,
  SetLoRaSymbNumTimeout = 0xA0,
  GetStatus = 0xC0,
  SetFs = 0xC1,
  SetCad = 0xC5,
  SetTxContinuousWave = 0xD1,
  SetTxInfinitePreamble = 0xD2,

};

void wait_ready(HalIo const &hal) {
  // wait for busy pin to go low
  while (hal.io_check0()) {
    yield();
  }
}

void print_status(uint8_t status) {
  PRINT_DEBUG(1, F("Status %x mode : %x, status %x"), status, (status >> 4) & 7,
              (status >> 1) & 7);
}

template <int parameter_length> struct Sx1262Command {
  RadioCommand const command;
  uint8_t parameter[parameter_length];

  uint8_t *begin() { return parameter; }
  uint8_t *end() { return parameter + parameter_length; }

  uint8_t const *begin() const { return parameter; }
  uint8_t const *end() const { return parameter + parameter_length; }
};

template <int parameter_length>
void send_command(HalIo const &hal,
                  Sx1262Command<parameter_length> const &cmd) {
  PRINT_DEBUG(2, F("Cmd> %x"), cmd.command);

  hal.beginspi();
  wait_ready(hal);
  hal.spi(cmd.command);

  std::for_each(cmd.begin(), cmd.end(),
                [&hal](uint8_t const val) { hal.spi(val); });

  hal.endspi();
}

template <int parameter_length>
void read_command(HalIo const &hal, Sx1262Command<parameter_length> &cmd) {
  PRINT_DEBUG(2, F("Cmd< %x"), cmd.command);

  hal.beginspi();
  wait_ready(hal);
  hal.spi(cmd.command);
  hal.spi(0x00);

  std::generate(cmd.begin(), cmd.end(), [&hal]() { return hal.spi(0x00); });

  hal.endspi();
}

template <int data_length> struct Sx1262Register {
  uint16_t const address;
  uint8_t data[data_length];

  uint8_t *begin() { return data; }
  uint8_t *end() { return data + data_length; }

  uint8_t const *begin() const { return data; }
  uint8_t const *end() const { return data + data_length; }
};

template <int data_length>
void read_register(HalIo const &hal, Sx1262Register<data_length> &reg) {
  PRINT_DEBUG(2, F("Reg< %x"), reg.address);

  hal.beginspi();
  wait_ready(hal);
  // Read register command
  hal.spi(RadioCommand::ReadRegister);
  // send adress
  hal.spi(reg.address >> 8);
  hal.spi(reg.address & 0xff);
  // send initial NOP
  hal.spi(0x00);

  // read data
  std::generate(reg.begin(), reg.end(), [&hal]() { return hal.spi(0x00); });

  hal.endspi();
}

template <int data_length>
void write_register(HalIo const &hal, Sx1262Register<data_length> const &reg) {
  PRINT_DEBUG(2, F("Reg> %x"), reg.address);

  hal.beginspi();
  wait_ready(hal);
  // Read register command
  hal.spi(RadioCommand::WriteRegister);
  // send adress
  hal.spi(reg.address >> 8);
  hal.spi(reg.address & 0xff);
  // Write data
  std::for_each(reg.begin(), reg.end(), [&hal](uint8_t val) { hal.spi(val); });
  hal.endspi();
}

constexpr uint8_t sf_to_parameter(sf_t sf) {
  // SF7 => 7, SF8=> 8 ...
  return (7 - SF7 + sf);
}

constexpr uint8_t bw_to_parameter(BandWidth bw) {
  // BW125 => 0x04, BW250 => 0x05 ...
  return (0x04 - static_cast<uint8_t>(BandWidth::BW125) +
          static_cast<uint8_t>(bw));
}

constexpr uint8_t cr_to_parameter(CodingRate cr) {
  // CR_4_5 => 0x01, CR_4_6 => 0x02 ...
  return (0x01 - static_cast<uint8_t>(CodingRate::CR_4_5) +
          static_cast<uint8_t>(cr));
}

constexpr uint8_t crForLog(rps_t const rps) {
  return (5 - static_cast<uint8_t>(CodingRate::CR_4_5) +
          static_cast<uint8_t>(rps.getCr()));
}

CONST_TABLE(uint16_t, BW_ENUM_TO_VAL)[] = {125, 250, 500, 0};

uint16_t bwForLog(rps_t const rps) {
  auto index = static_cast<uint8_t>(rps.getBw());
  return TABLE_GET_U2(BW_ENUM_TO_VAL, index);
}

} // namespace

void RadioSx1262::init() {
  PRINT_DEBUG(1, F("Radio Init"));
  hal.init();
  // manually reset radio
  // drive RST pin low
  hal.pin_rst(0);
  // wait >100us for SX1262 to detect reset
  hal_wait(OsDeltaTime::from_ms(1));
  // configure RST pin floating
  hal.pin_rst(2);
  // wait 5ms after reset
  hal_wait(OsDeltaTime::from_ms(5));
  wait_ready(hal);

  if (IS_DEBUG_ENABLE(1)) {
    // Check defaut config to see if reset is ok
    Sx1262Register<2> regres{0x0740, {0x00, 0x00}};
    read_register(hal, regres);
    PRINT_DEBUG(1, F("Config synch word %x==14"), regres.data[0]);
    PRINT_DEBUG(1, F("Config synch word %x==24"), regres.data[1]);
    print_status(get_status());
  }

  // go to sleep without saving state
  set_sleep();
}

// get random seed from wideband noise rssi
void RadioSx1262::init_random(uint8_t randbuf[16]) {
  PRINT_DEBUG(1, F("Init random"));

  init_config();
  set_rx_continious();

  delay(100);

  Sx1262Register<4> random_register = {0x0819, {0x00}};
  for (int i = 0; i < 4; i++) {
    read_register(hal, random_register);
    PRINT_DEBUG(1, F("Random %x %x %x %x "), random_register.data[0],
                random_register.data[1], random_register.data[2],
                random_register.data[3]);
    std::copy(random_register.begin(), random_register.end(), randbuf + 4 * i);
  }
  set_standby(false);
  set_sleep();
}

uint8_t RadioSx1262::rssi() const { return 0; }

// called by hal ext IRQ handler
// (radio goes to stanby mode after tx/rx operations)
uint8_t RadioSx1262::handle_end_rx(uint8_t *const framePtr) {
  uint16_t flags = get_irq_status();

  uint16_t const RxDone = 1 << 1;
  uint16_t const Timeout = 1 << 9;

  uint8_t length = 0;
  if (flags & RxDone) {
    // read message length
    length = read_frame(framePtr);
    // read rx quality parameters
    // SNR [dB] * 4
    // last_packet_snr_reg =
    // static_cast<int8_t>(hal.read_reg(LORARegPktSnrValue)); RSSI [dBm]  - 139
    // last_packet_rssi_reg = hal.read_reg(LORARegPktRssiValue);
  } else if (flags & Timeout) {
    // indicate timeout
    PRINT_DEBUG(1, F("RX timeout"));
  }

  // no interrupt
  set_dio1_irq_params(0x00);
  clear_all_irq();

  set_sleep();
  return length;
}

void RadioSx1262::handle_end_tx() const {
  // no interrupt
  set_dio1_irq_params(0x00);
  clear_all_irq();

  set_sleep();
}

void RadioSx1262::rst() const {
  // go to sleep without saving state
  set_sleep();
}

void RadioSx1262::tx(uint32_t const freq, rps_t const rps, int8_t const txpow,
                     uint8_t const *const framePtr, uint8_t const frameLength) {
  init_config();
  set_rf_frequency(freq);
  set_modulation_params_lora(rps);
  set_packet_params_lora(rps, frameLength, false);
  set_tx_power(txpow);
  // enable antenna switch for TX
  hal.pin_rxtx(1);

  write_frame(framePtr, frameLength);
  clear_all_irq();
  uint16_t const TxDone = 1 << 0;
  uint16_t const Timeout = 1 << 9;
  set_dio1_irq_params(TxDone | Timeout);
  set_tx();
  print_status(get_status());

  PRINT_DEBUG(1, F("TXMODE, freq=%" PRIu32 ", len=%d, SF=%d, BW=%d, CR=4/%d"),
              freq, frameLength, rps.sf + 6, bwForLog(rps), crForLog(rps));
}

void RadioSx1262::rx(uint32_t const freq, rps_t const rps, uint8_t const rxsyms,
                     OsTime const rxtime) {
  init_config();
  set_rf_frequency(freq);
  set_modulation_params_lora(rps);
  set_packet_params_lora(rps, MAX_LEN_FRAME, true);
  // enable antenna switch for RX
  hal.pin_rxtx(0);

  set_lora_symb_num_timeout(rxsyms);
  uint16_t const RxDone = 1 << 1;
  uint16_t const Timeout = 1 << 9;
  set_dio1_irq_params(RxDone | Timeout);
  clear_all_irq();

  // ramp up
  set_fs();
  // now instruct the radio to receive
  // busy wait until exact rx time
  if (rxtime < os_getTime()) {
    PRINT_DEBUG(1, F("RX LATE :  %" PRIu32 " WANTED, late %" PRIi32 " ms"),
                rxtime, (os_getTime() - rxtime).to_ms());
  }
  hal_waitUntil(rxtime);
  set_rx();
}

/**
 * Check the IO pin.
 * Return true if the radio has finish it's operation
 */
bool RadioSx1262::io_check() const {
  // print_status(get_status());
  // PRINT_DEBUG(1, F("Irq Status %x, Error %x"), get_irq_status(),
  //             get_device_errors());

  return hal.io_check1();
}

RadioSx1262::RadioSx1262(lmic_pinmap const &pins) : Radio(pins) {}

void RadioSx1262::set_sleep() const {
  PRINT_DEBUG(1, F("Set Radio to sleep"));

  // RTC Disable, Cold start
  send_command(hal, Sx1262Command<1>{RadioCommand::SetSleep, {0x00}});
}

void RadioSx1262::set_standby(bool use_xosc) const {
  // RC mode
  uint8_t const param1 = use_xosc ? 0x01 : 0x00;
  send_command(hal, Sx1262Command<1>{RadioCommand::SetStandby,
                                     {param1}});
}

void RadioSx1262::set_packet_type_lora() const {
  // Set paquet type LORA = 0x01
  send_command(hal, Sx1262Command<1>{RadioCommand::SetPacketType, {0x01}});
}

void RadioSx1262::set_modulation_params_lora(rps_t const rps) const {
  Sx1262Command<4> cmd = {RadioCommand::SetModulationParams, {0x00}};
  // sf
  cmd.parameter[0] = sf_to_parameter(rps.sf);
  cmd.parameter[1] = bw_to_parameter(rps.getBw());
  cmd.parameter[2] = cr_to_parameter(rps.getCr());
  // Must be enabled for: SF11/BW125, SF12/BW125, SF12/BW250
  if (((rps.sf == SF11 || rps.sf == SF12) && rps.getBw() == BandWidth::BW125) ||
      (rps.sf == SF12 && rps.getBw() == BandWidth::BW250)) {
    cmd.parameter[3] = 1;
  } else {
    cmd.parameter[3] = 0;
  }
  send_command(hal, cmd);
}

void RadioSx1262::set_rf_frequency(uint32_t const freq) const {
  Sx1262Command<4> cmd{RadioCommand::SetRfFrequency, {0x00}};
  uint32_t const rf_freq = (((uint64_t)freq << 25) / 32000000);
  wmsbf4(cmd.parameter, rf_freq);
  send_command(hal, cmd);
}

void RadioSx1262::set_packet_params_lora(rps_t rps, uint8_t frameLength,
                                         bool inv) const {
  Sx1262Command<6> cmd{RadioCommand::SetPacketParams,
                       {
                           // Preamble
                           0x00,
                           0x08,
                           // explicit header
                           0x00,
                           // length
                           frameLength,
                           // crc
                           static_cast<uint8_t>(rps.nocrc ? 0x00 : 0x01),
                           // inv for rx
                           static_cast<uint8_t>(inv ? 0x01 : 0x00),
                       }};

  send_command(hal, cmd);
}

void RadioSx1262::set_sync_word_lora() const {

  Sx1262Register<2> reg{0x740, {0x34, 0x44}};
  write_register(hal, reg);
}

void RadioSx1262::set_regulator_mode_dcdc() const {
  // regulator mode to DCDC
  send_command(hal, Sx1262Command<1>{RadioCommand::SetRegulatorMode, {0x01}});
}

void RadioSx1262::init_config() const {
  PRINT_DEBUG(1, F("Init Configure"));
  // Wakeup
  set_standby(false);
  set_regulator_mode_dcdc();

  // BOARD have TCXO, need calibration
  calibrate_image_868();
  set_DIO3_as_tcxo_ctrl();
  calibrate_all();
  set_standby(true);

  // set_DIO2_as_rf_switch_ctrl();
  set_packet_type_lora();
  set_sync_word_lora();
}

void RadioSx1262::set_tx_power(int8_t const txpow) const {
  int8_t pw = txpow;
  // high power PA: -9 ... +22 dBm
  if (pw > 22)
    pw = 22;
  if (pw < -9)
    pw = -9;
  // set PA config (and reset OCP to 140mA)
  send_command(hal, Sx1262Command<4>{RadioCommand::SetPaConfig,
                                     {0x04, 0x07, 0x00, 0x01}});
  // ramp up 200ms
  send_command(hal, Sx1262Command<2>{RadioCommand::SetTxParams,
                                     {static_cast<uint8_t>(pw), 0x04}});
}

void RadioSx1262::write_frame(uint8_t const *framePtr,
                              uint8_t frameLength) const {
  // set base address
  send_command(
      hal, Sx1262Command<2>{RadioCommand::SetBufferBaseAddress, {0x00, 0x00}});

  hal.beginspi();
  wait_ready(hal);
  // Write buffer
  hal.spi(RadioCommand::WriteBuffer);
  // offset
  hal.spi(0x00);
  for (uint8_t i = 0; i < frameLength; i++) {
    hal.spi(framePtr[i]);
  }
  hal.endspi();
}

uint8_t RadioSx1262::read_frame(uint8_t *framePtr) const {
  // read frame status
  Sx1262Command<2> frame_status = {RadioCommand::GetRxBufferStatus,
                                   {0x00, 0x00}};
  read_command(hal, frame_status);

  uint8_t len = frame_status.parameter[0] > MAX_LEN_FRAME
                    ? MAX_LEN_FRAME
                    : frame_status.parameter[0];

  hal.beginspi();
  wait_ready(hal);
  hal.spi(RadioCommand::ReadBuffer);
  // offset
  hal.spi(frame_status.parameter[1]);
  hal.spi(0x00);
  for (uint8_t i = 0; i < len; i++) {
    framePtr[i] = hal.spi(0x00);
  }
  hal.endspi();
  return len;
}

uint8_t RadioSx1262::get_status() const {
  // get satus is special format
  hal.beginspi();
  hal.spi(RadioCommand::GetStatus);
  uint8_t status = hal.spi(0x00);
  hal.endspi();
  return status;
}

uint16_t RadioSx1262::get_device_errors() const {
  Sx1262Command<2> cmd = {RadioCommand::GetDeviceErrors, {0x00}};
  read_command(hal, cmd);
  return rmsbf2(cmd.parameter);
}

uint16_t RadioSx1262::get_irq_status() const {
  Sx1262Command<2> cmd = {RadioCommand::GetIrqStatus, {0x00}};
  read_command(hal, cmd);
  return rmsbf2(cmd.parameter);
}

void RadioSx1262::clear_all_irq() const {
  send_command(hal,
               Sx1262Command<2>{RadioCommand::ClearIrqStatus, {0x03, 0xFF}});
}

void RadioSx1262::set_dio1_irq_params(uint16_t mask) const {

  uint8_t maskH = mask >> 8;
  uint8_t maskL = mask & 0xFF;

  send_command(hal, Sx1262Command<8>{RadioCommand::SetDioIrqParams,
                                     {maskH, maskL,
                                      // DIO1
                                      maskH, maskL,
                                      // DIO2
                                      0x00, 0x00,
                                      // DIO 3
                                      0x00, 0x00}});
}

void RadioSx1262::set_rx() const {
  // Timeout 0 (symbol timeout)
  send_command(hal, Sx1262Command<3>{RadioCommand::SetRx, {0x00, 0x00, 0x00}});
}

void RadioSx1262::set_rx_continious() const {
  send_command(hal, Sx1262Command<3>{RadioCommand::SetRx, {0xFF, 0xFF, 0xFF}});
}

uint8_t RadioSx1262::get_rssi_inst() const {
  Sx1262Command<1> cmd{RadioCommand::GetRssiInst, {0x00}};
  read_command(hal, cmd);
  return cmd.parameter[0];
}

void RadioSx1262::set_tx() const {
  // Timeout 10s => 0x09C400
  send_command(hal, Sx1262Command<3>{RadioCommand::SetTx, {0x09, 0xC4, 0x00}});
}

void RadioSx1262::set_fs() const {

  send_command(hal, Sx1262Command<0>{RadioCommand::SetFs, {}});
}

void RadioSx1262::set_lora_symb_num_timeout(uint8_t rxsyms) const {
  send_command(hal,
               Sx1262Command<1>{RadioCommand::SetLoRaSymbNumTimeout, {rxsyms}});
}

void RadioSx1262::calibrate_image_868() const {
  send_command(hal,
               Sx1262Command<2>{RadioCommand::CalibrateImage, {0xD7, 0xDB}});
}

void RadioSx1262::set_DIO2_as_rf_switch_ctrl() const {
  send_command(hal,
               Sx1262Command<1>{RadioCommand::SetDIO2AsRfSwitchCtrl, {0x01}});
}

void RadioSx1262::calibrate_all() const {
  send_command(hal, Sx1262Command<1>{RadioCommand::Calibrate, {0x7F}});
}

void RadioSx1262::clear_device_errors() const {
  send_command(hal, Sx1262Command<1>{RadioCommand::ClearDeviceErrors, {0x00}});
}

void RadioSx1262::set_DIO3_as_tcxo_ctrl() const {
  // 1.8V  5ms
  send_command(hal, Sx1262Command<4>{RadioCommand::SetDIO3AsTcxoCtrl,
                                     {0x02, 0x00, 0x01, 0x40}});
}
