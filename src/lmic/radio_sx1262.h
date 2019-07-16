/*******************************************************************************

 *******************************************************************************/

#ifndef _radio_sx1262_h_
#define _radio_sx1262_h_

#include "lorabase.h"
#include "osticks.h"
#include "radio.h"
#include <stdint.h>

class RadioSx1262 final : public Radio {

public:
  explicit RadioSx1262(lmic_pinmap const &pins);
  void init(void) override;
  void rst() const override;
  void tx(uint32_t freq, rps_t rps, int8_t txpow, uint8_t const *framePtr,
          uint8_t frameLength) override;
  void rx(uint32_t freq, rps_t rps, uint8_t rxsyms, OsTime rxtime) override;

  void init_random(uint8_t randbuf[16]) override;
  uint8_t handle_end_rx(uint8_t *framePtr) override;
  void handle_end_tx() const override;
  bool io_check() const override;

  uint8_t rssi() const override;

private:
  void set_sleep() const;
  void set_standby(bool use_xosc) const;
  void set_packet_type_lora() const;
  void set_modulation_params_lora(rps_t rps) const;
  void set_rf_frequency(uint32_t freq) const;
  void set_sync_word_lora() const;
  void set_packet_params_lora(rps_t rps, uint8_t frameLength, bool inv) const;
  void set_tx_power(int8_t txpow) const;
  void set_regulator_mode_dcdc() const;

  void init_config() const;

  void write_frame(uint8_t const *framePtr, uint8_t frameLength) const;
  uint8_t read_frame(uint8_t *framePtr) const;
  uint8_t get_status() const;
  uint16_t get_device_errors() const;
  uint16_t get_irq_status() const;

  void clear_all_irq() const;
  void set_dio1_irq_params(uint16_t mask) const;
  void set_rx() const;
  void set_rx_continious() const;
  void set_tx() const;
  void set_fs() const;
  void set_lora_symb_num_timeout(uint8_t rxsyms) const;
  void calibrate_image_868() const;
  uint8_t get_rssi_inst() const;
  void set_DIO2_as_rf_switch_ctrl() const;
  void calibrate_all() const;
  void clear_device_errors() const;
  void set_DIO3_as_tcxo_ctrl() const;
};

#endif