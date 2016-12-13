#include "Node.h"

namespace photosynq {

void Node::begin() {
  config_.set_buffer(get_buffer());
  config_.validator_.set_node(*this);
  config_.reset();
  config_.load();
  state_.set_buffer(get_buffer());
  state_.validator_.set_node(*this);
  state_.reset();
#if !defined(DISABLE_SERIAL)
  // Start Serial after loading config to set baud rate.
  Serial.begin(115200);
#endif  // #ifndef DISABLE_SERIAL
  adc_ = new ADC();
  if (config_._.i2c_address > 0) { Wire.setClock(400000); }
}


void Node::set_i2c_address(uint8_t value) {
  // Validator expects `uint32_t` by reference.
  uint32_t address = value;
  /* Validate address and update the active `Wire` configuration if the address
   * is valid. */
  config_.validator_.i2c_address_(address, config_._.i2c_address);
  config_._.i2c_address = address;
}

}  // namespace photosynq
