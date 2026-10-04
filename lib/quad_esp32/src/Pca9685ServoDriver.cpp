#include "Pca9685ServoDriver.h"

#include <cmath>

namespace qspider {

namespace {
constexpr uint8_t kMode1 = 0x00;
constexpr uint8_t kMode2 = 0x01;
constexpr uint8_t kLed0OnL = 0x06;
constexpr uint8_t kAllLedOnL = 0xFA;
constexpr uint8_t kPreScale = 0xFE;

constexpr uint8_t kMode1Restart = 0x80;
constexpr uint8_t kMode1AutoInc = 0x20;
constexpr uint8_t kMode1Sleep = 0x10;
constexpr uint8_t kMode1AllCall = 0x01;
constexpr uint8_t kMode2OutDrv = 0x04;  // totem-pole outputs
constexpr uint8_t kFullOffBit = 0x10;   // bit 4 of LEDn_OFF_H

constexpr uint16_t kPhaseStepTicks = 400;  // 8 phase slots, max start 2800 + 492 < 4096
}  // namespace

Pca9685ServoDriver::Pca9685ServoDriver(const PcaConfig& cfg, TwoWire& wire) : cfg_(cfg), wire_(wire) {
  const float pre = cfg.oscillatorHz / (4096.0f * cfg.pwmFrequencyHz) - 1.0f;
  prescale_ = static_cast<uint8_t>(std::lround(pre < 3.0f ? 3.0f : (pre > 255.0f ? 255.0f : pre)));
  usPerTick_ = (prescale_ + 1) / cfg.oscillatorHz * 1e6f;
}

void Pca9685ServoDriver::setOutputEnable(bool enabled) {
  if (cfg_.outputEnablePin < 0) return;
  pinMode(cfg_.outputEnablePin, OUTPUT);
  digitalWrite(cfg_.outputEnablePin, enabled ? LOW : HIGH);  // OE is active low
}

bool Pca9685ServoDriver::end(bool ok) {
  if (!ok) ++errors_;
  return ok;
}

bool Pca9685ServoDriver::writeReg(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(cfg_.i2cAddress);
  wire_.write(reg);
  wire_.write(value);
  return end(wire_.endTransmission() == 0);
}

bool Pca9685ServoDriver::readRegs(uint8_t reg, uint8_t* out, uint8_t count) {
  wire_.beginTransmission(cfg_.i2cAddress);
  wire_.write(reg);
  if (wire_.endTransmission(false) != 0) return end(false);
  if (wire_.requestFrom(static_cast<uint16_t>(cfg_.i2cAddress), count) != count) return end(false);
  for (uint8_t i = 0; i < count; ++i) out[i] = static_cast<uint8_t>(wire_.read());
  return true;
}

bool Pca9685ServoDriver::present() {
  wire_.beginTransmission(cfg_.i2cAddress);
  return wire_.endTransmission() == 0;
}

bool Pca9685ServoDriver::begin() {
  setOutputEnable(false);  // hardware-disable outputs first, if OE is wired
  if (!wireStarted_) {
    wire_.begin(cfg_.sdaPin, cfg_.sclPin, cfg_.i2cClockHz);
    wire_.setTimeOut(5);  // a stuck bus must not stall the control loop for long
    wireStarted_ = true;
  }
  if (!present()) return end(false);

  bool ok = writeReg(kMode1, kMode1Sleep | kMode1AllCall);  // prescale can only change while asleep
  ok = ok && writeReg(kPreScale, prescale_);
  // Every channel fully off.
  wire_.beginTransmission(cfg_.i2cAddress);
  wire_.write(kAllLedOnL);
  wire_.write(0x00);
  wire_.write(0x00);
  wire_.write(0x00);
  wire_.write(kFullOffBit);
  ok = ok && end(wire_.endTransmission() == 0);
  ok = ok && writeReg(kMode2, kMode2OutDrv);
  ok = ok && writeReg(kMode1, kMode1AutoInc | kMode1AllCall);  // wake up
  delayMicroseconds(600);                                       // oscillator start-up (500 us)
  ok = ok && writeReg(kMode1, kMode1Restart | kMode1AutoInc | kMode1AllCall);
  ok = ok && verify();
  if (ok) setOutputEnable(true);  // registers say "off", so enabling OE is safe
  return ok;
}

uint16_t Pca9685ServoDriver::phaseOf(int channel) const {
  return static_cast<uint16_t>((channel % 8) * kPhaseStepTicks);
}

uint16_t Pca9685ServoDriver::degToTicks(float deg) const {
  const float us = cfg_.pulseAtMinDegUs + (deg / 180.0f) * (cfg_.pulseAtMaxDegUs - cfg_.pulseAtMinDegUs);
  return static_cast<uint16_t>(std::lround(us / usPerTick_));
}

float Pca9685ServoDriver::ticksToDeg(uint16_t ticks) const {
  const float us = ticks * usPerTick_;
  return (us - cfg_.pulseAtMinDegUs) / (cfg_.pulseAtMaxDegUs - cfg_.pulseAtMinDegUs) * 180.0f;
}

bool Pca9685ServoDriver::writeAll(const float (&angleDeg)[kChannels]) {
  wire_.beginTransmission(cfg_.i2cAddress);
  wire_.write(kLed0OnL);
  for (int ch = 0; ch < kChannels; ++ch) {
    if (std::isnan(angleDeg[ch])) {
      wire_.write(0x00);
      wire_.write(0x00);
      wire_.write(0x00);
      wire_.write(kFullOffBit);
    } else {
      const uint16_t on = phaseOf(ch);
      const uint16_t off = static_cast<uint16_t>(on + degToTicks(angleDeg[ch]));
      wire_.write(on & 0xFF);
      wire_.write(on >> 8);
      wire_.write(off & 0xFF);
      wire_.write((off >> 8) & 0x0F);
    }
  }
  return end(wire_.endTransmission() == 0);
}

bool Pca9685ServoDriver::allOff() {
  setOutputEnable(false);  // works even if I2C is dead
  wire_.beginTransmission(cfg_.i2cAddress);
  wire_.write(kAllLedOnL);
  wire_.write(0x00);
  wire_.write(0x00);
  wire_.write(0x00);
  wire_.write(kFullOffBit);
  return end(wire_.endTransmission() == 0);
}

bool Pca9685ServoDriver::verify() {
  uint8_t mode1 = 0, pre = 0;
  if (!readRegs(kMode1, &mode1, 1) || !readRegs(kPreScale, &pre, 1)) return false;
  // After a power-on reset MODE1 = 0x11 (asleep, no auto-increment) and PRE_SCALE = 0x1E.
  return !(mode1 & kMode1Sleep) && (mode1 & kMode1AutoInc) && pre == prescale_;
}

bool Pca9685ServoDriver::readChannel(int channel, float& angleDeg, bool& off) {
  if (channel < 0 || channel >= kChannels) return false;
  uint8_t r[4];
  if (!readRegs(static_cast<uint8_t>(kLed0OnL + 4 * channel), r, 4)) return false;
  off = (r[3] & kFullOffBit) != 0;
  const uint16_t on = r[0] | ((r[1] & 0x0F) << 8);
  const uint16_t offTicks = r[2] | ((r[3] & 0x0F) << 8);
  angleDeg = off ? 0.0f : ticksToDeg(static_cast<uint16_t>(offTicks - on));
  return true;
}

bool Pca9685ServoDriver::sleep() {
  return writeReg(kMode1, kMode1Sleep | kMode1AutoInc | kMode1AllCall);
}

}  // namespace qspider
