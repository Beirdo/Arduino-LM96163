#include <Arduino.h>
#include <Wire.h>

#include "LM96163.h"

const uint8_t lm96163_status_regs[LM96163_MAX_STATUS] = {
  0x46, 0x47, 0x00, 0x01, 0x10, 0x02, 0x16, 0x33
};

void lm96163_alert_isr(void);

void lm96163_alert_isr(void)
{
  _status[LM96163_STATUS_ALERT_STATUS] = read_register(0x02);
}

LM96163::LM96163()
{
  _lut = 0;
  _pin_alert = -1;
  _pin_tcrit = -1;
  _on = false;
  _pwm_override = 0;
  _interrupts_on = false;
  _initialized = false;
  _pulse_per_rev = 2;
}

LM96163::~LM96163()
{
  disableInterrupts();
}

void LM96163::setPulsePerRev(uint8_t value)
{
  if (value > 0 && value <= 4) {
    _pulse_per_rev = value;
  }
}

void LM96163::setLUT(LM96163_LUT_t *lut, uint8_t hysteresis)
{
  _lut = lut;
  _hysterisis = hysteresis;
  fanOnOff(true);
}

bool LM96163::begin(TwoWire &wire, uint8_t pin_alert = -1, uint8_t pin_tcrit = -1)
{
  _wire = wire;
  disableInterrupts();
  _pin_alert = pin_alert;
  _pin_tcrit = pin_tcrit;

  _wire.begin();
  return configure();
}

bool LM96163::configure(bool fast)
{
  _wire.beginTransmission(I2C_ADDR_LM96163);
  _initialized  = (_wire.endTransmission() == 0);

  if (!_initialized) {
    return false;
  }

  if (!fast || _lut) {
    // Wait for not ready to clear
    uint8_t value = 0xFF;

    while (value & 0x80) {
      value = read_register(0x33);
      delay(1);
    }

    // enable remote diode TruTherm mode
    write_register(0x30, 0x02);

    // setup PWM and tach
    uint8_t pwm_control = 0x38;
    write_register(0x4A, pwm_control);

    // setup spin-up settings (do nothing special)
    write_register(0x4B, 0x00);

    // setup PWM frequency as 22.5kHz with extened duty cycle resolution
    write_register(0x4D, 0x08);

    // setup enhanced config
    write_register(0x45, 0x7B);
    
    if (_lut && _on) {
      // Lookup table offset
      write_register(0x4E, 0x00);

      uint8_t regnum = 0x4F;
      for (int i = 0; i < 12; i++) {
        write_register(regnum++, _lut[i].temperature);
        write_register(regnum++, _lut[i].pwm_duty_cycle);
      }

      // Turn on the LUT use
      pwm_control &= 0x1B;
      write_register(0x4A, pwm_control;
      return true;
    }
  }


  // Make sure LUT is shut off.
  write_register(0x4A, 0x38);

  if (!_on) {
    _pwm_override = 0;
  }
  write_register(0x4C. _pwm_override);
  return true;
}

void LM96163::enableInterrupts(void)
{
  if (!_interrupts_on) {
    if (_alert_pin >= 0) {
      attachInterrupt(digitalPinToInterrupt(_alert_pin), lm96163_alert_isr, FALLING);
    }
  }
  _interrupts_on = true;
}

void LM96163::disableInterrupts(void)
{
  if (_interrupts_on) {
    if (_alert_pin >= 0) {
      detachInterrupt(digitalPinToInterrupt(_alert_pin));
    }
  }
  _interrupts_on = false;
}

void LM96163:;setAlertMask(uint8_t value)
{
  write_register(0x16, value);
}

void LM96163:;fanOnOff(bool on, uint8_t override_pwm)
{
  _on = on;
  _override_pwm = override_pwm;

  if (!_initialized) {
    return;
  }

  configure(true);
}

void LM96163:;pollStatus(void)
{
  for (i = 0; i < LM96163_MAX_STATUS; i++ ) {
    _status[i] = read_register(lm96163_status_regs[i]);
  }

  // Convert the tach counts to RPM
  uint16_t tach = (uint16_t)_status[LM96163_STATUS_TACH_LSB] | ((uint16_t)_status[LM96163_STATUS_TACH_MSB] << 8);
  if (tach == 0xFFFF || tach == 0x0000) {
    tach = 0x0000;
  } else {
    tach = (uint16_t)(10800000 / (int)(_pulse_per_count * tach));
  }

  _status[LM96164_STATUS_TACH_LSB] = (uint8_t)(tach & 0x00FF);
  _status[LM96164_STATUS_TACH_MSB] = (uint8_t)((tach >> 8) & 0x00FF);
}

uint16_t LM96163:;getStatus(int index)
{
  if (index < 0 || index >= LM96163_MAX_STATUS) {
    return 0;
  }

  if (index == LM96163_STATUS_TACH || index == LM96163_STATUS_REMOTE_TEMP) {
    return (uint16_t)_status[index] | ((uint16_t)_status[index + 1] << 8);
  }

  return (uint16_t)status[index];
}

void LM96163::write_register(uint8_t regnum, uint8_t value)
{
  if (!_initialized) {
    return;
  }

  _wire.beginTransmission(I2C_ADDR_LM96163);
  _wire.write(regnum);
  _wire.write(value);
  _wire.endTransmission();
}

uint8_t LM96163::read_register(uint8_t regnum)
{
  if (!_initialized) {
    return 0x00;
  }

  _wire.beginTransmission(I2C_ADDR_LM96163);
  _wire.write(regnum);
  _wire.endTransmission(false);
  
  _wire.requestFrom(I2C_ADDR_LM96163, 1);

  return _wire.read();
}

