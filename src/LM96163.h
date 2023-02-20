#ifndef __LM96163_h_
#define __LM96163_h_

#include <Arduino.h>
#include <Wire.h>

// This is a highlander chip... There can only be one...
#define I2C_ADDR_LM96163  0x4C

typedef struct {
  uint8_t temperature;
  uint8_t pwm_duty_cycle;
} LM96163_LUT_t;

enum {
  LM96163_STATUS_TACH,
  LM96163_STATUS_TACH_LSB = LM96163_STATUS_TACH,
  LM96163_STATUS_TACH_MSB,
  LM96163_STATUS_LOCAL_TEMP,
  LM96163_STATUS_REMOTE_TEMP,
  LM96163_STATUS_REMOTE_TEMP_MSB = LM96163_STATUS_REMOTE_TEMP,
  LM96163_STATUS_REMOTE_TEMP_LSB,
  LM96163_STATUS_ALERT_STATUS,
  LM96163_STATUS_ALERT_MASK,
  LM96163_STATUS_POR_STATUS,
  LM96163_MAX_STATUS,
};

extern const uint8_t lm96163_status_regs[LM96163_MAX_STATUS];

class LM96163
{
  public:
    LM96163();
    ~LM96163();
    void setLUT(LM96163_LUT_t *lut, uint8_t hysteresis = 10);
    bool begin(TwoWire *wire, uint8_t pin_alert = -1, uint8_t pin_tcrit = -1);

    void enableInterrupts(void);
    void disableInterrupts(void);
    void setAlertMask(uint8_t value);
    void fanOnOff(bool on, uint8_t override_pwm = 0);
    void pollStatus(void);
    uint16_t getStatus(int index);
    void setPulsePerRev(uint8_t value);
    
    friend void lm96163_alert_isr(void);

  protected:
    void setAlertStatus(uint8_t status) { _status[LM96163_STATUS_ALERT_STATUS] = status; };
    bool _initialized;
    TwoWire *_wire;
    uint8_t _alert_pin;
    uint8_t _alert_mask;
    uint8_t _tcrit_pin;
    bool _interrupts;
    LM96163_LUT_t *_lut;
    uint8_t _hysteresis;
    bool _on;
    bool _interrupts_on;
    uint8_t _pwm_override;
    uint8_t _status[LM96163_MAX_STATUS];
    uint8_t _pulse_per_rev;

    bool configure(bool fast = false);
    void write_register(uint8_t regnum, uint8_t value);
    uint8_t read_register(uint8_t regnum);
};

extern LM96163 lm96163;

#endif

