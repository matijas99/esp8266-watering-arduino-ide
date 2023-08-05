#include "Arduino.h"
#include "MCP23017-SOLDERED.h"

//////////////////////////////////////////////////////////
// PIN
//////////////////////////////////////////////////////////
class Pin {
  protected:
    String _pinAddress;

  public:
    virtual void setPinMode(uint8_t mode);
    virtual void doDigitalWrite(uint8_t val);
    virtual int doDigitalRead();
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PIN NATIVE
//////////////////////////////////////////////////////////
class PinNative : public Pin {
  protected:
    uint8_t _pinAddress;

  public:
    PinNative(uint8_t pinAddress) {
        _pinAddress = pinAddress;
    }

    void setPinMode(uint8_t mode) {
      pinMode(_pinAddress, mode);
    }
    void doDigitalWrite(uint8_t val) {
      digitalWrite(_pinAddress, val);
    }
    int doDigitalRead() {
      return digitalRead(_pinAddress);
    }
};
//////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////
// PIN EXTENDER
//////////////////////////////////////////////////////////
class PinExtender : public Pin {
  protected:
    uint8_t _pinAddress;
    MCP_23017* _mcp;

  public:
    PinExtender(MCP_23017* mcp, uint8_t pinAddress) {
        _mcp = mcp;
        _pinAddress = pinAddress;
    }

    void setPinMode(uint8_t mode) {
      _mcp->pinMode(_pinAddress, mode);
    }
    void doDigitalWrite(uint8_t val) {
      _mcp->digitalWrite(_pinAddress, val);
    }
    int doDigitalRead() {
      return _mcp->digitalRead(_pinAddress);
    }
};
//////////////////////////////////////////////////////////