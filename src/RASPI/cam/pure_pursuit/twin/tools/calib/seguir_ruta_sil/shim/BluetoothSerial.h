#pragma once
#include "Arduino.h"

class BluetoothSerial : public SrSerial {
 public:
  BluetoothSerial() : SrSerial(2) {}
};
