#pragma once

class TwoWire {
 public:
  void begin(int, int) {}
  void setClock(unsigned long) {}
};

extern TwoWire Wire;
