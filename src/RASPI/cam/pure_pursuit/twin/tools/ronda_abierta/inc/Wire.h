#pragma once
#include <cstdint>
class TwoWire { public: void begin(int = 0, int = 0) {} void setClock(uint32_t) {} };
extern TwoWire Wire;
