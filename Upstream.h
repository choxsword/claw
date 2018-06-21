#pragma once
#include "Arduino.h"


namespace xzj
{
enum class Command{ Undefined = 0, OpenClaw, CloseClaw, Stop, Unload, Load };

class Upstream
{
  public:
	  Upstream(int _baud = 1e6) : baud(_baud) { Serial.begin(baud); }
    Command check();
    void feed_back(const Command &status);
  private:
    int baud;
};

} // namespace xzj