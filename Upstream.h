#pragma once
#include "Arduino.h"


namespace xzj
{
enum class Command{
	//0-100为无需关心返回信息的
	Undefined = 0, RcvCmd, Subscribe,
	OpenClaw = 10, CloseClaw, Stop, Unload, Load, SetSpeed, MoveTo,
	ReadPos = 101, ReadSpeed, ReadForce
};

class Upstream
{
  public:
	  Upstream(int _baud = 115200) : baud(_baud) { Serial.begin(baud); }//实际上没有效果，因为upstream
	  //是在编译期生成的，需要在运行期才能设置实际波特率
    void check();
    void feed_back(const Command &status,int val);
	int rcv_data=-1;
	Command rcv_cmd = Command::Undefined;
  private:
    int baud;
};

} // namespace xzj