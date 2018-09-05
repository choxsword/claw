#pragma once
#include "Arduino.h"


namespace xzj
{
enum class Command{
	//0-100为无需关心返回信息的，即只会返回一个简单的确认包
	Undefined = 0, RcvCmd,
	OpenClaw = 10, CloseClaw, Stop, Unload, Load, MoveTo,MoveSpeed,
	GrabByFuzzy, GrabByP, GrabByPI, GrabByImpedance, GrabByTry,
	//配置参数30-50
	Subscribe = 30, SetSpeed,SetForce, SetPCtrl_P, SetPCtrl_Sample,
	SetPICtrl_P, SetPICtrl_I, SetPICtrl_Sample,
	SetFuzzy_Sample,

	ReadPos = 101, ReadSpeed, ReadForce,ReadForceVice,ReportNewForce,ReportForceTiny,
	//200以上为调试用途
	DebugState = 200, DebugValueInt, DebugValueDouble,DebugValueTiny, DebugInfo, DebugChar
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