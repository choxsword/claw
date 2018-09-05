#pragma once
#include "Arduino.h"


namespace xzj
{
enum class Command{
	//0-100Ϊ������ķ�����Ϣ�ģ���ֻ�᷵��һ���򵥵�ȷ�ϰ�
	Undefined = 0, RcvCmd,
	OpenClaw = 10, CloseClaw, Stop, Unload, Load, MoveTo,MoveSpeed,
	GrabByFuzzy, GrabByP, GrabByPI, GrabByImpedance, GrabByTry,
	//���ò���30-50
	Subscribe = 30, SetSpeed,SetForce, SetPCtrl_P, SetPCtrl_Sample,
	SetPICtrl_P, SetPICtrl_I, SetPICtrl_Sample,
	SetFuzzy_Sample,

	ReadPos = 101, ReadSpeed, ReadForce,ReadForceVice,ReportNewForce,ReportForceTiny,
	//200����Ϊ������;
	DebugState = 200, DebugValueInt, DebugValueDouble,DebugValueTiny, DebugInfo, DebugChar
};

class Upstream
{
  public:
	  Upstream(int _baud = 115200) : baud(_baud) { Serial.begin(baud); }//ʵ����û��Ч������Ϊupstream
	  //���ڱ��������ɵģ���Ҫ�������ڲ�������ʵ�ʲ�����
    void check();
    void feed_back(const Command &status,int val);
	int rcv_data=-1;
	Command rcv_cmd = Command::Undefined;
  private:
    int baud;
};

} // namespace xzj