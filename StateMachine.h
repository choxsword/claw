#pragma once

#include "Claw.h"
//状态机相关宏
#define INIT 0
#define POS_CTRL 1
#define FORCE_CTRL 2

#define STATE_LOOP 3
#define EXIT_LOOP 4
#define STATE_STABLE 5
#define STATE_SLIP 6
#define STATE_UNSTABLE 7
#define STATE_FAST_SLIP 8
#define STATE_HOLD_ON 9
#define STATE_EXIT 10
#define STATE_STABLE_1 11

//subscribe相关宏
#define SUB_POS 0
#define SUB_FORCE 1
#define SUB_SPEED 2 

//信息相关宏
#define INFO_HOLD_ON 0
#define INFO_HINT 1
#define INFO_INT 2

namespace xzj
{
	class Controller;
	class IClawState {
		friend Controller;
	public:
		void tick();
		IClawState( Controller* _ctrl) {
			ctrl = _ctrl;
		}
	protected:
		void general();
		virtual void state_related(){}
		bool is_trans = false;
		Controller* ctrl = nullptr;
	};

	class State_Init:public IClawState {
	public:
		State_Init(Controller * ctrl) :IClawState(ctrl ){}
		void state_related();
	};

	class State_PosCtrl:public IClawState {
	public:
		State_PosCtrl(Controller * ctrl) :IClawState(ctrl){}
		void state_related();
	};
	
	class State_ForceCtrl :public IClawState {
	public:
		State_ForceCtrl(Controller* ctrl):IClawState(ctrl){}
		void state_related();
	};


	class Controller {
	public:
		Controller(MyServo& _servo, Sensor & _sensor, Alarm& _alarm) :
			servo(_servo), sensor(_sensor), alarm(_alarm)
		{
			for (auto& i : szStates) {
				i = nullptr;
			}
			szStates[INIT] = new State_Init(this);
			szStates[POS_CTRL] = new State_PosCtrl(this);
			szStates[FORCE_CTRL] = new State_ForceCtrl(this);
			CurState = szStates[INIT];
			CurState->is_trans = false;
			clear_indices();
			for (auto &i : subscribe) {
				i = false;
			}
		}

	private:
		IClawState * CurState = nullptr;
		IClawState* szStates[10];

		MyServo & servo;
		Sensor & sensor;
//		Sensor & m_ViceSensor;
		Alarm & alarm;

		double hold_force = 1;//N;
		double kp = 15;
		double ki = 10;
		double fd = 0;
		double dPICtrl_P = 15.0;
		double dPICtrl_I = 1;
		int iPICtrl_Sample = 50.0;
		int CmdDelay[9] = { 10,260,160,120,60,50,40,40,40 };
		double dPCtrl_P = 10;
		int iPCtrl_Sample = 10;
		unsigned long is_stable_pre_time;

		int iFuzzy_Sample = 50;

		int cur_speed = 0;
		double dCurSpeed= 0.0;
		int tar_pos = 0;
		int stable_cnt = 0;
		bool bIsStable = true;
		double indices[20];
		//Fuzzy_controller fuzzy;
		Upstream up_conn;
	public:

		bool is_interrupt = false;//用于指示某个死循环中是否遇到了需要处理的指令
		void check_bus();
		void handle();//处理上位机发来的请求
		void init();
		void publish();

		void debug_report(Command cmd, int val);

		Command get_cmd();

		bool is_interruptted() {
			return is_interrupt;
		}

		~Controller() {
			for (auto i : szStates) {
				delete i;
			}
		}

		void trans(int state) {
			//debug_report(Command::DebugState, state);
			CurState = szStates[state];
			CurState->is_trans = true;
		}

		void tick() {
			CurState->tick();
		}

		void grab_by_try(const double exp_force, const double v0 = 80);

		void open_claw();
		void stop_claw();

		void grab_by_p(const double exp_force, const double v0 = 80);
		void test_p(const double exp_force, const double v0 = 0);

		void grab_by_pi(const double, const double = 80);
		void test_pi(const double, const double = 0);

		void grab_by_admit(const double);
		void set_pi(double _kp = 10.0, double _ki = 8);

		void hold_on(bool cond);
		void launch(double);

		void wait_print_sensor(unsigned long time);
		void wait_print_pos(unsigned long time);

		void grab_by_fuzzy(const double exp_force, const double v0 = 80);
	private:
		void Delay(int cnt);
		void Delay(int cnt, int speed);
		void hold_lightly();
		bool stable_judge(double e,double de);
		void clear_stable();
		double stable_judge_fuzzy(const double& fd, const double&fk);
		int is_stable(double indicator,bool add_fd=false);
		void clear_indices();
		void add_fd(double val);
	public:
		bool subscribe[3];
		bool bToBeHandled = false;//用于跳出死循环后判断是否有待处理任务

		void handle_subscribe() {
			int val = up_conn.rcv_data;
			subscribe[SUB_POS] = ((val & 4) != 0);
			subscribe[SUB_FORCE] = ((val & 2) != 0);
			subscribe[SUB_SPEED] = ((val & 1) != 0);
		}

	};


}