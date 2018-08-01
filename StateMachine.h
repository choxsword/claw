#pragma once

#include "Claw.h"
#define INIT 0
#define POS_CTRL 1
//#define FORCE_CTRL 2
#define SUB_POS 0
#define SUB_FORCE 1
#define SUB_SPEED 2 

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
	
	class Controller {
	public:
		Controller(MyServo& _servo, Sensor & _sensor, Alarm& _alarm) :servo(_servo), sensor(_sensor), alarm(_alarm) {
			for (auto& i : szStates) {
				i = nullptr;
			}
			szStates[INIT] = new State_Init(this);
			szStates[POS_CTRL] = new State_PosCtrl(this);
			CurState = szStates[INIT];
			CurState->is_trans = false;

			for (auto &i : subscribe) {
				i = false;
			}
		}

	private:
		IClawState * CurState = nullptr;
		IClawState* szStates[10];

		MyServo & servo;
		Sensor & sensor;
		Alarm & alarm;
		double kp = 15;
		double ki = 10;
		int cur_speed = 0;
		int tar_pos = 0;
		Fuzzy_controller fuzzy;
		Upstream up_conn;
	public:

		void check_bus();
		void handle();
		void init();
		void publish();
		Command get_cmd();

		~Controller() {
			for (auto i : szStates) {
				delete i;
			}
		}

		void trans(int state) {
			CurState = szStates[state];
			CurState->is_trans = true;
		}

		void tick() {
			CurState->tick();
		}

		void grab_by_p(const double exp_force, const double v0 = 0);
		void test_p(const double exp_force, const double v0 = 0);

		void grab_by_pi(const double, const double = 0);
		void test_pi(const double, const double = 0);

		void grab_by_admit(const double);
		void set_pi(double _kp = 10.0, double _ki = 8);

		void hold_on(double);
		void launch(double);

		void wait_print_sensor(unsigned long time);
		void wait_print_pos(unsigned long time);

		void test_fuzzy(const double exp_force, const double v0 = 0);
		void grab_by_fuzzy(const double exp_force, const double v0 = 0);

	public:
		bool subscribe[3];
		void handle_subscribe() {
			int val = up_conn.rcv_data;
			subscribe[SUB_POS] = ((val & 4) != 0);
			subscribe[SUB_FORCE] = ((val & 2) != 0);
			subscribe[SUB_SPEED] = ((val & 1) != 0);
		}

	};


}