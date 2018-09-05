#include "StateMachine.h"
#include "Upstream.h"
#include "Claw.h"
using namespace xzj;

void IClawState::tick() {
	general();
	state_related();
}

void IClawState::general() {
	if (!is_trans&&!ctrl->is_interrupt)
	{
		ctrl->check_bus();//如果是从别的状态转移过来的，说明之前的指令还没被处理，因此不需要再次读取
	}
	
	switch (ctrl->get_cmd())
	{
		case Command::Undefined:break;
		case Command::Subscribe: {

			ctrl->handle(); 
			break;
		}
		default:break;
	}
	static unsigned char tick_cnt = 0;

	if (tick_cnt < 20) {
		++tick_cnt;
		return;
	}
	else {
		tick_cnt = 0; 
		ctrl->publish();
	}
}

void State_Init::state_related(){
	switch (ctrl->get_cmd())
	{
	case Command::Undefined:
		break;
	case Command::Stop:
		break;
	case Command::MoveTo:
	{
		ctrl->trans(POS_CTRL); 
		break; 
	}
	case Command::GrabByFuzzy:
	case Command::GrabByImpedance:
	case Command::GrabByPI:
	case Command::GrabByTry:
	case Command::GrabByP:
	{	
		ctrl->trans(FORCE_CTRL);
		break;
	}
	default:
		ctrl->handle();
	}
}


void State_ForceCtrl::state_related() {

	Command&& rcv_cmd = ctrl->get_cmd();
	int iRcvCmd = static_cast<int>(rcv_cmd);
	if (iRcvCmd <= 50 && iRcvCmd >= 30) {
		ctrl->handle();
		return;
	}

	switch (rcv_cmd)
	{
		case Command::Undefined:
			break;
		case Command::Stop:
			break;
		case Command::MoveTo:
		case Command::OpenClaw:
		case Command::CloseClaw:
		{
			ctrl->is_interrupt = true;
			ctrl->trans(POS_CTRL);
			
			break;
		}

		default:
		{
			if (!is_trans&&!ctrl->is_interrupt)
			{
					ctrl->is_interrupt = true;
					//ctrl->debug_report(Command::DebugInfo, INFO_INT);
					return;
			}
			ctrl->handle();
			break;
		}
	}
}

void State_PosCtrl::state_related() {

	switch (ctrl->get_cmd())
	{
		case Command::GrabByFuzzy:
		case Command::GrabByImpedance:
		case Command::GrabByPI:
		case Command::GrabByP:
		case Command::GrabByTry:
		{
			ctrl->trans(FORCE_CTRL);
			break;
		}
		default:
			ctrl->handle();
	}
}

void Controller::wait_print_sensor(unsigned long time)
{
    unsigned long start = millis();
    servo.shut();
    while (millis() - start < time)
    {
        Serial.println(sensor.read_load());
    }
    servo.launch();
}

void Controller::wait_print_pos(unsigned long time)
{
    unsigned long start = millis();
    while (millis() - start < time)
    {
        servo.print(servo.readPosition());
    }
}

void Controller::debug_report(Command cmd, int val) {
	up_conn.feed_back(cmd, val);
}


void Controller::launch(const double speed){
    servo.mov_speed(servo.max_ang,speed);
	while (sensor.read_load() < 0.5 && !is_interruptted()) {
		tick();
	}
}

void Controller::test_p(const double fd, const double v0){
    kp=15;
    servo.open_claw(sensor);
    alarm.start();
    double uk = kp * fd;
    double fk = 0;
    double error = 0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    while (true)
    {
        fk = sensor.read_load();
        error = fd - fk;
        uk = kp * error;
        servo.print(uk);
        delay(10);
    }
   alarm.success();
}


void Controller::check_bus() {
	 up_conn.check();
}

Command  Controller::get_cmd() {
	return up_conn.rcv_cmd;
}

void Controller::init() {
	servo.open_claw();
}
void Controller::publish() 
{
	if (subscribe[0]) {
		up_conn.feed_back(Command::ReadPos, servo.readPositionSafe());
	}
	if (subscribe[1]) {
		up_conn.feed_back(Command::ReadForce,sensor.readForceSafe());
		//up_conn.feed_back(Command::ReadForce, m_ViceSensor.readForceSafe());
		//up_conn.feed_back(Command::DebugValueInt, analogRead(2));
	}

	if (subscribe[2]) {
		up_conn.feed_back(Command::ReadSpeed, servo.readSpeedSafe());
	}
}

void Controller::handle() {
	CurState->is_trans = false;//已经处理了，说明进入了当前状态
	is_interrupt = false;
	//bToBeHandled = false;//同上
	const Command& rcv_cmd = up_conn.rcv_cmd;
	switch (rcv_cmd)
	{
	case xzj::Command::Undefined:
		return;
	case xzj::Command::OpenClaw:servo.open_claw(&dCurSpeed);
		break;
	case xzj::Command::CloseClaw:servo.close_claw(&dCurSpeed);
		break;
	case xzj::Command::Stop:stop_claw();
		break;
	case xzj::Command::Unload:servo.torqueStatus(OFF);
		break;
	case xzj::Command::Load:servo.torqueStatus(ON);
		break;
	case xzj::Command::ReadPos: {
		up_conn.feed_back(Command::ReadPos, servo.readPositionSafe());
		break;
	}
	case Command::Subscribe:{
		handle_subscribe();
		break;
	}
	case Command::MoveTo: {
		tar_pos = up_conn.rcv_data;
		servo.moveSpeed(tar_pos, cur_speed);
		break; 
	}
	case Command::SetSpeed: {
		cur_speed = up_conn.rcv_data;
		dCurSpeed = Adaptor::rec_speed(cur_speed);
		if (servo.moving()==1&&CurState!=szStates[FORCE_CTRL])
			servo.moveSpeed(tar_pos, cur_speed);
		break;
	}

	case Command::ReadForce: {
		up_conn.feed_back(Command::ReadForce, sensor.readForceSafe());
		break;
	}
	
	case Command::ReadSpeed: {
		up_conn.feed_back(Command::ReadSpeed, servo.readSpeedSafe());
		break;
	}
	case Command::GrabByFuzzy: {
		grab_by_fuzzy(Adaptor::sensor_i2d(up_conn.rcv_data));
		break;
	}
	case Command::GrabByP: {
		grab_by_p(Adaptor::sensor_i2d(up_conn.rcv_data));
		break;
	}
	case Command::GrabByPI: {
		grab_by_pi(Adaptor::sensor_i2d(up_conn.rcv_data));
		break;
	}
	case Command::GrabByTry: {
		grab_by_try(Adaptor::sensor_i2d(up_conn.rcv_data));
		break;
	}
	case Command::SetForce: {
		fd = (Adaptor::sensor_i2d(up_conn.rcv_data));
		break;
	}

	case Command::SetPCtrl_P: {
		dPCtrl_P = Adaptor::format_i2d(up_conn.rcv_data);
		//debug_report(Command::DebugValueDouble, dPCtrl_P / 100.0 * 1023);
		break;
	}
	case Command::SetPCtrl_Sample: {
		iPCtrl_Sample = up_conn.rcv_data;
		//debug_report(Command::DebugValueInt, iPCtrl_Sample);
		break;
	}
	case Command::SetPICtrl_P: {
		dPICtrl_P = Adaptor::format_i2d(up_conn.rcv_data); 
		//debug_report(Command::DebugValueDouble, dPICtrl_P / 100.0 * 1023);
		break;
	}
	case Command::SetPICtrl_I: {
		dPICtrl_I = Adaptor::format_i2d(up_conn.rcv_data); 
		//debug_report(Command::DebugValueDouble, dPICtrl_I / 100.0 * 1023);
		break;
	}
	case Command::SetPICtrl_Sample: {
		iPICtrl_Sample = up_conn.rcv_data; 
		//debug_report(Command::DebugValueInt, iPICtrl_Sample);
		break;
	}
	case Command::SetFuzzy_Sample: {
		iFuzzy_Sample = up_conn.rcv_data;
		//debug_report(Command::DebugValueInt, iPICtrl_Sample);
		break;
	}
	case Command::MoveSpeed: {
		int data = up_conn.rcv_data;
		int pos = (data & 0x800) == 0 ? servo.iMaxAng : servo.iMinAng;
		int cnt = 0;
		double fk;
		Fuzzy_controller fuzzy;
		double fd = 3;
		while (++cnt < 100)
		{
			fk = sensor.read_load();
			double error = fd - fk;
			double uk = -dPCtrl_P * error;
			double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
			int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
			int iRet = servo.moveSpeed(pos, data & 0x7ff);
			double val = stable_judge_fuzzy(fd, fk);
			is_stable(val);
			Delay(iPCtrl_Sample);
		}
		debug_report(Command::DebugInfo, INFO_HINT);
		//debug_report(Command::DebugValueInt, iRet);
		break;
	}
								
	default:
		break;
	}
}
void Controller::clear_stable() {
	stable_cnt = 0;
	bIsStable = true;
}
bool Controller::stable_judge(double e,double de) 
{
	if ( abs(e) > 0.13)
	{
		stable_cnt = 0;
		debug_report(Command::DebugState, STATE_UNSTABLE);
		bIsStable = false;
		return false;
	}
	if (!bIsStable &&abs(e) < 0.05) 
	{
		stable_cnt +=1;
		//debug_report(Command::DebugState, STATE_STABLE);
		//bIsStable = true;
		//return true;
	}
	de = abs(de);
	if (de < 0.05)
	{
		if (stable_cnt < 15)
		{
			++stable_cnt;
		}
		if (!bIsStable && stable_cnt >= 15)
		{
			debug_report(Command::DebugState, STATE_STABLE);
			bIsStable = true;
		}
	}
	else
	{
		if (stable_cnt > 0) {
			stable_cnt -= (de > 0.06 ? 3 : 2);
		}
		if( stable_cnt<0  && bIsStable)
		{
			stable_cnt = 0;
			debug_report(Command::DebugState, STATE_UNSTABLE);
			bIsStable = false;
		}
	}
	return bIsStable;
}

double Controller::stable_judge_fuzzy(const double& fd, const double&fk)
{
	static Fuzzy_controller fuzzy(2, 0.5, 1);
	return fuzzy.realize(fd, fk);
}


void Controller::grab_by_p(const double fd_deprecated, const double v0)
{
    open_claw();
    alarm.start();

    double uk = dPCtrl_P * fd;
    double fk = 0;
    double error = 0,de=0,pre=0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);
	clear_stable();
	clear_indices();
	debug_report(Command::DebugState, STATE_LOOP);
    while (!is_interruptted())
    {
        fk = sensor.read_load();
        error = fd - fk;
        uk =-dPCtrl_P * error;
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
	/*	debug_report(Command::DebugInfo, INFO_HINT)*/;
	/*	
		debug_report(Command::DebugValueDouble, abs(uk_servo) / 100.0 * 1023);*/
		int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
		debug_report(Command::DebugValueInt, iSpeed);
		debug_report(Command::DebugChar, uk_servo > 0.0 ? '+' : '-');
        int iRet=servo.mov_speed((uk_servo>0?max_angle:min_angle),iSpeed);

		double val=stable_judge_fuzzy(fd, fk);
		Delay(iPCtrl_Sample, iSpeed);
		while (is_stable(val)==  STATE_STABLE  )
		{
			hold_on(true);
			fk = sensor.read_load();
			val = stable_judge_fuzzy(fd, fk);
			Delay(100, 30);
			if (is_interruptted())
			{
				debug_report(Command::DebugState, EXIT_LOOP);
				return;
			}
		}
		hold_on(false);
    }
	debug_report(Command::DebugState, EXIT_LOOP);
}

void Controller::set_pi(double _kp,double _ki){
    kp=_kp;
    ki=_ki;
}

void  Controller::grab_by_try(const double exp_force, const double v0) {
	open_claw();
	alarm.start();
	launch(v0);
	hold_lightly();
}

void Controller::clear_indices() {
	for (auto&& i : indices) {
		i = 1;
	}
	is_stable_pre_time = millis();
}

void Controller::add_fd(double val) {
	static unsigned long pre_time = millis();
	unsigned long cur_time = millis();
	if (cur_time - pre_time < 100)
	{
		return;
	}
	pre_time = cur_time;
	fd = min(6, fd += val);
}

int Controller::is_stable(double indicator ,bool is_add_fd)
{
	static int pre_state = STATE_STABLE;
	static bool is_overshoot = false;
	//debug_report(Command::ReportForceTiny, (indicator + 1) / 1023);
	double sum = abs(indicator);
	for (int i = 0; i < 9; ++i)
	{
		indices[i] = indices[i + 1];
		sum += indices[i];
	}
	indices[9] = abs(indicator);
	double average = sum / 10;
	bIsStable= (average < 0.06);
	if (bIsStable) 
	{
		debug_report(Command::DebugState, STATE_STABLE);
		is_overshoot = false;
		return pre_state= STATE_STABLE;
	}
	else
	{
		unsigned long cur_time = millis();
		if (abs(cur_time - is_stable_pre_time) < 800)
		{
			debug_report(Command::ReportForceTiny, (indicator + 1) * 1023);
			if (indicator > 0)
			{
				if (is_overshoot)
				{
					is_overshoot = false;
					return pre_state = STATE_SLIP;//避免由于调整引发的问题
				}
				if (is_add_fd)
				{
					add_fd(0.1);
					debug_report(Command::ReportNewForce, (fd / 10 * 1023));
				}
				debug_report(Command::DebugState, STATE_SLIP);
				return pre_state = STATE_SLIP;
			}
			else
			{
				debug_report(Command::DebugState, STATE_UNSTABLE);
				is_overshoot = true;
				return pre_state = STATE_UNSTABLE;
			}
		}
		is_stable_pre_time = cur_time;
	}
	debug_report(Command::DebugState, STATE_STABLE_1);
	return pre_state = STATE_STABLE_1;
}

void Controller::hold_lightly() {
	Fuzzy_controller fuzzy;
	double uk = 0;
	double fk = 0;
	fd = hold_force;
	double max_angle = servo.max_ang;
	double min_angle = servo.min_ang;
	
	debug_report(Command::DebugState, STATE_LOOP);

	double e=fd, pre=0,de=fd;
	clear_stable();
	clear_indices();
	while (!is_interruptted() && (is_stable(stable_judge_fuzzy(fd, fk)) == STATE_STABLE))
	{
		fk = sensor.read_load();
        e = fd - fk;
        uk =-dPCtrl_P * e;
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
        servo.mov_speed((uk_servo>0?max_angle:min_angle),iSpeed);
		//de = e - pre;
		//pre = e;
		Delay(iPCtrl_Sample, iSpeed);
	}
	debug_report(Command::DebugInfo, INFO_HINT);

	
	while (!is_interruptted()) 
	{
		fk = sensor.read_load();
        e = fd - fk;
        uk =-dPCtrl_P * e;
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
        servo.mov_speed((uk_servo>0?max_angle:min_angle),iSpeed);
		//de = e - pre;
		//pre = e;
		double indicator = stable_judge_fuzzy(fd, fk);
		is_stable(indicator,true);
		
		Delay(iPCtrl_Sample, iSpeed);
	}
}

void Controller::test_pi(const double fd, const double v0)
{
    servo.open_claw(sensor);
    alarm.start();
    kp=10;ki=0.5;
    double vk_old = ki * fd;
    double uk = 0;
    double fk = 0;
    double error_old = fd, error = fd;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;

    while (true)
    {
        fk = sensor.read_load();
        error = fd - fk;

        uk += kp * (error - error_old) + ki * error;

        error_old=error;
        delay(50);
        servo.print(uk);
    }

    alarm.success();
}   
void Controller::grab_by_pi(const double fd_deprecated, const double v0)
{
	open_claw();
    alarm.start();
    double vk_old = ki * fd;
    double uk = 0;
    double fk = 0;
    double  error = fd,de=0,pre=0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);

	debug_report(Command::DebugState, STATE_LOOP);
    while (!is_interruptted())
    {
        fk = sensor.read_load();
        error = fd - fk;
		if (abs(error) < 0.05) {
			error = 0;
		}
		de = error - pre;
		pre = error;
        uk -= dPICtrl_P * de + dPICtrl_I * error;//末端速度
		//debug_report(Command::DebugInfo, INFO_HINT);
	/*	debug_report(Command::DebugChar, uk_servo > 0.0 ? '+' : '-');
		debug_report(Command::DebugValueDouble, abs(uk_servo) / 100.0 * 1023);*/
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
		debug_report(Command::DebugValueInt, iSpeed);
        int iRet=servo.mov_speed((uk_servo>0?max_angle:min_angle),iSpeed);
		debug_report(Command::DebugValueInt, iSpeed);

		stable_judge(error,de);
		Delay(iPCtrl_Sample, iSpeed);
    }
	debug_report(Command::DebugState,EXIT_LOOP);

    //servo.stop();

    //hold_on(fd);
    //alarm.success();
}


void Controller::open_claw() {
	servo.open_claw(&dCurSpeed);
	while (!servo.is_in_place(servo.iMinAng)) {
		tick();
	}
}

void Controller::grab_by_admit(const double fd){



}
void Controller::Delay(int cnt) {

	int k = 0;
	while (k < cnt&& is_interrupt==false) {
		delay(1);
		tick();
		++k;
	}
}

void Controller::Delay(int cnt, int iSpeed)
{
	if (iSpeed < 8)
	{
		Delay(CmdDelay[iSpeed]);
	}
	else
	{
		Delay(cnt);
	}
}

void Controller::grab_by_fuzzy(const double fd_deprecated,const double v0){
	Fuzzy_controller fuzzy;
	open_claw();
    alarm.start();
    double uk=0;
    double fk=0;
	double error = 0, pre = 0, de = 0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    fuzzy.set_para(1.5,0.2,2);
	launch(v0);
	debug_report(Command::DebugState, STATE_LOOP);
    while(!is_interruptted()) {
        fk=sensor.read_load();
        uk=-fuzzy.realize(fd,fk);
	/*	debug_report(Command::DebugChar, uk > 0.0 ? '+' : '-');
	    debug_report(Command::DebugValueDouble, abs(fd) / 100.0 * 1023);
	    debug_report(Command::DebugValueDouble, abs(fk) / 100.0 * 1023);
	    debug_report(Command::DebugValueDouble, abs(uk) / 100.0 * 1023);*/
		error = fd - fk;
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		int iSpeed = Adaptor::adapt_speed(abs(uk_servo));
        int iRet=servo.mov_speed((uk_servo>0?max_angle:min_angle),iSpeed);
		//debug_report(Command::DebugValueInt, iSpeed);
		de = error - pre;
		pre = error;
		stable_judge(error,de);
		Delay(iPCtrl_Sample, iSpeed);
    }
	debug_report(Command::DebugState,EXIT_LOOP);
}

void xzj::Controller::stop_claw()
{
	/*while (!is_interruptted())
	{
		Delay(5);
		servo.mov_speed(servo.min_ang, 1);
	}*/
	servo.torqueStatus(OFF);
	servo.torqueStatus(ON);
}

void xzj::Controller::hold_on(bool hold)
{
	static bool repete_req = false;
	if (hold == true)
	{
		if (!repete_req)
		{
			stop_claw();
			debug_report(Command::DebugState, STATE_HOLD_ON);
			repete_req = true;
		}
	}
	else
	{
		if (repete_req == true)
		{
			debug_report(Command::DebugState, STATE_EXIT);
		}
		repete_req = false;
	}
}
