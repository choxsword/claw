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
					ctrl->debug_report(Command::DebugInfo, INFO_INT);
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
    while(sensor.read_load()<0.5);
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
void Controller::publish() {
	if (subscribe[0]) {
		up_conn.feed_back(Command::ReadPos, servo.readPositionSafe());
	}
	if (subscribe[1]) {
		up_conn.feed_back(Command::ReadForce,sensor.readForceSafe());
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
	case xzj::Command::Stop:servo.stop();
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
	default:
		break;
	}
}



void Controller::grab_by_p(const double fd, const double v0)
{
    kp=dPCtrl_P;
    open_claw();
    alarm.start();

    double uk = kp * fd;
    double fk = 0;
    double error = 0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);
	debug_report(Command::DebugState, STATE_LOOP);
    while (!is_interruptted())
    {
        fk = sensor.read_load();
         error = fd - fk;
/*		if (abs(error) < 0.05)
			error = 0*/;
        uk =-kp * error;
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		//debug_report(Command::DebugInfo, INFO_HINT);
		//debug_report(Command::DebugValueInt,Adaptor::adapt_speed(abs(uk_servo)));
		//debug_report(Command::DebugValueDouble, servo.read_pos() / 100.0 * 1023);
		/*debug_report(Command::DebugChar, uk_servo > 0.0 ? '+' : '-');
		debug_report(Command::DebugValueDouble, abs(uk_servo) / 100.0 * 1023);*/

        servo.mov_speed((uk_servo>0?max_angle:min_angle),abs(uk_servo));
        Delay(iPCtrl_Sample);
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

void Controller::hold_lightly() {
	Fuzzy_controller fuzzy;
	double uk = 0;
	double fk = 0;
	double fd = hold_force;
	double max_angle = servo.max_ang;
	double min_angle = servo.min_ang;
	fuzzy.set_para(fd, 0.5, 60);
	double e, pre;
	unsigned int stable_cnt = 0;
	while (!is_interruptted() && stable_cnt<20) {
		fk = sensor.read_load();
		uk = fuzzy.realize(fd, fk);
		servo.mov_speed((uk>0 ? max_angle : min_angle), abs(uk));
		Delay(iFuzzy_Sample);
		e = fd - fk;
		if (abs(e) < 0.1)
			++stable_cnt;
	}
	debug_report(Command::DebugState, STATE_STABLE);
	unsigned int  unstable_cnt=0;
	while (!is_interruptted() && stable_cnt<20) {
		fk = sensor.read_load();
		uk = fuzzy.realize(fd, fk);
		servo.mov_speed((uk>0 ? max_angle : min_angle), abs(uk));
		e = fd - fk;
		if (e>0.1) {
			++unstable_cnt;
			if (unstable_cnt > 3) {
				fd += 0.5;
			}
		}
		else {
			unstable_cnt = 0;
		}
		Delay(iFuzzy_Sample);
	}
}

void Controller::hold_on(double fd)
{
	debug_report(Command::DebugInfo, INFO_HOLD_ON);
    int pos = servo.readPosition();
    unsigned long start_time = millis();
    while (millis() - start_time < 5000 && !is_interruptted())
    {
        double  fk = sensor.read_load();
        double error = fd - fk;
        if (abs(error) > 0.1)
        {
            pos += ((error > 0) ? 1 : -1);
            servo.moveSpeed(pos, 0);
            Delay(50);
        }
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
void Controller::grab_by_pi(const double fd, const double v0)
{
	open_claw();
    alarm.start();
    kp=dPICtrl_P;ki=dPICtrl_I;
    double vk_old = ki * fd;
    double uk = 0;
    double fk = 0;
    double error_old = 0, error = fd;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);

	debug_report(Command::DebugState, STATE_LOOP);
    while (!is_interruptted())
    {
        fk = sensor.read_load();
        error = fd - fk;
		//消除抖动
		/*if (abs(error) < 0.05)
			error = 0;*/
        uk -= kp * (error - error_old) + ki * error;//末端速度
		double uk_servo = Adaptor::end_speed2servo_speed(uk, servo.read_pos());
		//debug_report(Command::DebugInfo, INFO_HINT);
		//debug_report(Command::DebugValueDouble, servo.read_pos() / 100.0 * 1023);
		//debug_report(Command::DebugChar, fk > 0.0 ? '+' : '-');
		//debug_report(Command::DebugValueDouble, abs(fk)/ 100.0 * 1023);
		//debug_report(Command::DebugChar, uk_servo > 0.0 ? '+' : '-');
		//debug_report(Command::DebugValueDouble, abs(uk_servo) / 100.0 * 1023);
        error_old=error;
		servo.mov_speed((uk_servo>0 ? max_angle : min_angle), abs(uk_servo));
		Delay(iPICtrl_Sample);
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

void Controller::grab_by_fuzzy(const double fd,const double v0){
	Fuzzy_controller fuzzy;
	open_claw();
    alarm.start();
    double uk=0;
    double fk=0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    fuzzy.set_para(fd,0.5,60);
	debug_report(Command::DebugState, STATE_LOOP);
    while(!is_interruptted()) {
        fk=sensor.read_load();
        uk=fuzzy.realize(fd,fk);

		//debug_report(Command::DebugChar, uk > 0.0 ? '+' : '-');
		//debug_report(Command::DebugValueDouble, abs(uk) / 100.0 * 1023);

        servo.mov_speed((uk>0?max_angle:min_angle),abs(uk));
        Delay(iFuzzy_Sample);
    }
	debug_report(Command::DebugState,EXIT_LOOP);
	/*Fuzzy_controller fuzzy1;
	fuzzy1.set_para(3, 3, 3);
	debug_report(Command::DebugValueDouble, fuzzy1.get_e_pre()/100.0*1023);
	debug_report(Command::DebugValueDouble, fuzzy1.realize(2.3,1)/ 100.0 * 1023);
	debug_report(Command::DebugInfo, INFO_HINT);
	for(int i=0;i<10;++i)
		debug_report(Command::DebugValueDouble,a[i]/100.0*1023);*/

}