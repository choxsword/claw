#include "StateMachine.h"
#include "Upstream.h"
#include "Claw.h"
using namespace xzj;

void IClawState::tick() {
	general();
	state_related();
}

void IClawState::general() {
	static unsigned char tick_cnt = 0;
	if (tick_cnt < 20) {
		++tick_cnt;
		return;
	}
	if (!is_trans)
	{
		ctrl->check_bus();//如果是从别的状态转移过来的，说明之前的指令还没被处理，因此不需要再次读取
	}
	else {
		is_trans = false;
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
	tick_cnt = 0;
	ctrl->publish();
}

void State_Init::state_related(){
	switch (ctrl->get_cmd())
	{
	case Command::Undefined:
		break;
	case Command::Stop:
		break;
	case Command::MoveTo:
		ctrl->trans(POS_CTRL);
	default:
		ctrl->handle();
	}
}

void State_PosCtrl::state_related() {


	switch (ctrl->get_cmd())
	{
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
	const Command& rcv_cmd = up_conn.rcv_cmd;
	switch (rcv_cmd)
	{
	case xzj::Command::Undefined:
		return;
	case xzj::Command::OpenClaw:servo.open_claw();
		break;
	case xzj::Command::CloseClaw:servo.close_claw();
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
		if (servo.moving()==1)
			servo.moveSpeed(tar_pos, cur_speed);
		break;
	}

	case Command::ReadForce: {
		up_conn.feed_back(Command::ReadForce, sensor.readForceSafe());
		break;
	}
	
	case Command::ReadSpeed: {
		up_conn.feed_back(Command::ReadSpeed, servo.readSpeedSafe());
	}

	default:
		break;
	}

}



void Controller::grab_by_p(const double fd, const double v0)
{
    kp=20;
    servo.open_claw(sensor);
    alarm.start();

    double uk = kp * fd;
    double fk = 0;
    double error = 0;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);
//??P??

    while (true)
    {
        fk = sensor.read_load();
         error = fd - fk;
        if (error <= 0.03)
        {
            break;
        }
        uk = kp * error;
        // servo.mov_speed(max_angle, uk);
        servo.mov_speed((uk>0?max_angle:min_angle),abs(uk));
        delay(10);
        servo.print(fk);
    }

    servo.stop();
#ifdef _CLAW_DEBUG_
    servo.print("-----------------  P CONTROL COMPLETE   ------------------");
    servo.print(fk);
   // servo.print(servo.readPosition());
#endif
   hold_on(fd);
   alarm.success();
}

void Controller::set_pi(double _kp,double _ki){
    kp=_kp;
    ki=_ki;
}

void Controller::hold_on(double fd)
{
    int pos = servo.readPosition();
    unsigned long start_time = millis();
    while (millis() - start_time < 5000)
    {
        double  fk = sensor.read_load();
        double error = fd - fk;
        servo.print(fk);
        if (abs(error) > 0.1)
        {
            pos += ((error > 0) ? 1 : -1);
            servo.moveSpeed(pos, 0);

#ifdef _CLAW_DEBUG_
          // servo.print(pos);
          //  servo.print(servo.readPosition());
#endif
            delay(50);
        }
    }

#ifdef _CLAW_DEBUG_
    servo.print("111111111111111111111111111111111");
    servo.print(Adaptor::rec_pos(pos) - 60);
#endif
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
    servo.open_claw(sensor);
    alarm.start();
    kp=10;ki=1;
    double vk_old = ki * fd;
    double uk = 0;
    double fk = 0;
    double error_old = fd, error = fd;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;
    launch(v0);

//??PI??
    while (true)
    {
        fk = sensor.read_load();
        error = fd - fk;

        uk += kp * (error - error_old) + ki * error;

        error_old=error;
        servo.mov_speed((uk>0?max_angle:min_angle),abs(uk));
        delay(50);
        servo.print(fk);
    }

    servo.stop();
#ifdef _CLAW_DEBUG_
    servo.print("-----------------  PI CONTROL COMPLETE   ------------------");
    servo.print(fk);
    servo.print(servo.readPosition());
#endif
    hold_on(fd);
    alarm.success();
}




void Controller::grab_by_admit(const double fd){



}


void Controller::grab_by_fuzzy(const double fd,const double v0){
    servo.open_claw(sensor);
    alarm.start();
    double uk=0;
    double fk=fd;
    double max_angle = servo.max_ang;
    double min_angle=servo.min_ang;

    fuzzy.set_para(fd,0.5,60);
    double e,pre;
    while(true) {
        fk=sensor.read_load();
        uk=fuzzy.realize(fd,fk);
        servo.mov_speed((uk>0?max_angle:min_angle),abs(uk));
        delay(10);
        e=fd-fk;
        //servo.print(uk);
        servo.print(fk);
        pre=e;
    }
}