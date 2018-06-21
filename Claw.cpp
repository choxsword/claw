#include "Claw.h"
#include "DynamixelSerial.h"

using namespace xzj;

int MyServo::mov_to(double pos){
    return DynamixelClass::move(Adaptor::adapt_pos(pos+offset));
}

int MyServo::mov_speed(double pos, double speed){
    int hex_speed=Adaptor::adapt_speed(speed);
    if(hex_speed==0){
       // stop();
        return -10;
    }
    return DynamixelClass::moveSpeed(Adaptor::adapt_pos(pos+offset),hex_speed);
}

double MyServo::read_pos(){
    return Adaptor::rec_pos(DynamixelClass::readPosition())-offset;
}

  double MyServo::read_speed(){
    return Adaptor::rec_speed(DynamixelClass::readSpeed());
}


void MyServo::launch(){
    DynamixelClass::begin(baud,pin);
}

void MyServo::shut(){
    DynamixelClass::end();
    Serial.begin(serial_baud);
}

int MyServo::read_load(){
    return DynamixelClass::readLoad();
}

void MyServo::stop(){
    stop(read_pos());
}

void MyServo::stop(double pos){
    mov_to(pos);
}

void MyServo::init(int torque_state){
    launch();
   // DynamixelClass::reset();
   DynamixelClass::setRDT(150);//跟 WAIT_RDT有关
    DynamixelClass::torqueStatus(torque_state);
}

unsigned long MyServo::test_com_speed(){
    auto start=micros();
    readSpeed();
    return micros()-start;
}
void MyServo::open_claw(Sensor& sensor){
    double pos=0;
    mov_speed(min_ang,60);
  do {
    pos = read_pos();
    print(sensor.read_load());
  } while (abs(pos - min_ang) > 1);
  delay(wait_position);
}

void MyServo::open_claw(double* speed){
    if(speed==nullptr)
        mov_to(min_ang);
    else
        mov_speed(min_ang,*speed);
}

void MyServo::close_claw(double*speed){
    if(speed==nullptr)
        mov_to(max_ang);
    else
        mov_speed(max_ang,*speed);
}

bool MyServo::is_in_place(const double pos){
    return abs(read_pos()-pos)<1.0;
}

double Sensor::read_load(){
    int digit=analogRead(pin);
    double volt= static_cast<double>(digit)/1023*5.0;
    double res = p4 * pow(volt, 4) + p3 * pow(volt, 3) + p2 * pow(volt, 2) + p1 * volt + p0;
    return res < 0 ? 0 : res;
}

double Sensor::read_volt()
{
    int digit = analogRead(pin);
    double volt = static_cast<double>(digit) / 1023 * 5.0;
    return volt;
}

void Sensor::alter_threshold(double new_threshold)
{
    threshold = new_threshold;
}

bool Sensor::is_safe()
{
    return read_load() < threshold;
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



void Controller::check() {
	
	Command rcv_cmd = up_conn.check();
	switch (rcv_cmd)
	{
	case xzj::Command::Undefined:
		break;
	case xzj::Command::OpenClaw:servo.open_claw();
		break;
	case xzj::Command::CloseClaw:
		break;
	case xzj::Command::Stop:
		break;
	case xzj::Command::Unload:servo.torqueStatus(OFF);
		break;
	case xzj::Command::Load:servo.torqueStatus(ON);
		break;
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
