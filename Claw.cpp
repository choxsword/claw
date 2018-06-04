#include "Claw.h"
#include "DynamixelSerial.h"
using namespace xzj;

int MyServo::mov_to(double pos){
    return DynamixelClass::move(Adaptor::adapt_pos(pos+offset));
}

int MyServo::mov_speed(double pos, double speed){
    return DynamixelClass::moveSpeed(Adaptor::adapt_pos(pos+offset),Adaptor::adapt_speed(speed));
}

double MyServo::read_pos(){
    return Adaptor::rec_pos(DynamixelClass::readPosition())-offset;
}

  double MyServo::read_speed(){
    return Adaptor::rec_speed(DynamixelClass::readSpeed());
}


void MyServo::launch(){
    Serial.end();  
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
   DynamixelClass::setRDT(120);//跟 WAIT_RDT有关
  // DynamixelClass::reset();
    DynamixelClass::torqueStatus(torque_state);
}

unsigned long MyServo::test_com_speed(){
    auto start=micros();
    readSpeed();
    return micros()-start;
}

double Sensor::read_load(){
    int digit=analogRead(pin);
    double volt= static_cast<double>(digit)/1023*5.0;
    double res=p4*pow(volt,4)+ p3*pow(volt,3)+p2*pow(volt,2)+p1*volt+p0;
    return res<0?0:res;
}
double Sensor::read_volt(){
    int digit=analogRead(pin);
    double volt= static_cast<double>(digit)/1023*5.0;
    return volt;
}

void Sensor::alter_threshold(double new_threshold){
    threshold=new_threshold;
}

bool Sensor::is_safe(){
    return read_load()<threshold;
}